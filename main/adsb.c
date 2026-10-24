// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — tâche de récupération des avions autour de la position.
// API ouvertes, sans clé : https://opendata.adsb.fi/api/v2/lat/{lat}/lon/{lon}/dist/{NM} (limite 1 req/s)
// et en secours https://api.adsb.lol/v2/point/{lat}/{lon}/{NM}. Réponse JSON au format readsb.
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "adsb.h"
#include "net.h"

#define TAG "ADSB"

#define FETCH_PERIOD_MS   5000    // rafraîchissement normal
#define RETRY_PERIOD_MS   10000   // après une erreur
#define MIN_GAP_MS        1500    // jamais plus d'une requête toutes les 1,5 s (limite adsb.fi : 1/s)
#define MAX_POS_AGE_S     60.0f   // position plus vieille : avion ignoré

static adsb_snapshot_t *s_pub;    // dernière donnée publiée (lue par l'interface)
static adsb_snapshot_t *s_work;   // donnée en cours de construction
static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static double s_lat, s_lon;
static volatile float s_radius_km = 40.0f;
static float s_fetched_km;

static float dist_km(float lat, float lon) {
    float dx = (lon - (float)s_lon) * 111.32f * cosf((float)s_lat * (float)M_PI / 180.0f);
    float dy = (lat - (float)s_lat) * 110.574f;
    return sqrtf(dx * dx + dy * dy);
}

static void copy_str(const cJSON *obj, const char *key, char *dst, size_t cap) {
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    dst[0] = 0;
    if (cJSON_IsString(it) && it->valuestring) {
        strlcpy(dst, it->valuestring, cap);
        // l'indicatif est complété par des espaces ("AFR7419 ")
        for (int i = strlen(dst) - 1; i >= 0 && dst[i] == ' '; i--) {
            dst[i] = 0;
        }
    }
}

static bool get_num(const cJSON *obj, const char *key, double *out) {
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!cJSON_IsNumber(it)) {
        return false;
    }
    *out = it->valuedouble;
    return true;
}

// Remplit s_work->ac depuis le JSON ; garde les ADSB_MAX_AC avions les plus proches
static bool parse(const char *txt) {
    cJSON *root = cJSON_Parse(txt);
    if (!root) {
        return false;
    }
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "aircraft");   // adsb.fi v2
    if (!cJSON_IsArray(arr)) {
        arr = cJSON_GetObjectItemCaseSensitive(root, "ac");                  // adsb.lol / readsb v2
    }
    if (!cJSON_IsArray(arr)) {
        cJSON_Delete(root);
        return false;
    }
    static float dists[ADSB_MAX_AC];
    int n = 0;
    const cJSON *it;
    cJSON_ArrayForEach(it, arr) {
        double lat, lon, v;
        if (!get_num(it, "lat", &lat) || !get_num(it, "lon", &lon)) {
            continue;
        }
        float age = get_num(it, "seen_pos", &v) ? (float)v : 0.0f;
        if (age > MAX_POS_AGE_S) {
            continue;
        }
        float d = dist_km((float)lat, (float)lon);
        int slot = n;
        if (n == ADSB_MAX_AC) {
            // tableau plein : remplace l'avion le plus éloigné s'il est plus loin que celui-ci
            int far = 0;
            for (int i = 1; i < n; i++) {
                if (dists[i] > dists[far]) {
                    far = i;
                }
            }
            if (dists[far] <= d) {
                continue;
            }
            slot = far;
        } else {
            n++;
        }
        adsb_ac_t *a = &s_work->ac[slot];
        memset(a, 0, sizeof(*a));
        dists[slot] = d;
        copy_str(it, "hex", a->hex, sizeof(a->hex));
        copy_str(it, "flight", a->flight, sizeof(a->flight));
        copy_str(it, "t", a->type, sizeof(a->type));
        copy_str(it, "r", a->reg, sizeof(a->reg));
        a->lat = (float)lat;
        a->lon = (float)lon;
        a->pos_age_s = age;
        const cJSON *alt = cJSON_GetObjectItemCaseSensitive(it, "alt_baro");
        if (cJSON_IsString(alt) && alt->valuestring && strcmp(alt->valuestring, "ground") == 0) {
            a->on_ground = true;
            a->has_alt = true;
            a->alt_ft = 0;
        } else if (cJSON_IsNumber(alt)) {
            a->has_alt = true;
            a->alt_ft = (int32_t)alt->valuedouble;
        } else if (get_num(it, "alt_geom", &v)) {
            a->has_alt = true;
            a->alt_ft = (int32_t)v;
        }
        if (get_num(it, "gs", &v)) {
            a->gs_kt = (float)v;
        }
        if (get_num(it, "track", &v) || get_num(it, "true_heading", &v)) {
            a->track_deg = (float)v;
            a->has_track = true;
        }
        if (get_num(it, "baro_rate", &v) || get_num(it, "geom_rate", &v)) {
            a->vrate_fpm = (int16_t)fmax(-32000, fmin(32000, v));
        }
    }
    cJSON_Delete(root);
    s_work->count = n;
    return true;
}

// Reprend la traînée de chaque avion déjà connu et y ajoute sa position précédente
static void merge_trails(void) {
    for (int i = 0; i < s_work->count; i++) {
        adsb_ac_t *a = &s_work->ac[i];
        for (int j = 0; j < s_pub->count; j++) {
            const adsb_ac_t *o = &s_pub->ac[j];
            if (strcmp(a->hex, o->hex) != 0) {
                continue;
            }
            a->trail_n = o->trail_n;
            memcpy(a->trail_lat, o->trail_lat, sizeof(a->trail_lat));
            memcpy(a->trail_lon, o->trail_lon, sizeof(a->trail_lon));
            if (fabsf(o->lat - a->lat) > 1e-4f || fabsf(o->lon - a->lon) > 1e-4f) {
                if (a->trail_n == ADSB_TRAIL) {
                    memmove(a->trail_lat, a->trail_lat + 1, (ADSB_TRAIL - 1) * sizeof(float));
                    memmove(a->trail_lon, a->trail_lon + 1, (ADSB_TRAIL - 1) * sizeof(float));
                    a->trail_n--;
                }
                a->trail_lat[a->trail_n] = o->lat;
                a->trail_lon[a->trail_n] = o->lon;
                a->trail_n++;
            }
            break;
        }
    }
}

// fmt_url : gabarit avec, dans l'ordre, latitude, longitude et rayon (NM)
static bool fetch(const char *fmt_url, const char *source, int nm) {
    char url[160];
    snprintf(url, sizeof(url), fmt_url, s_lat, s_lon, nm);
    char *txt = NULL;
    size_t len = 0;
    int64_t t0 = esp_timer_get_time();
    int status = net_http_get(url, &txt, &len, 12000);
    if (status != 200 || !txt) {
        ESP_LOGW(TAG, "%s : HTTP %d", source, status);
        free(txt);
        return false;
    }
    bool ok = parse(txt);
    free(txt);
    if (ok) {
        strlcpy(s_work->source, source, sizeof(s_work->source));
        ESP_LOGI(TAG, "%s : %d avions (%d NM, %u octets, %lld ms)", source, s_work->count, nm,
                 (unsigned)len, (esp_timer_get_time() - t0) / 1000);
    } else {
        ESP_LOGW(TAG, "%s : JSON invalide", source);
    }
    return ok;
}

static void adsb_task(void *arg) {
    int64_t last_req = 0;
    for (;;) {
        if (!wifi_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        int64_t gap = (esp_timer_get_time() - last_req) / 1000;
        if (gap < MIN_GAP_MS) {
            vTaskDelay(pdMS_TO_TICKS(MIN_GAP_MS - gap));
        }
        last_req = esp_timer_get_time();

        float km = s_radius_km;
        // rayon du cercle affiché + 5 % (avions qui entrent dans le champ), en milles nautiques
        int nm = (int)ceilf(km * 1.05f / 1.852f) + 1;
        if (nm > 250) {
            nm = 250;
        }
        bool ok = fetch("https://opendata.adsb.fi/api/v2/lat/%.4f/lon/%.4f/dist/%d", "adsb.fi", nm)
               || fetch("https://api.adsb.lol/v2/point/%.4f/%.4f/%d", "adsb.lol", nm);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (ok) {
            merge_trails();
            s_work->seq = s_pub->seq + 1;
            s_work->ok = true;
            s_work->fetch_us = esp_timer_get_time();
            adsb_snapshot_t *t = s_pub;   // échange des tampons : s_work devient la donnée publiée
            s_pub = s_work;
            s_work = t;
            s_fetched_km = km;
        } else {
            s_pub->ok = false;
            s_pub->seq++;
        }
        xSemaphoreGive(s_lock);

        // attend la prochaine période, ou un réveil anticipé (zoom arrière au-delà du rayon chargé)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ok ? FETCH_PERIOD_MS : RETRY_PERIOD_MS));
    }
}

void adsb_start(double lat, double lon) {
    if (s_task) {
        return;
    }
    s_lat = lat;
    s_lon = lon;
    s_pub = heap_caps_calloc(1, sizeof(adsb_snapshot_t), MALLOC_CAP_SPIRAM);
    s_work = heap_caps_calloc(1, sizeof(adsb_snapshot_t), MALLOC_CAP_SPIRAM);
    assert(s_pub && s_work);
    s_lock = xSemaphoreCreateMutex();   // en dernier : adsb_copy_if_new() teste s_lock
    // 8 Ko de pile : cJSON + client HTTPS (mbedTLS alloue ses tampons en PSRAM, cf. sdkconfig.defaults)
    xTaskCreatePinnedToCore(adsb_task, "adsb", 8 * 1024, NULL, 4, &s_task, 0);
}

void adsb_set_radius_km(float km) {
    float before = s_radius_km;
    s_radius_km = km;
    if (s_task && km > before && km > s_fetched_km) {
        xTaskNotifyGive(s_task);
    }
}

bool adsb_copy_if_new(adsb_snapshot_t *dst, uint32_t *seq) {
    if (!s_lock) {
        return false;
    }
    bool fresh = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_pub->seq != *seq) {
        // copie uniquement la partie utile du tableau
        size_t head = offsetof(adsb_snapshot_t, ac) + (size_t)s_pub->count * sizeof(adsb_ac_t);
        memcpy(dst, s_pub, head);
        *seq = s_pub->seq;
        fresh = true;
    }
    xSemaphoreGive(s_lock);
    return fresh;
}
