// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — requêtes HTTP(S) et géolocalisation par l'adresse IP publique
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "net.h"

#define TAG "HTTP"

#define HTTP_MAX_BODY (768 * 1024)   // garde-fou : au-delà, la réponse est tronquée -> erreur

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
    bool overflow;
} http_body_t;

// Accumule le corps (déjà décodé si "chunked") dans un tampon en PSRAM qui grandit au besoin
static esp_err_t http_event(esp_http_client_event_t *evt) {
    http_body_t *body = (http_body_t *)evt->user_data;
    if (evt->event_id != HTTP_EVENT_ON_DATA || body->overflow) {
        return ESP_OK;
    }
    size_t need = body->len + evt->data_len + 1;
    if (need > body->cap) {
        size_t cap = body->cap ? body->cap : 16 * 1024;
        while (cap < need) {
            cap *= 2;
        }
        if (cap > HTTP_MAX_BODY) {
            body->overflow = true;
            return ESP_OK;
        }
        char *nb = heap_caps_realloc(body->buf, cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!nb) {
            body->overflow = true;
            return ESP_OK;
        }
        body->buf = nb;
        body->cap = cap;
    }
    memcpy(body->buf + body->len, evt->data, evt->data_len);
    body->len += evt->data_len;
    body->buf[body->len] = 0;
    return ESP_OK;
}

int net_http_get(const char *url, char **out, size_t *out_len, int timeout_ms) {
    *out = NULL;
    if (out_len) {
        *out_len = 0;
    }
    http_body_t body = {0};
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = timeout_ms,
        .event_handler = http_event,
        .user_data = &body,
        .user_agent = "RadarAvion-ESP32/1.0",
        .crt_bundle_attach = esp_crt_bundle_attach,   // certificats racine embarqués (HTTPS)
        .buffer_size = 2048,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return -1;
    }
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || body.overflow || !body.buf) {
        ESP_LOGW(TAG, "GET %s : %s%s", url, esp_err_to_name(err), body.overflow ? " (reponse trop grande)" : "");
        free(body.buf);
        return -1;
    }
    *out = body.buf;
    if (out_len) {
        *out_len = body.len;
    }
    return status;
}

// Lit un nombre JSON (absent -> false)
static bool json_num(const cJSON *obj, const char *key, double *out) {
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!cJSON_IsNumber(it)) {
        return false;
    }
    *out = it->valuedouble;
    return true;
}

static void json_str(const cJSON *obj, const char *key, char *dst, size_t cap) {
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(it) && it->valuestring) {
        strlcpy(dst, it->valuestring, cap);
    }
}

// Interroge un service de géolocalisation IP ; ok_key / lat_key / lon_key dépendent du service
static bool geoloc_try(const char *url, const char *ok_key, const char *ok_value,
                       const char *lat_key, const char *lon_key,
                       double *lat, double *lon, char *city, size_t city_len) {
    char *txt = NULL;
    int status = net_http_get(url, &txt, NULL, 8000);
    if (status != 200 || !txt) {
        free(txt);
        return false;
    }
    bool ok = false;
    cJSON *root = cJSON_Parse(txt);
    if (root) {
        const cJSON *flag = cJSON_GetObjectItemCaseSensitive(root, ok_key);
        bool success = ok_value ? (cJSON_IsString(flag) && strcmp(flag->valuestring, ok_value) == 0)
                                : cJSON_IsTrue(flag);
        if (success && json_num(root, lat_key, lat) && json_num(root, lon_key, lon)) {
            city[0] = 0;
            json_str(root, "city", city, city_len);
            ok = true;
        }
        cJSON_Delete(root);
    }
    free(txt);
    return ok;
}

bool geoloc_by_ip(double *lat, double *lon, char *city, size_t city_len) {
    // ip-api.com : gratuit, sans clé, HTTP uniquement (pas de TLS : rapide sur l'ESP32)
    if (geoloc_try("http://ip-api.com/json/?fields=status,lat,lon,city&lang=fr",
                   "status", "success", "lat", "lon", lat, lon, city, city_len)) {
        ESP_LOGI(TAG, "Geolocalisation ip-api : %.4f, %.4f (%s)", *lat, *lon, city);
        return true;
    }
    // Secours : ipwho.is (HTTPS)
    if (geoloc_try("https://ipwho.is/?fields=success,latitude,longitude,city",
                   "success", NULL, "latitude", "longitude", lat, lon, city, city_len)) {
        ESP_LOGI(TAG, "Geolocalisation ipwho.is : %.4f, %.4f (%s)", *lat, *lon, city);
        return true;
    }
    ESP_LOGW(TAG, "Geolocalisation IP impossible");
    return false;
}
