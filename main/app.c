// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — tâche principale et état partagé.
// Enchaînement : Wi-Fi enregistré ? -> connexion -> géolocalisation par l'IP publique -> radar.
// Sans réseau enregistré, en cas d'échec de connexion, ou sur demande (appui long sur l'encodeur) :
// portail de configuration sur le point d'accès RadarAvion_Wifi.
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "app.h"
#include "net.h"
#include "adsb.h"

#define TAG "APP"

#define STA_CONNECT_TIMEOUT_MS  30000
#define GEOLOC_RETRY_MS         10000

static SemaphoreHandle_t s_lock;
static app_status_t s_status;

// ---------------------------------------------------------------------------------------------
// État partagé
// ---------------------------------------------------------------------------------------------
void app_status_get(app_status_t *out) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_lock);
}

void app_status_set_phase(app_phase_t phase, const char *info) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.phase = phase;
    strlcpy(s_status.info, info ? info : "", sizeof(s_status.info));
    xSemaphoreGive(s_lock);
}

void app_status_set_ssid(const char *ssid, bool saved) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    strlcpy(s_status.ssid, ssid, sizeof(s_status.ssid));
    s_status.has_saved_ssid = saved;
    xSemaphoreGive(s_lock);
}

void app_status_set_pos(double lat, double lon, const char *city) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.lat = lat;
    s_status.lon = lon;
    s_status.has_pos = true;
    strlcpy(s_status.city, city ? city : "", sizeof(s_status.city));
    xSemaphoreGive(s_lock);
}

void app_status_set_wifi(bool ok, const char *ip) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.wifi_ok = ok;
    strlcpy(s_status.ip, ip ? ip : "", sizeof(s_status.ip));
    xSemaphoreGive(s_lock);
}

void app_status_set_portal_clients(int n) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.portal_clients = n;
    xSemaphoreGive(s_lock);
}

void app_request_portal(void) {
    app_config_set_force_portal(true);
    esp_restart();
}

// cJSON (réponses ADS-B : des milliers de petits objets) -> PSRAM, pour préserver la RAM interne
static void *json_malloc(size_t sz) {
    return heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

// ---------------------------------------------------------------------------------------------
// Tâche principale
// ---------------------------------------------------------------------------------------------
static void app_task(void *arg) {
    app_config_t cfg;
    app_config_load(&cfg);
    app_status_set_ssid(cfg.ssid, cfg.ssid[0] != 0);
    wifi_init_common();

    if (app_config_take_force_portal()) {
        wifi_portal_run("Configuration demandée", cfg.ssid[0] != 0);
    }
    if (!cfg.ssid[0]) {
        wifi_portal_run("Aucun réseau Wi-Fi enregistré", false);
    }

    char msg[112];
    snprintf(msg, sizeof(msg), "Connexion à « %s »", cfg.ssid);
    app_status_set_phase(APP_PHASE_CONNECTING, msg);
    if (!wifi_sta_connect(cfg.ssid, cfg.pass, STA_CONNECT_TIMEOUT_MS)) {
        snprintf(msg, sizeof(msg), "Connexion à « %s » impossible", cfg.ssid);
        wifi_portal_run(msg, true);
    }

    // Position : saisie manuelle prioritaire, sinon déduite de l'adresse IP publique
    if (cfg.has_manual_pos) {
        app_status_set_pos(cfg.lat, cfg.lon, "Position manuelle");
        ESP_LOGI(TAG, "Position manuelle : %.4f, %.4f", cfg.lat, cfg.lon);
    } else {
        app_status_set_phase(APP_PHASE_LOCATING, "Localisation par l'adresse IP");
        double lat, lon;
        char city[48];
        while (!geoloc_by_ip(&lat, &lon, city, sizeof(city))) {
            app_status_set_phase(APP_PHASE_LOCATING, "Géolocalisation indisponible, nouvel essai...");
            vTaskDelay(pdMS_TO_TICKS(GEOLOC_RETRY_MS));
        }
        app_status_set_pos(lat, lon, city);
        cfg.lat = lat;
        cfg.lon = lon;
    }

    app_status_set_phase(APP_PHASE_RADAR, "");
    adsb_start(cfg.lat, cfg.lon);
    ESP_LOGI(TAG, "Radar demarre (RAM interne libre : %u o)", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    vTaskDelete(NULL);
}

void app_start(void) {
    s_lock = xSemaphoreCreateMutex();
    cJSON_Hooks hooks = { .malloc_fn = json_malloc, .free_fn = free };
    cJSON_InitHooks(&hooks);
    app_status_set_phase(APP_PHASE_BOOT, "Démarrage");
    // Pile 8 Ko : HTTPS (géolocalisation de secours) et serveur du portail démarrés depuis cette tâche
    xTaskCreatePinnedToCore(app_task, "app", 8 * 1024, NULL, 5, NULL, 0);
}
