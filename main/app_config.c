// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — configuration persistante en NVS (identifiants Wi-Fi, position, zoom)
#include <string.h>
#include <stdlib.h>
#include "nvs.h"
#include "esp_log.h"
#include "app.h"

#define TAG "CONFIG"
#define NVS_NS "radar"

// Lit un texte NVS dans dst (vide si absent)
static void nvs_get_text(nvs_handle_t h, const char *key, char *dst, size_t cap) {
    size_t len = cap;
    if (nvs_get_str(h, key, dst, &len) != ESP_OK) {
        dst[0] = 0;
    }
}

// Accepte "48.85" comme "48,85" (clavier français)
static bool parse_coord(const char *txt, double min, double max, double *out) {
    if (!txt || !txt[0]) {
        return false;
    }
    char tmp[24];
    strlcpy(tmp, txt, sizeof(tmp));
    for (char *p = tmp; *p; p++) {
        if (*p == ',') {
            *p = '.';
        }
    }
    char *end = NULL;
    double v = strtod(tmp, &end);
    if (end == tmp || v < min || v > max) {
        return false;
    }
    *out = v;
    return true;
}

void app_config_load(app_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->zoom_idx = -1;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;   // premier démarrage : rien d'enregistré
    }
    nvs_get_text(h, "ssid", cfg->ssid, sizeof(cfg->ssid));
    nvs_get_text(h, "pass", cfg->pass, sizeof(cfg->pass));
    char lat[24], lon[24];
    nvs_get_text(h, "lat", lat, sizeof(lat));
    nvs_get_text(h, "lon", lon, sizeof(lon));
    cfg->has_manual_pos = parse_coord(lat, -90, 90, &cfg->lat) && parse_coord(lon, -180, 180, &cfg->lon);
    int32_t zoom = -1;
    if (nvs_get_i32(h, "zoom", &zoom) == ESP_OK) {
        cfg->zoom_idx = zoom;
    }
    nvs_close(h);
}

esp_err_t app_config_save_wifi(const char *ssid, const char *pass, const char *lat, const char *lon) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    double d;
    // Coordonnées invalides ou incomplètes : on les ignore (géolocalisation IP)
    bool pos_ok = parse_coord(lat, -90, 90, &d) && parse_coord(lon, -180, 180, &d);
    err = nvs_set_str(h, "ssid", ssid);
    if (err == ESP_OK) err = nvs_set_str(h, "pass", pass ? pass : "");
    if (err == ESP_OK) err = nvs_set_str(h, "lat", pos_ok ? lat : "");
    if (err == ESP_OK) err = nvs_set_str(h, "lon", pos_ok ? lon : "");
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Wi-Fi enregistre : '%s' (position %s)", ssid, pos_ok ? "manuelle" : "par IP");
    return err;
}

void app_config_save_zoom(int idx) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "zoom", idx);
        nvs_commit(h);
        nvs_close(h);
    }
}

void app_config_set_force_portal(bool on) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "portal", on ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool app_config_take_force_portal(void) {
    nvs_handle_t h;
    uint8_t v = 0;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    if (nvs_get_u8(h, "portal", &v) == ESP_OK && v) {
        nvs_set_u8(h, "portal", 0);
        nvs_commit(h);
    }
    nvs_close(h);
    return v != 0;
}
