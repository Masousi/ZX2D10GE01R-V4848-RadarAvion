// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — état partagé de l'application et configuration persistante (NVS).
// L'état (phase, position, Wi-Fi...) est écrit par les tâches réseau et lu par l'interface LVGL :
// toujours passer par ces fonctions (protégées par un mutex), jamais d'appel LVGL hors de la tâche LVGL.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// Nom du point d'accès de configuration (portail Wi-Fi)
#define APP_AP_SSID      "RadarAvion_Wifi"
#define APP_AP_IP        "192.168.4.1"

// ---------------------------------------------------------------------------------------------
// Configuration persistante (NVS, espace "radar")
// ---------------------------------------------------------------------------------------------
typedef struct {
    char ssid[33];
    char pass[65];
    bool has_manual_pos;     // position saisie à la main dans le portail (sinon : géolocalisation IP)
    double lat;
    double lon;
    int zoom_idx;            // dernier niveau de zoom choisi à l'encodeur (-1 = défaut)
} app_config_t;

void app_config_load(app_config_t *cfg);
// lat / lon : texte saisi dans le portail, vide = géolocalisation par l'adresse IP
esp_err_t app_config_save_wifi(const char *ssid, const char *pass, const char *lat, const char *lon);
void app_config_save_zoom(int idx);
// Demande d'ouverture du portail au prochain démarrage (appui long sur l'encodeur)
void app_config_set_force_portal(bool on);
// Lit puis efface la demande ci-dessus
bool app_config_take_force_portal(void);

// ---------------------------------------------------------------------------------------------
// État partagé
// ---------------------------------------------------------------------------------------------
typedef enum {
    APP_PHASE_BOOT = 0,
    APP_PHASE_CONNECTING,    // connexion au Wi-Fi enregistré
    APP_PHASE_PORTAL,        // point d'accès RadarAvion_Wifi + page de configuration
    APP_PHASE_LOCATING,      // géolocalisation par l'adresse IP publique
    APP_PHASE_RADAR,         // radar en fonctionnement
} app_phase_t;

typedef struct {
    app_phase_t phase;
    char ssid[33];           // réseau utilisé (ou à configurer)
    char info[112];          // message d'état (raison du portail, erreur...)
    char city[48];           // ville déduite de l'IP (ou "Position manuelle")
    bool has_pos;
    double lat;
    double lon;
    bool wifi_ok;
    char ip[16];
    int portal_clients;      // appareils connectés au point d'accès
    bool has_saved_ssid;     // un réseau est déjà enregistré (portail : appui = réessayer)
} app_status_t;

void app_status_get(app_status_t *out);
void app_status_set_phase(app_phase_t phase, const char *info);
void app_status_set_ssid(const char *ssid, bool saved);
void app_status_set_pos(double lat, double lon, const char *city);
void app_status_set_wifi(bool ok, const char *ip);
void app_status_set_portal_clients(int n);

// Démarre la tâche principale (Wi-Fi -> géolocalisation -> radar). NVS doit être initialisé.
void app_start(void);
// Redémarre la carte en mode configuration Wi-Fi
void app_request_portal(void);
