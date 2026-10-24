// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — accès réseau : Wi-Fi (station + portail de configuration),
// requêtes HTTP(S) et géolocalisation par l'adresse IP publique.
#pragma once

#include <stdbool.h>
#include <stddef.h>

// --- Wi-Fi (wifi_portal.c) ---
void wifi_init_common(void);
// Se connecte au réseau ; true si une adresse IP est obtenue avant timeout_ms.
// Ensuite, la reconnexion est automatique en cas de coupure.
bool wifi_sta_connect(const char *ssid, const char *pass, int timeout_ms);
bool wifi_is_connected(void);
void wifi_get_ip(char *buf, size_t len);
// Lance le point d'accès RadarAvion_Wifi + serveur web + DNS captif. Ne revient jamais
// (redémarrage après enregistrement, ou après 5 min sans visiteur si un réseau est déjà enregistré).
void wifi_portal_run(const char *reason, bool has_saved_ssid);

// --- HTTP (net_http.c) ---
// GET url (http ou https). *out = corps de la réponse terminé par 0 (en PSRAM, à libérer avec free()).
// Retourne le code HTTP, ou -1 en cas d'erreur réseau.
int net_http_get(const char *url, char **out, size_t *out_len, int timeout_ms);
// Position approximative d'après l'IP publique (ip-api.com, puis ipwho.is en secours)
bool geoloc_by_ip(double *lat, double *lon, char *city, size_t city_len);
