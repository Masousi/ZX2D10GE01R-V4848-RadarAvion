// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — récupération du trafic aérien via les API ADS-B communautaires
// (opendata.adsb.fi, secours api.adsb.lol : même format "readsb"), sans récepteur radio.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ADSB_MAX_AC   200     // avions conservés par requête (les plus proches)
#define ADSB_TRAIL    8       // positions précédentes mémorisées (traînée)

typedef struct {
    char hex[8];              // adresse OACI 24 bits (identifiant unique)
    char flight[10];          // indicatif (ex. AFR7419), vide si inconnu
    char type[6];             // type d'appareil (ex. A320)
    char reg[12];             // immatriculation (ex. F-GKXP)
    float lat, lon;           // dernière position reçue
    float pos_age_s;          // âge de cette position au moment de la requête
    int32_t alt_ft;           // altitude barométrique (pieds)
    bool has_alt;
    bool on_ground;
    float gs_kt;              // vitesse sol (nœuds)
    float track_deg;          // route vraie (0 = nord)
    bool has_track;
    int16_t vrate_fpm;        // vitesse verticale (pieds/min)
    uint8_t trail_n;
    float trail_lat[ADSB_TRAIL];   // [0] = plus ancienne
    float trail_lon[ADSB_TRAIL];
} adsb_ac_t;

typedef struct {
    uint32_t seq;             // incrémenté à chaque nouvelle donnée
    bool ok;                  // dernière requête réussie
    int64_t fetch_us;         // esp_timer_get_time() de la réponse
    char source[16];
    int count;
    adsb_ac_t ac[ADSB_MAX_AC];
} adsb_snapshot_t;

// Lance la tâche de récupération autour de (lat, lon)
void adsb_start(double lat, double lon);
// Rayon d'interrogation (km) ; un rayon plus grand que l'actuel déclenche une requête immédiate
void adsb_set_radius_km(float km);
// Copie la dernière donnée dans dst si elle est plus récente que *seq (met *seq à jour)
bool adsb_copy_if_new(adsb_snapshot_t *dst, uint32_t *seq);
