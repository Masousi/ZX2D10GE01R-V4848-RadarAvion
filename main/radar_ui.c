// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — interface LVGL de l'écran rond 480x480.
// - Radar style sonar dessiné à la main (événement DRAW_MAIN) : fond dégradé, cercles de distance,
//   graduations, points cardinaux, balayage rotatif avec traînée, avions (triangle orienté selon la route,
//   couleur selon l'altitude, effet "phosphore" ravivé au passage du balayage, traînée, indicatif + altitude).
// - Positions extrapolées entre deux requêtes (vitesse sol + route) : les avions se déplacent en continu.
// - Encodeur : rotation = zoom (rayon 2 à 160 km), appui court = avion suivant (fiche détaillée),
//   appui long 3 s = portail de configuration Wi-Fi.
// - Écrans de connexion / localisation et écran du portail (QR code pour rejoindre RadarAvion_Wifi).
// Toutes les fonctions de ce fichier s'exécutent dans la tâche LVGL.
// ⚫ MODIFIÉ (24/10/2026) : performances : le fond (disques, cercles, graduations, points cardinaux,
// distances) était redessiné à chaque image (masques circulaires sur 400x400 px, ~60 ms -> 8 img/s,
// balayage saccadé). Il est maintenant dessiné une seule fois dans un canvas (PSRAM), redessiné
// seulement au changement de zoom ; la traînée du balayage est faite de triangles (moins coûteux
// que des secteurs d'arc). Ajout de l'écran "Code Wi-Fi" (nom du réseau, mot de passe, QR code).
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "board.h"
#include "mt8901.h"
#include "adsb.h"
#include "radar_ui.h"

#define TAG "UI"

LV_FONT_DECLARE(font_fr_20);
LV_FONT_DECLARE(font_fr_28);

// --- géométrie ---
#define CX              240
#define CY              240
#define R_RADAR         200      // rayon du disque radar (px) ; marge extérieure pour les points cardinaux
#define R_CARDINAL      221

// --- animation ---
#define TICK_MS         33       // ~30 images/s visées
#define SWEEP_PERIOD_S  4.0f     // un tour de balayage
#define TRAIL_DEG       70.0f    // longueur de la traînée du balayage
// #define TRAIL_STEPS     14
// ⚫ MODIFIÉ (24/10/2026) : 14 secteurs d'arc -> 10 triangles (7°)
#define TRAIL_STEPS     10

// --- entrées ---
#define ENC_CW_ZOOMS_IN 1        // 1 : rotation horaire = zoom avant ; mettre 0 pour inverser
#define ENC_LOCKOUT_MS  130      // un cran = un niveau de zoom (ignore les impulsions suivantes du même cran)
#define BTN_SHORT_MS    600
#define BTN_HOLD_MS     3000     // appui long -> configuration Wi-Fi
// ⚫ AJOUTÉ (24/10/2026) : relâché entre BTN_SHORT_MS et BTN_HOLD_MS -> écran "Code Wi-Fi"
#define WCODE_TIMEOUT_MS 60000   // écran "Code Wi-Fi" refermé après 1 min
#define CARD_TIMEOUT_MS 20000    // fiche avion masquée après 20 s sans appui
#define MAX_LABELS      25       // au-delà, seuls les avions sélectionnés gardent leur étiquette

// --- couleurs ---
#define C_DISC0   0x010d07
#define C_DISC1   0x021a0e
#define C_DISC2   0x032412
#define C_RING    0x1f8f55
#define C_LABEL   0x52c98a
#define C_SWEEP   0x36ff9a
#define C_NORTH   0xff6a3d
#define C_TEXT    0xd8ffe8
#define C_DIM     0x7fbf9c
#define C_WARN    0xffa040

#define DEG2RAD   ((float)M_PI / 180.0f)
#define RAD2DEG   (180.0f / (float)M_PI)

static const float ZOOM_KM[] = { 2, 5, 10, 20, 40, 80, 160 };
#define ZOOM_N        (int)(sizeof(ZOOM_KM) / sizeof(ZOOM_KM[0]))
#define ZOOM_DEFAULT  4

typedef struct {
    int16_t x, y;
    float dist_km;
    float brg_deg;     // relèvement depuis le centre (0 = nord)
    uint16_t idx;      // index dans s_snap->ac
} vis_t;

// --- objets LVGL ---
static lv_obj_t *s_radar;
// ⚫ AJOUTÉ (24/10/2026) : fond pré-dessiné (canvas 480x480 RGB565 en PSRAM), parent de s_radar
static lv_obj_t *s_bg;
static int s_bg_zoom = -1;          // niveau de zoom des distances dessinées dans le fond
// ⚫ AJOUTÉ (24/10/2026) : écran "Code Wi-Fi"
static lv_obj_t *s_wcode, *s_wcode_qr, *s_wcode_ssid, *s_wcode_pass;
static int64_t s_wcode_until_ms;
static bool wcode_visible(void);   // défini plus bas (section "Code Wi-Fi")
static lv_obj_t *s_zoom_pill, *s_zoom_lbl;
static lv_obj_t *s_count_lbl, *s_city_lbl;
static lv_obj_t *s_card, *s_card_title, *s_card_type, *s_card_body;
static lv_obj_t *s_msg, *s_msg_lbl;
static lv_obj_t *s_hold_arc, *s_hold_lbl;
static lv_obj_t *s_portal, *s_portal_info, *s_portal_steps, *s_portal_hint;

// --- état ---
static app_status_t s_st;
static app_phase_t s_shown_phase = (app_phase_t)-1;
static adsb_snapshot_t *s_snap;
static uint32_t s_seq;
static vis_t s_vis[ADSB_MAX_AC];
static int s_vis_n;
static bool s_has_pos;
static float s_lat0, s_lon0, s_coslat0;
static int s_zoom_idx = ZOOM_DEFAULT;
static float s_range_km = 40.0f;    // rayon affiché (animé vers ZOOM_KM[s_zoom_idx])
static float s_sweep_deg;
static int64_t s_last_us;
static int64_t s_zoom_save_at_ms, s_zoom_flash_until_ms;
static int16_t s_enc_last;
static int s_enc_acc;
static int64_t s_enc_step_ms;
static bool s_btn_down, s_btn_fired;
static int64_t s_btn_t0_ms, s_portal_req_at_ms;
static char s_sel_hex[8];
static int64_t s_sel_until_ms;
static int64_t s_next_status_ms, s_next_text_ms;
// mesures de performance (journal série toutes les 10 s)
static int64_t s_perf_t0, s_perf_draw_us;
static int s_perf_frames;

static inline int64_t now_ms(void) {
    return esp_timer_get_time() / 1000;
}

static float wrap360(float a) {
    a = fmodf(a, 360.0f);
    return a < 0 ? a + 360.0f : a;
}

// Couleur d'un avion selon son altitude
static uint32_t alt_rgb(const adsb_ac_t *a) {
    if (a->on_ground) return 0x9a9a9a;
    if (!a->has_alt)  return 0xd0d0d0;
    if (a->alt_ft < 3000)  return 0xffa040;   // orange : basse altitude (approche, décollage)
    if (a->alt_ft < 10000) return 0xffe14a;   // jaune
    if (a->alt_ft < 20000) return 0x7dff6a;   // vert
    if (a->alt_ft < 30000) return 0x4ae0ff;   // cyan
    return 0xc39bff;                          // violet : croisière haute
}

// "12 345" (espace des milliers)
static void fmt_thousands(char *b, size_t n, int v) {
    if (v >= 1000) {
        snprintf(b, n, "%d %03d", v / 1000, v % 1000);
    } else {
        snprintf(b, n, "%d", v);
    }
}

static void fmt_alt(char *b, size_t n, const adsb_ac_t *a) {
    if (a->on_ground) {
        snprintf(b, n, "SOL");
    } else if (!a->has_alt) {
        snprintf(b, n, "---");
    } else if (a->alt_ft >= 10000) {
        snprintf(b, n, "FL%03d", (int)(a->alt_ft / 100));        // niveau de vol (centaines de pieds)
    } else {
        char t[12];
        fmt_thousands(t, sizeof(t), (int)(lroundf(a->alt_ft / 100.0f) * 100));
        snprintf(b, n, "%s ft", t);
    }
}

// "2,5" / "10" (virgule française)
static void fmt_km(char *b, size_t n, float km) {
    if (km < 10.0f && fabsf(km - roundf(km)) > 0.05f) {
        snprintf(b, n, "%.1f", km);
        char *p = strchr(b, '.');
        if (p) *p = ',';
    } else {
        snprintf(b, n, "%d", (int)lroundf(km));
    }
}

static const char *ac_name(const adsb_ac_t *a) {
    return a->flight[0] ? a->flight : a->reg[0] ? a->reg : a->hex;
}

static void set_text(lv_obj_t *lbl, const char *txt) {
    if (strcmp(lv_label_get_text(lbl), txt) != 0) {
        lv_label_set_text(lbl, txt);
    }
}

// ---------------------------------------------------------------------------------------------
// Dessin du radar
// ---------------------------------------------------------------------------------------------
static void draw_disc(lv_draw_ctx_t *ctx, int x, int y, int r, uint32_t rgb, lv_opa_t opa) {
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.radius = LV_RADIUS_CIRCLE;
    d.bg_color = lv_color_hex(rgb);
    d.bg_opa = opa;
    lv_area_t a = { x - r, y - r, x + r, y + r };
    lv_draw_rect(ctx, &d, &a);
}

// Arc (angles "relèvement" : 0 = nord, sens horaire) ; width = r -> secteur plein
static void draw_arc(lv_draw_ctx_t *ctx, int r, int width, float from_brg, float to_brg, uint32_t rgb, lv_opa_t opa) {
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.color = lv_color_hex(rgb);
    d.width = width;
    d.opa = opa;
    lv_point_t c = { CX, CY };
    // LVGL : 0° = 3 h, sens horaire
    uint16_t a0 = (uint16_t)wrap360(from_brg - 90.0f);
    uint16_t a1 = (uint16_t)wrap360(to_brg - 90.0f);
    if (to_brg - from_brg >= 360.0f) {
        a0 = 0;
        a1 = 360;
    }
    lv_draw_arc(ctx, &d, &c, r, a0, a1);
}

static void draw_line(lv_draw_ctx_t *ctx, int x0, int y0, int x1, int y1, int w, uint32_t rgb, lv_opa_t opa) {
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = lv_color_hex(rgb);
    d.width = w;
    d.opa = opa;
    lv_point_t p0 = { x0, y0 }, p1 = { x1, y1 };
    lv_draw_line(ctx, &d, &p0, &p1);
}

static void draw_text(lv_draw_ctx_t *ctx, int x, int y, int w, int h, const char *txt, const lv_font_t *font,
                      uint32_t rgb, lv_opa_t opa, lv_text_align_t align) {
    lv_draw_label_dsc_t d;
    lv_draw_label_dsc_init(&d);
    d.font = font;
    d.color = lv_color_hex(rgb);
    d.opa = opa;
    d.align = align;
    lv_area_t a = { x, y, x + w - 1, y + h - 1 };
    lv_draw_label(ctx, &d, &a, txt, NULL);
}

// Triangle orienté selon la route (pointe = avant de l'avion)
static void draw_plane(lv_draw_ctx_t *ctx, int x, int y, float hdg_deg, float size, uint32_t rgb, lv_opa_t opa) {
    static const float P[3][2] = { { 0, -11 }, { -7, 8 }, { 7, 8 } };
    float h = hdg_deg * DEG2RAD, c = cosf(h), s = sinf(h);
    lv_point_t p[3];
    for (int k = 0; k < 3; k++) {
        float px = P[k][0] * size, py = P[k][1] * size;
        p[k].x = x + (lv_coord_t)lroundf(px * c - py * s);
        p[k].y = y + (lv_coord_t)lroundf(px * s + py * c);
    }
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(rgb);
    d.bg_opa = opa;
    lv_draw_polygon(ctx, &d, p, 3);
}

static bool project(float lat, float lon, int *x, int *y) {
    float dx = (lon - s_lon0) * 111.32f * s_coslat0;
    float dy = (lat - s_lat0) * 110.574f;
    float scale = R_RADAR / s_range_km;
    float px = dx * scale, py = dy * scale;
    if (px * px + py * py > (float)R_RADAR * R_RADAR) {
        return false;
    }
    *x = CX + (int)lroundf(px);
    *y = CY - (int)lroundf(py);
    return true;
}

// static void draw_background(lv_draw_ctx_t *ctx) {
//     // dégradé radial approché par 3 disques
//     draw_disc(ctx, CX, CY, R_RADAR, C_DISC0, LV_OPA_COVER);
//     draw_disc(ctx, CX, CY, R_RADAR * 7 / 10, C_DISC1, LV_OPA_COVER);
//     draw_disc(ctx, CX, CY, R_RADAR * 4 / 10, C_DISC2, LV_OPA_COVER);
// }
// ⚫ RETIRÉ (24/10/2026) : draw_background() et draw_grid() (cercles de distance, croix, graduations tous
// les 10° (plus longues tous les 30°), points cardinaux dans la marge extérieure, distances des cercles
// le long du relèvement 30°) dessinaient tout à chaque image -> render_background() : même rendu,
// dessiné une seule fois dans le canvas s_bg.

static void bg_disc(int r, uint32_t rgb) {
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.radius = LV_RADIUS_CIRCLE;
    d.bg_color = lv_color_hex(rgb);
    d.bg_opa = LV_OPA_COVER;
    lv_canvas_draw_rect(s_bg, CX - r, CY - r, 2 * r + 1, 2 * r + 1, &d);
}

static void bg_line(int x0, int y0, int x1, int y1, int w, lv_opa_t opa) {
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = lv_color_hex(C_RING);
    d.width = w;
    d.opa = opa;
    lv_point_t p[2] = { { x0, y0 }, { x1, y1 } };
    lv_canvas_draw_line(s_bg, p, 2, &d);
}

static void bg_text(int x, int y, int w, const char *txt, const lv_font_t *font, uint32_t rgb, lv_text_align_t align) {
    lv_draw_label_dsc_t d;
    lv_draw_label_dsc_init(&d);
    d.font = font;
    d.color = lv_color_hex(rgb);
    d.align = align;
    lv_canvas_draw_text(s_bg, x, y, w, &d, txt);
}

// Fond fixe du radar, redessiné uniquement quand le zoom change (distances des cercles)
static void render_background(void) {
    int64_t t0 = esp_timer_get_time();
    lv_canvas_fill_bg(s_bg, lv_color_black(), LV_OPA_COVER);
    // dégradé radial approché par 3 disques
    bg_disc(R_RADAR, C_DISC0);
    bg_disc(R_RADAR * 7 / 10, C_DISC1);
    bg_disc(R_RADAR * 4 / 10, C_DISC2);
    // cercles de distance
    for (int k = 1; k <= 4; k++) {
        lv_draw_arc_dsc_t d;
        lv_draw_arc_dsc_init(&d);
        d.color = lv_color_hex(C_RING);
        d.width = k == 4 ? 2 : 1;
        d.opa = k == 4 ? 220 : 110;
        lv_canvas_draw_arc(s_bg, CX, CY, R_RADAR * k / 4, 0, 360, &d);
    }
    // croix
    bg_line(CX - R_RADAR, CY, CX + R_RADAR, CY, 1, 70);
    bg_line(CX, CY - R_RADAR, CX, CY + R_RADAR, 1, 70);
    // graduations tous les 10° (plus longues tous les 30°)
    for (int deg = 0; deg < 360; deg += 10) {
        float a = deg * DEG2RAD, sn = sinf(a), cs = cosf(a);
        int len = (deg % 30 == 0) ? 12 : 6;
        bg_line(CX + (int)lroundf((R_RADAR - len) * sn), CY - (int)lroundf((R_RADAR - len) * cs),
                CX + (int)lroundf(R_RADAR * sn), CY - (int)lroundf(R_RADAR * cs),
                deg % 30 == 0 ? 2 : 1, 200);
    }
    // points cardinaux dans la marge extérieure
    static const char *CARD[4] = { "N", "E", "S", "O" };
    for (int i = 0; i < 4; i++) {
        float a = i * 90 * DEG2RAD;
        int x = CX + (int)lroundf(R_CARDINAL * sinf(a));
        int y = CY - (int)lroundf(R_CARDINAL * cosf(a));
        bg_text(x - 12, y - 9, 24, CARD[i], &lv_font_montserrat_16, i == 0 ? C_NORTH : C_LABEL, LV_TEXT_ALIGN_CENTER);
    }
    // distances des cercles (niveau de zoom choisi), le long du relèvement 30°
    char t[16];
    float range = ZOOM_KM[s_zoom_idx];
    for (int k = 1; k <= 4; k++) {
        float r = R_RADAR * k / 4.0f;
        fmt_km(t, sizeof(t) - 3, range * k / 4.0f);
        if (k == 4) {
            strcat(t, " km");
        }
        bg_text(CX + (int)lroundf(r * 0.5f) + 3, CY - (int)lroundf(r * 0.866f) + 2, 60, t,
                &lv_font_montserrat_12, C_LABEL, LV_TEXT_ALIGN_LEFT);
    }
    s_bg_zoom = s_zoom_idx;
    ESP_LOGI(TAG, "Fond du radar dessine en %lld ms", (esp_timer_get_time() - t0) / 1000);
}

static void draw_sweep(lv_draw_ctx_t *ctx) {
    float step = TRAIL_DEG / TRAIL_STEPS;
    // ⚫ MODIFIÉ (24/10/2026) : secteurs d'arc (draw_arc de largeur = rayon : masques circulaires + angulaires
    // sur tout le secteur) remplacés par des triangles centre -> bord (corde à 7° : < 0,4 px du cercle)
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(C_SWEEP);
    for (int i = 0; i < TRAIL_STEPS; i++) {
        float k = 1.0f - (float)i / TRAIL_STEPS;
        lv_opa_t opa = (lv_opa_t)(95.0f * k * k);
        if (opa < 4) {
            break;
        }
        // draw_arc(ctx, R_RADAR, R_RADAR, s_sweep_deg - (i + 1) * step, s_sweep_deg - i * step, C_SWEEP, opa);
        float a0 = (s_sweep_deg - (i + 1) * step) * DEG2RAD, a1 = (s_sweep_deg - i * step) * DEG2RAD;
        lv_point_t p[3] = {
            { CX, CY },
            { CX + (lv_coord_t)lroundf(R_RADAR * sinf(a0)), CY - (lv_coord_t)lroundf(R_RADAR * cosf(a0)) },
            { CX + (lv_coord_t)lroundf(R_RADAR * sinf(a1)), CY - (lv_coord_t)lroundf(R_RADAR * cosf(a1)) },
        };
        d.bg_opa = opa;
        lv_draw_polygon(ctx, &d, p, 3);
    }
    float a = s_sweep_deg * DEG2RAD;
    draw_line(ctx, CX, CY, CX + (int)lroundf(R_RADAR * sinf(a)), CY - (int)lroundf(R_RADAR * cosf(a)), 2, C_SWEEP, 230);
}

static void draw_aircraft(lv_draw_ctx_t *ctx) {
    if (!s_snap) {
        return;
    }
    bool labels = s_vis_n <= MAX_LABELS;
    for (int i = 0; i < s_vis_n; i++) {
        const vis_t *v = &s_vis[i];
        const adsb_ac_t *a = &s_snap->ac[v->idx];
        bool sel = s_sel_hex[0] && strcmp(a->hex, s_sel_hex) == 0;
        uint32_t rgb = alt_rgb(a);
        // "phosphore" : vif juste après le passage du balayage, puis s'estompe pendant le tour
        float since = wrap360(s_sweep_deg - v->brg_deg);
        lv_opa_t opa = (lv_opa_t)(255.0f - since / 360.0f * 140.0f);
        if (sel) {
            opa = LV_OPA_COVER;
        }

        // traînée (positions des requêtes précédentes)
        for (int t = 0; t < a->trail_n; t++) {
            int tx, ty;
            if (project(a->trail_lat[t], a->trail_lon[t], &tx, &ty)) {
                draw_disc(ctx, tx, ty, 2, rgb, (lv_opa_t)((30 + 110 * (t + 1) / a->trail_n) * opa / 255));
            }
        }
        // halo au passage du balayage
        if (since < 30.0f) {
            draw_disc(ctx, v->x, v->y, 14, rgb, (lv_opa_t)(80.0f * (1.0f - since / 30.0f)));
        }
        if (a->on_ground) {
            draw_disc(ctx, v->x, v->y, 4, rgb, opa);
        } else if (a->has_track) {
            draw_plane(ctx, v->x, v->y, a->track_deg, sel ? 1.25f : 1.0f, rgb, opa);
        } else {
            draw_disc(ctx, v->x, v->y, 5, rgb, opa);
        }
        if (sel) {
            draw_arc(ctx, 19, 2, 0, 360, 0xffffff, 230);
        }
        if (labels || sel) {
            char txt[32], alt[16];
            fmt_alt(alt, sizeof(alt), a);
            snprintf(txt, sizeof(txt), "%s\n%s", ac_name(a), alt);
            bool left = v->x > CX + R_RADAR / 2;   // près du bord droit : étiquette à gauche
            draw_text(ctx, left ? v->x - 92 : v->x + 12, v->y - 16, 80, 32, txt, &lv_font_montserrat_12,
                      sel ? 0xffffff : C_TEXT, opa, left ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT);
        }
    }
}

static void draw_center(lv_draw_ctx_t *ctx) {
    float ph = (float)(now_ms() % 2000) / 2000.0f;
    draw_arc(ctx, 5 + (int)(ph * 22), 2, 0, 360, C_SWEEP, (lv_opa_t)(170 * (1.0f - ph)));
    draw_disc(ctx, CX, CY, 4, C_SWEEP, LV_OPA_COVER);
}

static void radar_draw_cb(lv_event_t *e) {
    int64_t t0 = esp_timer_get_time();
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
    // draw_background(ctx);
    // ⚫ MODIFIÉ (24/10/2026) : fond et grille pré-dessinés dans s_bg (parent de s_radar)
    draw_sweep(ctx);
    // draw_grid(ctx);
    if (s_st.phase == APP_PHASE_RADAR) {
        draw_aircraft(ctx);
    }
    draw_center(ctx);
    s_perf_draw_us += esp_timer_get_time() - t0;
    s_perf_frames++;
}

// ---------------------------------------------------------------------------------------------
// Données : avions visibles
// ---------------------------------------------------------------------------------------------
static void compute_visible(void) {
    s_vis_n = 0;
    if (!s_has_pos || !s_snap) {
        return;
    }
    int64_t now = esp_timer_get_time();
    float since = (now - s_snap->fetch_us) / 1e6f;
    float scale = R_RADAR / s_range_km;
    for (int i = 0; i < s_snap->count; i++) {
        const adsb_ac_t *a = &s_snap->ac[i];
        float lat = a->lat, lon = a->lon;
        // estime la position actuelle (avion en vol) : vitesse sol x temps écoulé, selon la route
        if (a->has_track && !a->on_ground && a->gs_kt > 30.0f) {
            float age = fminf(a->pos_age_s + since, 60.0f);
            float d_km = a->gs_kt * 0.000514444f * age;   // nœuds -> km/s
            float tr = a->track_deg * DEG2RAD;
            lat += d_km * cosf(tr) / 110.574f;
            lon += d_km * sinf(tr) / (111.32f * s_coslat0);
        }
        float dx = (lon - s_lon0) * 111.32f * s_coslat0;
        float dy = (lat - s_lat0) * 110.574f;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > s_range_km) {
            continue;
        }
        vis_t *v = &s_vis[s_vis_n++];
        v->x = CX + (int16_t)lroundf(dx * scale);
        v->y = CY - (int16_t)lroundf(dy * scale);
        v->dist_km = dist;
        v->brg_deg = wrap360(atan2f(dx, dy) * RAD2DEG);
        v->idx = i;
    }
}

static const vis_t *find_selected(void) {
    if (!s_sel_hex[0]) {
        return NULL;
    }
    for (int i = 0; i < s_vis_n; i++) {
        if (strcmp(s_snap->ac[s_vis[i].idx].hex, s_sel_hex) == 0) {
            return &s_vis[i];
        }
    }
    return NULL;
}

// Appui court : avion suivant, du plus proche au plus lointain, puis plus de sélection
static void select_next(void) {
    if (s_vis_n == 0) {
        s_sel_hex[0] = 0;
        return;
    }
    static uint16_t order[ADSB_MAX_AC];
    for (int i = 0; i < s_vis_n; i++) {
        order[i] = i;
    }
    for (int i = 1; i < s_vis_n; i++) {   // tri par insertion selon la distance
        uint16_t k = order[i];
        int j = i - 1;
        while (j >= 0 && s_vis[order[j]].dist_km > s_vis[k].dist_km) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = k;
    }
    int cur = -1;
    for (int i = 0; i < s_vis_n && s_sel_hex[0]; i++) {
        if (strcmp(s_snap->ac[s_vis[order[i]].idx].hex, s_sel_hex) == 0) {
            cur = i;
            break;
        }
    }
    if (cur == s_vis_n - 1) {
        s_sel_hex[0] = 0;
    } else {
        strlcpy(s_sel_hex, s_snap->ac[s_vis[order[cur + 1]].idx].hex, sizeof(s_sel_hex));
        s_sel_until_ms = now_ms() + CARD_TIMEOUT_MS;
    }
}

// ---------------------------------------------------------------------------------------------
// Textes (fiche avion, compteur, zoom)
// ---------------------------------------------------------------------------------------------
static void update_zoom_label(void) {
    char t[24], km[12];
    fmt_km(km, sizeof(km), ZOOM_KM[s_zoom_idx]);
    snprintf(t, sizeof(t), "Rayon %s km", km);
    set_text(s_zoom_lbl, t);
}

static void update_texts(void) {
    char t[160];
    const vis_t *sel = find_selected();
    if (s_sel_hex[0] && (!sel || now_ms() > s_sel_until_ms)) {
        s_sel_hex[0] = 0;   // sorti du champ ou délai écoulé
        sel = NULL;
    }
    if (sel) {
        const adsb_ac_t *a = &s_snap->ac[sel->idx];
        char alt[16], dist[12];
        fmt_alt(alt, sizeof(alt), a);
        fmt_km(dist, sizeof(dist), sel->dist_km);
        set_text(s_card_title, ac_name(a));
        set_text(s_card_type, a->type[0] ? a->type : "");
        const char *vert = a->on_ground ? "au sol"
                         : a->vrate_fpm >= 300 ? "en montée"
                         : a->vrate_fpm <= -300 ? "en descente" : "en palier";
        snprintf(t, sizeof(t), "%s · %s\n%d km/h · cap %03d°\nà %s km%s%s", alt, vert,
                 (int)lroundf(a->gs_kt * 1.852f), (int)lroundf(a->track_deg) % 360, dist,
                 a->reg[0] && strcmp(a->reg, ac_name(a)) ? " · " : "",
                 a->reg[0] && strcmp(a->reg, ac_name(a)) ? a->reg : "");
        set_text(s_card_body, t);
        lv_obj_set_style_border_color(s_card, lv_color_hex(alt_rgb(a)), 0);
        lv_obj_clear_flag(s_card, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_count_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_city_lbl, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_card, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_count_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_city_lbl, LV_OBJ_FLAG_HIDDEN);
    }

    uint32_t col = C_TEXT;
    if (!s_st.wifi_ok) {
        snprintf(t, sizeof(t), "Wi-Fi perdu, reconnexion...");
        col = C_WARN;
    } else if (!s_snap || s_snap->seq == 0) {
        snprintf(t, sizeof(t), "Recherche des avions...");
    } else if (!s_snap->ok) {
        snprintf(t, sizeof(t), "Données indisponibles");
        col = C_WARN;
    } else {
        snprintf(t, sizeof(t), s_vis_n > 1 ? "%d avions" : "%d avion", s_vis_n);
    }
    set_text(s_count_lbl, t);
    lv_obj_set_style_text_color(s_count_lbl, lv_color_hex(col), 0);
    set_text(s_city_lbl, s_st.city[0] ? s_st.city : "");

    bool flash = now_ms() < s_zoom_flash_until_ms;
    lv_obj_set_style_border_color(s_zoom_pill, lv_color_hex(flash ? C_SWEEP : C_RING), 0);
    lv_obj_set_style_text_color(s_zoom_lbl, lv_color_hex(flash ? 0xffffff : C_TEXT), 0);
}

// ---------------------------------------------------------------------------------------------
// Phases (connexion, portail, radar)
// ---------------------------------------------------------------------------------------------
static void apply_status(void) {
    app_status_get(&s_st);
    if (s_st.has_pos && !s_has_pos) {
        s_has_pos = true;
        s_lat0 = (float)s_st.lat;
        s_lon0 = (float)s_st.lon;
        s_coslat0 = cosf(s_lat0 * DEG2RAD);
    }
    bool portal = s_st.phase == APP_PHASE_PORTAL;
    bool radar = s_st.phase == APP_PHASE_RADAR;
    if (s_st.phase != s_shown_phase) {
        s_shown_phase = s_st.phase;
        lv_obj_add_flag(s_portal, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_msg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_zoom_pill, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_count_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_city_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_card, LV_OBJ_FLAG_HIDDEN);
        if (portal) {
            // lv_obj_add_flag(s_radar, LV_OBJ_FLAG_HIDDEN);   // le radar n'est plus redessiné (CPU)
            // ⚫ MODIFIÉ (24/10/2026) : fond + radar (s_radar est un enfant de s_bg)
            lv_obj_add_flag(s_bg, LV_OBJ_FLAG_HIDDEN);   // le radar n'est plus redessiné (CPU)
            lv_obj_add_flag(s_wcode, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_portal, LV_OBJ_FLAG_HIDDEN);
        } else {
            // lv_obj_clear_flag(s_radar, LV_OBJ_FLAG_HIDDEN);
            // ⚫ MODIFIÉ (24/10/2026) : fond + radar
            if (!wcode_visible()) {
                lv_obj_clear_flag(s_bg, LV_OBJ_FLAG_HIDDEN);
            }
            if (radar) {
                lv_obj_clear_flag(s_zoom_pill, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(s_count_lbl, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(s_city_lbl, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(s_msg, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    if (portal) {
        char t[160];
        set_text(s_portal_info, s_st.info);
        snprintf(t, sizeof(t), s_st.portal_clients > 0
                 ? "%d appareil connecté\nOuvrez http://" APP_AP_IP
                 : "Wi-Fi : " APP_AP_SSID "\npuis http://" APP_AP_IP,
                 s_st.portal_clients);
        set_text(s_portal_steps, t);
        if (s_st.has_saved_ssid) {
            snprintf(t, sizeof(t), "Appui : réessayer %s", s_st.ssid);
        } else {
            t[0] = 0;
        }
        set_text(s_portal_hint, t);
    } else if (!radar) {
        set_text(s_msg_lbl, s_st.info[0] ? s_st.info : "Démarrage");
    }
}

// ---------------------------------------------------------------------------------------------
// ⚫ AJOUTÉ (24/10/2026) : écran "Code Wi-Fi" (réseau enregistré, mot de passe, QR code pour s'y connecter)
// ---------------------------------------------------------------------------------------------
// Échappement des caractères spéciaux du format "WIFI:" (\ ; , : ")
static void qr_escape(char *dst, size_t cap, const char *src) {
    size_t o = 0;
    for (; *src && o + 2 < cap; src++) {
        if (strchr("\\;,:\"", *src)) {
            dst[o++] = '\\';
        }
        dst[o++] = *src;
    }
    dst[o] = 0;
}

static bool wcode_visible(void) {
    return !lv_obj_has_flag(s_wcode, LV_OBJ_FLAG_HIDDEN);
}

static void wcode_hide(void) {
    lv_obj_add_flag(s_wcode, LV_OBJ_FLAG_HIDDEN);
    if (s_st.phase != APP_PHASE_PORTAL) {
        lv_obj_clear_flag(s_bg, LV_OBJ_FLAG_HIDDEN);
    }
}

static void wcode_show(void) {
    app_config_t cfg;
    app_config_load(&cfg);
    if (!cfg.ssid[0]) {
        return;
    }
    char ssid[80], pass[140], qr[240];
    qr_escape(ssid, sizeof(ssid), cfg.ssid);
    qr_escape(pass, sizeof(pass), cfg.pass);
    if (cfg.pass[0]) {
        snprintf(qr, sizeof(qr), "WIFI:T:WPA;S:%s;P:%s;;", ssid, pass);
    } else {
        snprintf(qr, sizeof(qr), "WIFI:T:nopass;S:%s;;", ssid);
    }
    lv_qrcode_update(s_wcode_qr, qr, strlen(qr));
    set_text(s_wcode_ssid, cfg.ssid);
    set_text(s_wcode_pass, cfg.pass[0] ? cfg.pass : "(réseau ouvert)");
    memset(&cfg, 0, sizeof(cfg));
    lv_obj_add_flag(s_bg, LV_OBJ_FLAG_HIDDEN);   // le radar n'est plus redessiné pendant ce temps
    lv_obj_clear_flag(s_wcode, LV_OBJ_FLAG_HIDDEN);
    s_wcode_until_ms = now_ms() + WCODE_TIMEOUT_MS;
}

// ---------------------------------------------------------------------------------------------
// Entrées : encodeur (zoom) et bouton
// ---------------------------------------------------------------------------------------------
static void zoom_step(int dir) {   // dir < 0 : zoom avant (rayon plus petit)
    int idx = s_zoom_idx + (dir < 0 ? -1 : 1);
    if (idx < 0 || idx >= ZOOM_N) {
        return;
    }
    s_zoom_idx = idx;
    adsb_set_radius_km(ZOOM_KM[idx]);
    update_zoom_label();
    s_zoom_flash_until_ms = now_ms() + 900;
    s_zoom_save_at_ms = now_ms() + 3000;   // enregistré en NVS une fois le réglage stabilisé
}

static void handle_input(void) {
    int64_t now = now_ms();
    bool radar = s_st.phase == APP_PHASE_RADAR;

    int16_t c = mt8901_get_count();
    int16_t d = c - s_enc_last;
    s_enc_last = c;
    if (now - s_enc_step_ms < ENC_LOCKOUT_MS) {
        s_enc_acc = 0;
    } else {
        s_enc_acc += d;
        if (s_enc_acc != 0) {
            // le compteur diminue en sens horaire (cf. ECO_STEP dans screen.c)
            int cw = s_enc_acc < 0 ? 1 : -1;
            s_enc_acc = 0;
            s_enc_step_ms = now;
            if (radar) {
                zoom_step(ENC_CW_ZOOMS_IN ? -cw : cw);
            }
        }
    }

    bool down = gpio_get_level(BTN_PIN) == 0;   // actif bas
    if (down && !s_btn_down) {
        s_btn_down = true;
        s_btn_fired = false;
        s_btn_t0_ms = now;
        // ⚫ AJOUTÉ (24/10/2026) : le texte est remplacé par "Configuration Wi-Fi..." une fois l'appui long atteint
        lv_label_set_text(s_hold_lbl, "Relâchez : code Wi-Fi\nMaintenez : configuration");
    }
    // if (down && !s_btn_fired && s_st.phase != APP_PHASE_PORTAL) {
    // ⚫ MODIFIÉ (24/10/2026) : pas d'appui long pendant l'affichage du code Wi-Fi (un appui le referme)
    if (down && !s_btn_fired && s_st.phase != APP_PHASE_PORTAL && !wcode_visible()) {
        int64_t held = now - s_btn_t0_ms;
        if (held > BTN_SHORT_MS) {
            lv_arc_set_value(s_hold_arc, (int)((held - BTN_SHORT_MS) * 100 / (BTN_HOLD_MS - BTN_SHORT_MS)));
            lv_obj_clear_flag(s_hold_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(s_hold_lbl, LV_OBJ_FLAG_HIDDEN);
        }
        if (held >= BTN_HOLD_MS) {
            s_btn_fired = true;
            lv_label_set_text(s_hold_lbl, "Configuration Wi-Fi...");
            s_portal_req_at_ms = now + 400;
        }
    }
    if (!down && s_btn_down) {
        s_btn_down = false;
        if (!s_btn_fired) {
            lv_obj_add_flag(s_hold_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(s_hold_lbl, LV_OBJ_FLAG_HIDDEN);
            if (wcode_visible()) {
                // ⚫ AJOUTÉ (24/10/2026) : un appui referme l'écran "Code Wi-Fi"
                wcode_hide();
            } else if (now - s_btn_t0_ms >= BTN_SHORT_MS) {
                // ⚫ AJOUTÉ (24/10/2026) : appui moyen (0,6 à 3 s) -> code Wi-Fi
                if (s_st.phase != APP_PHASE_PORTAL) {
                    wcode_show();
                }
            } else if (now - s_btn_t0_ms < BTN_SHORT_MS) {
                if (radar) {
                    select_next();
                    update_texts();
                } else if (s_st.phase == APP_PHASE_PORTAL && s_st.has_saved_ssid) {
                    esp_restart();   // nouvel essai du réseau enregistré
                }
            }
        }
    }
    if (s_portal_req_at_ms && now >= s_portal_req_at_ms) {
        app_request_portal();
    }
}

// ---------------------------------------------------------------------------------------------
// Boucle d'animation
// ---------------------------------------------------------------------------------------------
static void ui_tick(lv_timer_t *t) {
    int64_t now_us = esp_timer_get_time();
    float dt = s_last_us ? (now_us - s_last_us) / 1e6f : 0.0f;
    s_last_us = now_us;
    int64_t now = now_us / 1000;

    if (now >= s_next_status_ms) {
        s_next_status_ms = now + 250;
        apply_status();
    }
    handle_input();

    // rayon affiché : glisse vers le niveau choisi (interpolation logarithmique)
    float target = ZOOM_KM[s_zoom_idx];
    float lr = logf(s_range_km), lt = logf(target);
    s_range_km = fabsf(lt - lr) < 0.002f ? target : expf(lr + (lt - lr) * fminf(1.0f, dt * 9.0f));

    s_sweep_deg = wrap360(s_sweep_deg + dt * 360.0f / SWEEP_PERIOD_S);

    // ⚫ AJOUTÉ (24/10/2026) : fond redessiné seulement si le zoom a changé
    if (s_bg_zoom != s_zoom_idx) {
        render_background();
    }
    if (wcode_visible() && now >= s_wcode_until_ms) {
        wcode_hide();
    }

    if (s_st.phase == APP_PHASE_RADAR) {
        adsb_copy_if_new(s_snap, &s_seq);
        compute_visible();
        if (now >= s_next_text_ms) {
            s_next_text_ms = now + 400;
            update_texts();
        }
        if (s_zoom_save_at_ms && now >= s_zoom_save_at_ms) {
            s_zoom_save_at_ms = 0;
            app_config_save_zoom(s_zoom_idx);
        }
    }

    const vis_t *sel = (s_st.phase == APP_PHASE_RADAR) ? find_selected() : NULL;
    radar_led_set(s_st.phase, s_sweep_deg, sel ? sel->brg_deg : -1.0f,
                  sel ? alt_rgb(&s_snap->ac[sel->idx]) : 0);

    // if (!lv_obj_has_flag(s_radar, LV_OBJ_FLAG_HIDDEN)) {
    // ⚫ MODIFIÉ (24/10/2026) : c'est le fond s_bg (parent de s_radar) qui est masqué en mode portail / code Wi-Fi
    if (!lv_obj_has_flag(s_bg, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_invalidate(s_radar);
    }

    if (now_us - s_perf_t0 > 10000000) {
        if (s_perf_frames) {
            ESP_LOGI(TAG, "%.1f img/s, dessin %lld ms, %d avions affiches, PSRAM libre %u Ko, interne %u Ko",
                     s_perf_frames * 1e6f / (now_us - s_perf_t0), s_perf_draw_us / s_perf_frames / 1000, s_vis_n,
                     (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                     (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
        }
        s_perf_t0 = now_us;
        s_perf_draw_us = 0;
        s_perf_frames = 0;
    }
}

// ---------------------------------------------------------------------------------------------
// Construction de l'interface
// ---------------------------------------------------------------------------------------------
static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, uint32_t rgb, const char *txt) {
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(rgb), 0);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(l, txt);
    return l;
}

static lv_obj_t *make_panel(lv_obj_t *parent, int w, int h, lv_opa_t opa) {
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_remove_style_all(p);
    lv_obj_set_size(p, w, h);
    lv_obj_set_style_bg_color(p, lv_color_hex(0x041a10), 0);
    lv_obj_set_style_bg_opa(p, opa, 0);
    lv_obj_set_style_border_color(p, lv_color_hex(C_RING), 0);
    lv_obj_set_style_border_width(p, 1, 0);
    lv_obj_set_style_radius(p, 18, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return p;
}

static void build_portal(lv_obj_t *scr) {
    s_portal = lv_obj_create(scr);
    lv_obj_remove_style_all(s_portal);
    lv_obj_set_size(s_portal, 480, 480);
    lv_obj_set_style_bg_color(s_portal, lv_color_hex(0x020c07), 0);
    lv_obj_set_style_bg_opa(s_portal, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_portal, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *title = make_label(s_portal, &font_fr_28, 0x5dffa8, "Configuration Wi-Fi");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 48);
    s_portal_info = make_label(s_portal, &font_fr_20, C_WARN, "");
    lv_obj_set_width(s_portal_info, 330);
    lv_label_set_long_mode(s_portal_info, LV_LABEL_LONG_DOT);
    lv_obj_align(s_portal_info, LV_ALIGN_TOP_MID, 0, 88);

    // QR code "rejoindre le Wi-Fi" (appareil photo du téléphone), sur fond blanc (marge de lecture)
    lv_obj_t *box = lv_obj_create(s_portal);
    lv_obj_remove_style_all(box);
    lv_obj_set_size(box, 178, 178);
    lv_obj_set_style_bg_color(box, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(box, 12, 0);
    lv_obj_align(box, LV_ALIGN_CENTER, 0, -14);
    lv_obj_t *qr = lv_qrcode_create(box, 156, lv_color_black(), lv_color_white());
    const char *wifi_qr = "WIFI:T:nopass;S:" APP_AP_SSID ";;";
    lv_qrcode_update(qr, wifi_qr, strlen(wifi_qr));
    lv_obj_center(qr);

    s_portal_steps = make_label(s_portal, &font_fr_20, C_TEXT, "");
    lv_obj_align(s_portal_steps, LV_ALIGN_CENTER, 0, 108);
    s_portal_hint = make_label(s_portal, &font_fr_20, C_DIM, "");
    lv_obj_set_width(s_portal_hint, 300);
    lv_label_set_long_mode(s_portal_hint, LV_LABEL_LONG_DOT);
    lv_obj_align(s_portal_hint, LV_ALIGN_BOTTOM_MID, 0, -44);
}

void radar_ui_init(void) {
    s_snap = heap_caps_calloc(1, sizeof(adsb_snapshot_t), MALLOC_CAP_SPIRAM);
    assert(s_snap);

    app_config_t cfg;
    app_config_load(&cfg);
    if (cfg.zoom_idx >= 0 && cfg.zoom_idx < ZOOM_N) {
        s_zoom_idx = cfg.zoom_idx;
    }
    s_range_km = ZOOM_KM[s_zoom_idx];
    adsb_set_radius_km(s_range_km);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ⚫ AJOUTÉ (24/10/2026) : fond pré-dessiné (canvas RGB565 480x480 en PSRAM)
    void *bg_buf = heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(480, 480), MALLOC_CAP_SPIRAM);
    assert(bg_buf);
    s_bg = lv_canvas_create(scr);
    lv_canvas_set_buffer(s_bg, bg_buf, 480, 480, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_pos(s_bg, 0, 0);
    lv_obj_clear_flag(s_bg, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    render_background();

    // radar dessiné à la main
    // s_radar = lv_obj_create(scr);
    // ⚫ MODIFIÉ (24/10/2026) : enfant du fond (masqués ensemble)
    s_radar = lv_obj_create(s_bg);
    lv_obj_remove_style_all(s_radar);
    lv_obj_set_size(s_radar, 480, 480);
    lv_obj_clear_flag(s_radar, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_radar, radar_draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    // rayon (haut)
    s_zoom_pill = make_panel(scr, 170, 34, 200);
    lv_obj_set_style_radius(s_zoom_pill, LV_RADIUS_CIRCLE, 0);
    lv_obj_align(s_zoom_pill, LV_ALIGN_TOP_MID, 0, 58);
    s_zoom_lbl = make_label(s_zoom_pill, &font_fr_20, C_TEXT, "");
    lv_obj_center(s_zoom_lbl);
    update_zoom_label();

    // compteur + ville (bas)
    s_count_lbl = make_label(scr, &font_fr_20, C_TEXT, "");
    lv_obj_align(s_count_lbl, LV_ALIGN_BOTTOM_MID, 0, -86);
    s_city_lbl = make_label(scr, &font_fr_20, C_DIM, "");
    lv_obj_set_width(s_city_lbl, 220);
    lv_label_set_long_mode(s_city_lbl, LV_LABEL_LONG_DOT);
    lv_obj_align(s_city_lbl, LV_ALIGN_BOTTOM_MID, 0, -60);

    // fiche de l'avion sélectionné
    s_card = make_panel(scr, 280, 124, 225);
    lv_obj_set_style_border_width(s_card, 2, 0);
    lv_obj_align(s_card, LV_ALIGN_BOTTOM_MID, 0, -38);
    s_card_title = make_label(s_card, &font_fr_28, 0xffffff, "");
    lv_obj_align(s_card_title, LV_ALIGN_TOP_LEFT, 18, 6);
    s_card_type = make_label(s_card, &font_fr_20, C_DIM, "");
    lv_obj_align(s_card_type, LV_ALIGN_TOP_RIGHT, -18, 12);
    s_card_body = make_label(s_card, &font_fr_20, C_TEXT, "");
    lv_obj_set_style_text_align(s_card_body, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_line_space(s_card_body, -2, 0);
    lv_obj_align(s_card_body, LV_ALIGN_TOP_LEFT, 18, 40);

    // message pendant la connexion / localisation
    s_msg = make_panel(scr, 300, 120, 215);
    lv_obj_center(s_msg);
    lv_obj_t *sp = lv_spinner_create(s_msg, 1000, 60);
    lv_obj_set_size(sp, 44, 44);
    lv_obj_set_style_arc_width(sp, 4, LV_PART_MAIN);
    lv_obj_set_style_arc_width(sp, 4, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(sp, lv_color_hex(0x0d3a24), LV_PART_MAIN);
    lv_obj_set_style_arc_color(sp, lv_color_hex(C_SWEEP), LV_PART_INDICATOR);
    lv_obj_align(sp, LV_ALIGN_TOP_MID, 0, 12);
    s_msg_lbl = make_label(s_msg, &font_fr_20, C_TEXT, "Démarrage");
    lv_obj_set_width(s_msg_lbl, 280);
    lv_label_set_long_mode(s_msg_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_align(s_msg_lbl, LV_ALIGN_TOP_MID, 0, 66);

    build_portal(scr);

    // ⚫ AJOUTÉ (24/10/2026) : écran "Code Wi-Fi"
    s_wcode = lv_obj_create(scr);
    lv_obj_remove_style_all(s_wcode);
    lv_obj_set_size(s_wcode, 480, 480);
    lv_obj_set_style_bg_color(s_wcode, lv_color_hex(0x020c07), 0);
    lv_obj_set_style_bg_opa(s_wcode, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_wcode, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t *wt = make_label(s_wcode, &font_fr_28, 0x5dffa8, "Code Wi-Fi");
    lv_obj_align(wt, LV_ALIGN_TOP_MID, 0, 40);
    s_wcode_ssid = make_label(s_wcode, &font_fr_20, C_DIM, "");
    lv_obj_set_width(s_wcode_ssid, 320);
    lv_label_set_long_mode(s_wcode_ssid, LV_LABEL_LONG_DOT);
    lv_obj_align(s_wcode_ssid, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_t *wbox = lv_obj_create(s_wcode);
    lv_obj_remove_style_all(wbox);
    lv_obj_set_size(wbox, 178, 178);
    lv_obj_set_style_bg_color(wbox, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(wbox, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(wbox, 12, 0);
    lv_obj_align(wbox, LV_ALIGN_CENTER, 0, -18);
    s_wcode_qr = lv_qrcode_create(wbox, 156, lv_color_black(), lv_color_white());
    lv_obj_center(s_wcode_qr);
    s_wcode_pass = make_label(s_wcode, &font_fr_28, 0xffffff, "");
    lv_obj_set_width(s_wcode_pass, 360);
    lv_label_set_long_mode(s_wcode_pass, LV_LABEL_LONG_WRAP);
    lv_obj_align(s_wcode_pass, LV_ALIGN_CENTER, 0, 110);
    lv_obj_t *wh = make_label(s_wcode, &font_fr_20, C_DIM, "Appui : retour au radar");
    lv_obj_align(wh, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_obj_add_flag(s_wcode, LV_OBJ_FLAG_HIDDEN);

    // appui long : anneau de progression sur le bord de l'écran
    s_hold_arc = lv_arc_create(scr);
    lv_obj_set_size(s_hold_arc, 474, 474);
    lv_obj_center(s_hold_arc);
    lv_arc_set_rotation(s_hold_arc, 270);
    lv_arc_set_bg_angles(s_hold_arc, 0, 360);
    lv_arc_set_range(s_hold_arc, 0, 100);
    lv_obj_set_style_arc_opa(s_hold_arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_hold_arc, 7, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_hold_arc, lv_color_hex(C_WARN), LV_PART_INDICATOR);
    lv_obj_set_style_opa(s_hold_arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(s_hold_arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_clear_flag(s_hold_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_hold_arc, LV_OBJ_FLAG_HIDDEN);
    s_hold_lbl = make_label(scr, &font_fr_20, C_WARN, "Maintenir : configuration Wi-Fi");
    lv_obj_set_style_bg_color(s_hold_lbl, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_hold_lbl, 200, 0);
    lv_obj_set_style_pad_all(s_hold_lbl, 8, 0);
    lv_obj_set_style_radius(s_hold_lbl, 10, 0);
    lv_obj_center(s_hold_lbl);
    lv_obj_add_flag(s_hold_lbl, LV_OBJ_FLAG_HIDDEN);

    s_enc_last = mt8901_get_count();
    apply_status();
    radar_led_start();
    lv_timer_create(ui_tick, TICK_MS, NULL);
}
