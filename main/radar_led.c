// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — anneau de 13 LED WS2812 de l'encodeur, synchronisé avec l'écran :
// - radar : comète verte qui suit le balayage + LED dans la couleur de l'avion sélectionné, dans sa direction ;
// - connexion / localisation : comète bleue rapide ;
// - portail de configuration : respiration orange.
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "ws2812.h"
#include "radar_ui.h"

#define LED_PERIOD_MS   30
#define LED_MAX         40      // luminosité max (/255) : 13 LED à fond = ~0,8 A, trop pour un port USB
// Orientation de l'anneau : angle (0 = haut de l'écran, sens horaire) de la LED 0 et sens de numérotation.
// À ajuster si la comète ne tourne pas en phase avec le balayage de l'écran.
#define LED0_ANGLE_DEG  0.0f
#define LED_DIR         1

static volatile app_phase_t s_phase = APP_PHASE_BOOT;
static volatile float s_sweep_deg;
static volatile float s_sel_brg = -1.0f;
static volatile uint32_t s_sel_rgb;

void radar_led_set(app_phase_t phase, float sweep_deg, float sel_brg_deg, uint32_t sel_rgb) {
    s_phase = phase;
    s_sweep_deg = sweep_deg;
    s_sel_brg = sel_brg_deg;
    s_sel_rgb = sel_rgb;
}

static float led_angle(int i, int n) {
    float a = LED0_ANGLE_DEG + LED_DIR * i * 360.0f / n;
    return fmodf(a + 720.0f, 360.0f);
}

// Écart angulaire "derrière" la tête de la comète (0..360)
static float behind(float head, float a) {
    return fmodf(head - a + 720.0f, 360.0f);
}

static void led_task(void *arg) {
    int n = ws2812_count();
    for (;;) {
        float t = esp_timer_get_time() / 1e6f;
        app_phase_t phase = s_phase;
        for (int i = 0; i < n; i++) {
            float a = led_angle(i, n);
            uint8_t r = 0, g = 0, b = 0;
            if (phase == APP_PHASE_RADAR) {
                float d = behind(s_sweep_deg, a);
                float k = d < 120.0f ? (1.0f - d / 120.0f) : 0.0f;
                k *= k;
                g = (uint8_t)(LED_MAX * k);
                b = (uint8_t)(LED_MAX * 0.35f * k);
                float sel = s_sel_brg;
                if (sel >= 0.0f) {
                    float ds = fabsf(fmodf(sel - a + 540.0f, 360.0f) - 180.0f);   // écart -180..180 en valeur absolue
                    if (ds < 360.0f / n) {
                        float ks = 0.6f + 0.4f * sinf(t * 8.0f);
                        uint32_t c = s_sel_rgb;
                        r = (uint8_t)(((c >> 16) & 0xFF) * LED_MAX / 255 * ks);
                        g = (uint8_t)(((c >> 8) & 0xFF) * LED_MAX / 255 * ks);
                        b = (uint8_t)((c & 0xFF) * LED_MAX / 255 * ks);
                    }
                }
            } else if (phase == APP_PHASE_PORTAL) {
                float k = 0.15f + 0.85f * (0.5f + 0.5f * sinf(t * 2.0f));
                r = (uint8_t)(LED_MAX * k);
                g = (uint8_t)(LED_MAX * 0.45f * k);
            } else {
                float d = behind(fmodf(t * 360.0f, 360.0f), a);
                float k = d < 150.0f ? (1.0f - d / 150.0f) : 0.0f;
                b = (uint8_t)(LED_MAX * k * k);
                g = (uint8_t)(LED_MAX * 0.3f * k * k);
            }
            ws2812_set(i, r, g, b);
        }
        ws2812_show();
        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}

void radar_led_start(void) {
    if (ws2812_count() > 0) {
        xTaskCreatePinnedToCore(led_task, "leds", 3 * 1024, NULL, 3, NULL, 0);
    }
}
