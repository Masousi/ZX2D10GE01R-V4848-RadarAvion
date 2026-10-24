#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// #include "driver/pcnt.h"
//#include "driver/pulse_cnt.h"
// ⚫ MODIFIÉ (24/10/2026) : migration vers le nouveau pilote PCNT (l'ancien "driver/pcnt.h" est déprécié
// en IDF 5.x : warning "legacy pcnt driver is deprecated" à la compilation et au démarrage).
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define TAG = "mt8901";

#define PCNT_H_LIM_VAL      INT16_MAX
#define PCNT_L_LIM_VAL     INT16_MIN

// ⚫ 24/10/2026 : ancien filtre = 1000 cycles APB (80 MHz) = 12,5 µs, exprimé désormais en nanosecondes
#define PCNT_GLITCH_NS      12500

// static int unit = PCNT_UNIT_0;
static pcnt_unit_handle_t unit = NULL;

void mt8901_init(int16_t sig_pin, int16_t dir_pin) {
    /* Prepare configuration for the PCNT unit */
    /* Unité PCNT : mêmes limites que l'ancien driver (compteur 16 bits signé) */
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_H_LIM_VAL,
        .low_limit = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));

    /* Configure and enable the input filter */
    /* Filtre anti-rebond (équivalent pcnt_set_filter_value + pcnt_filter_enable) */
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = PCNT_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));

    /* Canal : SIG = impulsions (edge), DIR = sens (level) */
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = sig_pin,
        .level_gpio_num = dir_pin,
    };
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_config, &chan));

    // ⚫ 24/10/2026 : ancien pos_mode = DEC, neg_mode = INC
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    // ⚫ 24/10/2026 : ancien hctrl_mode = KEEP (DIR haut), lctrl_mode = REVERSE (DIR bas)
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // ⚫ 24/10/2026 : l'ancien driver activait les pull-ups sur SIG et DIR, on conserve ce comportement
    gpio_pullup_en(sig_pin);
    gpio_pullup_en(dir_pin);

    /* Remise à zéro puis démarrage du comptage */
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    /* Initialize PCNT's counter */
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));

    /* Everything is set up, now go to counting */
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

int16_t mt8901_get_count() {
    // int16_t count = 0;
    // pcnt_get_counter_value(unit, &count);
    int count = 0;
    pcnt_unit_get_count(unit, &count);
    return (int16_t)count;
}
