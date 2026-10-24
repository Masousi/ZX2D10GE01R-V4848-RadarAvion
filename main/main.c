// ⚫ MODIFIÉ (24/10/2026) : main minimal, démos supprimées.
// Initialise tous les périphériques du kit ZX2D10GE01R-V4848 puis fait tourner LVGL sur un écran vide :
// - écran RGB 480x480 (contrôleur configuré en SPI 9 bits) + rétroéclairage + LVGL (main/screen.c) ;
// - encodeur MT8901 (PCNT) + bouton-poussoir, déclarés comme encodeur LVGL (main/screen.c) ;
// - anneau de 13 LED WS2812 (RMT), éteint ;
// - moteur (GPIO MOTOR_PIN), à l'arrêt.
// Les broches libres du connecteur (EXT_PIN_0..3, cf. board.h) ne sont pas configurées.
// ⚫ AJOUTÉ (24/10/2026) : application RadarAvion (radar de trafic aérien via Internet) :
// NVS + tâche principale (app.c : Wi-Fi / portail RadarAvion_Wifi, géolocalisation IP, ADS-B) et
// interface radar (radar_ui.c) démarrée dans la tâche LVGL.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "board.h"
#include "ws2812.h"
// ⚫ AJOUTÉ (24/10/2026) : RadarAvion
#include "nvs_flash.h"
#include "app.h"
#include "radar_ui.h"

#define TAG "MAIN"

#define LED_COUNT 13   // anneau de LED de l'encodeur

extern void screen_init(void);

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

// Tick LVGL toutes les portTICK_PERIOD_MS
static void lvgl_tick_init(void) {
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = increase_lvgl_tick,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, portTICK_PERIOD_MS * 1000));
}

// Groupe LVGL par défaut relié à l'encodeur : sans groupe, LVGL ignore l'encodeur.
// Les widgets créés ensuite y sont ajoutés automatiquement.
static void encoder_group_init(void) {
    lv_group_t * g = lv_group_create();
    lv_group_set_default(g);
    for (lv_indev_t * indev = lv_indev_get_next(NULL); indev; indev = lv_indev_get_next(indev)) {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_ENCODER) {
            lv_indev_set_group(indev, g);
        }
    }
}

// Anneau de LED WS2812 : initialisé, toutes les LED éteintes
static void led_ring_init(void) {
    if (ws2812_init(LED_PIN, LED_COUNT) != ESP_OK) {
        ESP_LOGE(TAG, "ws2812_init a échoué");
        return;
    }
    ws2812_clear();
    ws2812_show();
}

// Moteur : sortie à l'arrêt
static void motor_init(void) {
    gpio_config_t motor_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << MOTOR_PIN
    };
    ESP_ERROR_CHECK(gpio_config(&motor_config));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_PIN, 0));
}

static void lvgl_task(void* arg) {
    // screen_init() alloue les frame buffers RGB 480x480 en PSRAM : CONFIG_SPIRAM obligatoire (sdkconfig.defaults)
    screen_init();
    lvgl_tick_init();
    encoder_group_init();
    ESP_LOGI(TAG, "Peripheriques initialises");
    // ⚫ AJOUTÉ (24/10/2026) : interface radar (toutes les fonctions LVGL restent dans cette tâche)
    radar_ui_init();

    for (;;) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ⚫ AJOUTÉ (24/10/2026) : NVS (identifiants Wi-Fi, zoom) ; effacée si la partition est d'un autre format
static void nvs_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void) {
    motor_init();
    led_ring_init();
    // ⚫ AJOUTÉ (24/10/2026) : NVS puis tâche principale RadarAvion (cœur 0, avec le Wi-Fi)
    nvs_init();
    app_start();
    // LVGL tourne dans sa propre tâche sur le cœur 1
    // xTaskCreatePinnedToCore(lvgl_task, NULL, 8 * 1024, NULL, 5, NULL, 1);
    // ⚫ MODIFIÉ (24/10/2026) : pile 8 Ko -> 12 Ko (dessin du radar : textes formatés, nombreux appels de dessin)
    xTaskCreatePinnedToCore(lvgl_task, NULL, 12 * 1024, NULL, 5, NULL, 1);
}
