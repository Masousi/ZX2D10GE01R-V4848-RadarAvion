// ⚫ AJOUTÉ (24/10/2026) : pilote minimal WS2812 via le périphérique RMT (voir ws2812.h)
#include <string.h>
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "ws2812.h"

#define WS2812_RESOLUTION_HZ 10000000 // 10 MHz : 1 tick = 0,1 µs (précision requise par le protocole WS2812)
#define WS2812_MAX_LEDS      64

static rmt_channel_handle_t s_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
static uint8_t s_pixels[WS2812_MAX_LEDS * 3]; // ordre GRB attendu par les WS2812
static int s_count = 0;

esp_err_t ws2812_init(int gpio, int led_count)
{
    if (led_count <= 0 || led_count > WS2812_MAX_LEDS) {
        return ESP_ERR_INVALID_ARG;
    }
    s_count = led_count;

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio,
        .mem_block_symbols = 64,
        .resolution_hz = WS2812_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_chan_config, &s_chan);
    if (err != ESP_OK) return err;

    led_strip_encoder_config_t encoder_config = {
        .resolution = WS2812_RESOLUTION_HZ,
    };
    err = rmt_new_led_strip_encoder(&encoder_config, &s_encoder);
    if (err != ESP_OK) return err;

    err = rmt_enable(s_chan);
    if (err != ESP_OK) return err;

    ws2812_clear();
    return ws2812_show();
}

void ws2812_set(int index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < 0 || index >= s_count) return;
    s_pixels[index * 3 + 0] = g;
    s_pixels[index * 3 + 1] = r;
    s_pixels[index * 3 + 2] = b;
}

void ws2812_clear(void)
{
    memset(s_pixels, 0, sizeof(s_pixels));
}

esp_err_t ws2812_show(void)
{
    if (!s_chan) return ESP_ERR_INVALID_STATE;
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    esp_err_t err = rmt_transmit(s_chan, s_encoder, s_pixels, s_count * 3, &tx_config);
    if (err != ESP_OK) return err;
    return rmt_tx_wait_all_done(s_chan, 100);
}

int ws2812_count(void)
{
    return s_count;
}
