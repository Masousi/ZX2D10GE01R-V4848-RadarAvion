#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "board.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "button.h"
#include "mt8901.h"

#define TAG "RGB"

#define ECO_O(y) (y>0)? -1:1
#define ECO_STEP(x) x? ECO_O(x):0

static button_t *g_btn;

static esp_lcd_panel_handle_t g_panel_handle = NULL;
// ⚫ AJOUTÉ (24/10/2026) : donné à chaque VSYNC (fin de trame) par __qsmd_rgb_on_vsync()
static SemaphoreHandle_t s_vsync_sem;
// ⚫ AJOUTÉ (25/10/2026) : frame buffer 0 du panneau (seul buffer depuis lequel le DMA peut être relancé)
static void *s_fb0;

static void __qsmd_rgb_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // esp_lcd_panel_draw_bitmap(g_panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_p);
    // lv_disp_flush_ready(disp_drv);
    // ⚫ MODIFIÉ (24/10/2026) : synchronisation VSYNC (correction du clignotement).
    // draw_bitmap() ne fait que programmer le passage sur ce buffer pour la trame SUIVANTE (IDF 5.0.9,
    // double_fb) : la trame en cours est encore lue dans l'autre buffer. Appeler lv_disp_flush_ready()
    // tout de suite faisait redessiner par LVGL le buffer en cours d'affichage -> images à moitié
    // dessinées visibles = clignotement permanent (le radar redessine tout l'écran à chaque image).
    // 1) attendre un début de trame (loin de la fin, où le DMA lit le pointeur du buffer suivant),
    // 2) programmer le nouveau buffer,
    // 3) attendre le début de trame suivant : l'écran lit alors le nouveau buffer, l'ancien est libre.
    // xSemaphoreTake(s_vsync_sem, 0);
    // xSemaphoreTake(s_vsync_sem, pdMS_TO_TICKS(50));
    // ⚫ RETIRÉ (24/10/2026) : étape 1 inutile avec CONFIG_LCD_RGB_RESTART_IN_VSYNC (sdkconfig.defaults) :
    // le pilote relance le DMA sur le buffer courant à chaque VSYNC, le changement de buffer a donc lieu
    // exactement à la trame suivante. Cette attente coûtait jusqu'à une trame (~17 ms) par image.
    int64_t t0 = esp_timer_get_time();
    esp_lcd_panel_draw_bitmap(g_panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_p);
    // ⚫ AJOUTÉ (24/10/2026) : draw_bitmap() vide le cache vers la PSRAM (460 Ko) : il doit rester bien
    // plus court qu'une trame (~17 ms) pour que l'étape 2 tombe loin de la fin de trame. Contrôle série :
    // ⚫ 25/10/2026 : en mode bounce buffer, draw_bitmap() ne vide plus le cache (le CPU relit la PSRAM via le
    // cache) : il ne fait que programmer le buffer, quelques µs.
    static int64_t s_bmp_max_us;
    static int s_bmp_n;
    int64_t dt = esp_timer_get_time() - t0;
    if (dt > s_bmp_max_us) {
        s_bmp_max_us = dt;
    }
    if (++s_bmp_n == 300) {
        ESP_LOGI(TAG, "draw_bitmap max %lld us (300 images)", s_bmp_max_us);
        s_bmp_n = 0;
        s_bmp_max_us = 0;
    }
    // ⚫ MODIFIÉ (25/10/2026) : avec les bounce buffers (cf. screen_init), s_vsync_sem est donné quand le
    // pilote a fini de COPIER une trame complète vers les bounce buffers et prend le buffer programmé
    // (bb_fb_index = cur_fb_index) : l'ancien buffer n'est plus lu du tout, LVGL peut y dessiner.
    // Même interruption, même cœur que cette tâche -> aucune course possible.
    // xSemaphoreTake(s_vsync_sem, 0);
    // xSemaphoreTake(s_vsync_sem, pdMS_TO_TICKS(50));
    // ⚫ MODIFIÉ (25/10/2026) : bounce buffers et CONFIG_LCD_RGB_RESTART_IN_VSYNC retirés (image décalée et
    // dédoublée horizontalement : la relance du DMA à chaque VSYNC tombait parfois en pleine ligne).
    // Sans relance, le DMA passe sur le nouveau buffer en fin de trame, en suivant le chaînage des
    // descripteurs (dont le dernier est lu ~4 lignes avant la fin). On attend DEUX VSYNC après
    // draw_bitmap() : VSYNC A = fin de la trame en cours (bascule faite, ou ratée si draw_bitmap() est
    // tombé juste après la lecture du dernier descripteur), VSYNC B = fin de la trame suivante : dans les
    // deux cas l'ancien buffer n'est plus lu, LVGL peut y dessiner. ~1,5 trame d'attente en moyenne.
    // ⚫ AJOUTÉ (25/10/2026) : resynchronisation de l'écran. Une lecture/écriture en flash (NVS, Wi-Fi au
    // démarrage) suspend le cache : le DMA prend du retard et l'image reste décalée horizontalement
    // (bord droit qui réapparaît à gauche) tant que le DMA n'est pas relancé. esp_lcd_rgb_panel_restart()
    // relance le DMA au VSYNC suivant, mais (IDF 5.0.9) TOUJOURS depuis le début du buffer 0 : on ne la
    // demande donc que lorsque le buffer qu'on vient de programmer EST le buffer 0, au plus 1 fois/s.
    // (C'est aussi pourquoi CONFIG_LCD_RGB_RESTART_IN_VSYNC, relance à chaque trame, faisait clignoter :
    // une image sur deux, l'écran réaffichait l'ancien contenu du buffer 0.)
    static int64_t s_last_restart_us;
    if (color_p == s_fb0 && t0 - s_last_restart_us > 1000000) {
        esp_lcd_rgb_panel_restart(g_panel_handle);
        s_last_restart_us = t0;
    }
    xSemaphoreTake(s_vsync_sem, 0);
    xSemaphoreTake(s_vsync_sem, pdMS_TO_TICKS(50));
    xSemaphoreTake(s_vsync_sem, pdMS_TO_TICKS(50));
    lv_disp_flush_ready(disp_drv);
}

// ⚫ AJOUTÉ (24/10/2026) : interruption de fin de trame (VSYNC) -> débloque __qsmd_rgb_disp_flush()
// ⚫ MODIFIÉ (25/10/2026) : branchée sur on_bounce_frame_finish (fin de copie d'une trame) au lieu de on_vsync
// ⚫ MODIFIÉ (25/10/2026) : de nouveau branchée sur on_vsync (bounce buffers retirés)
static IRAM_ATTR bool __qsmd_rgb_on_vsync(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_vsync_sem, &woken);
    return woken == pdTRUE;
}

// ⚫ AJOUTÉ (25/10/2026) : mesure du rendu LVGL (temps de rendu + flush, appelé après chaque image)
static void __qsmd_rgb_monitor(lv_disp_drv_t *disp_drv, uint32_t time_ms, uint32_t px)
{
    static uint32_t s_sum_ms, s_max_ms, s_n;
    s_sum_ms += time_ms;
    if (time_ms > s_max_ms) {
        s_max_ms = time_ms;
    }
    if (++s_n == 300) {
        ESP_LOGI(TAG, "rendu LVGL : moyen %u ms, max %u ms (300 images)", (unsigned)(s_sum_ms / s_n), (unsigned)s_max_ms);
        s_sum_ms = s_max_ms = s_n = 0;
    }
}

// ⚫ AJOUTÉ (25/10/2026) : le panneau est créé depuis le cœur 0. Ses interruptions (copie PSRAM -> bounce
// buffers, ~30 Mo/s) sont attachées au cœur qui crée le panneau : sur le cœur 0, elles ne ralentissent
// pas le rendu LVGL (cœur 1).
typedef struct {
    esp_lcd_rgb_panel_config_t *cfg;
    SemaphoreHandle_t done;
} panel_create_args_t;

static void __qsmd_rgb_create_task(void *arg)
{
    panel_create_args_t *a = (panel_create_args_t *)arg;
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(a->cfg, &g_panel_handle));
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        // .on_vsync = __qsmd_rgb_on_vsync,
        // .on_bounce_frame_finish = __qsmd_rgb_on_vsync,
        // ⚫ MODIFIÉ (25/10/2026) : bounce buffers retirés -> VSYNC
        .on_vsync = __qsmd_rgb_on_vsync,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(g_panel_handle, &cbs, NULL));
    xSemaphoreGive(a->done);
    vTaskDelete(NULL);
}

void qmsd_rgb_init(esp_lcd_rgb_panel_config_t *panel_config)
{
    static lv_disp_drv_t disp_drv;
    int buffer_size;
    void *buf1 = NULL;
    void *buf2 = NULL;
	static lv_disp_draw_buf_t draw_buf;

    lv_init();

    // ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(panel_config, &g_panel_handle));
    // ⚫ AJOUTÉ (24/10/2026) : synchronisation de l'affichage sur le VSYNC (cf. __qsmd_rgb_disp_flush)
    s_vsync_sem = xSemaphoreCreateBinary();
    // esp_lcd_rgb_panel_event_callbacks_t cbs = {
    //     .on_vsync = __qsmd_rgb_on_vsync,
    // };
    // ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(g_panel_handle, &cbs, NULL));
    // ⚫ MODIFIÉ (25/10/2026) : création du panneau + callbacks sur le cœur 0 (cf. __qsmd_rgb_create_task)
    panel_create_args_t args = { .cfg = panel_config, .done = xSemaphoreCreateBinary() };
    // xTaskCreatePinnedToCore(__qsmd_rgb_create_task, "lcd_init", 4096, &args, 5, NULL, 0);
    // ⚫ MODIFIÉ (25/10/2026) : cœur 1 (comme à l'origine) : sans bounce buffer, l'interruption ne fait que
    // signaler le VSYNC ; sur le cœur 0 elle serait retardée par le Wi-Fi
    xTaskCreatePinnedToCore(__qsmd_rgb_create_task, "lcd_init", 4096, &args, 5, NULL, 1);
    xSemaphoreTake(args.done, portMAX_DELAY);
    vSemaphoreDelete(args.done);
    ESP_ERROR_CHECK(esp_lcd_panel_reset(g_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(g_panel_handle));

    buffer_size = panel_config->timings.h_res * panel_config->timings.v_res;
    esp_lcd_rgb_panel_get_frame_buffer(g_panel_handle, 2, &buf1, &buf2);
    s_fb0 = buf1;   // ⚫ AJOUTÉ (25/10/2026) : cf. resynchronisation dans __qsmd_rgb_disp_flush()
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buffer_size);

    lv_disp_drv_init(&disp_drv);         
    disp_drv.flush_cb = __qsmd_rgb_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.hor_res = panel_config->timings.h_res;
    disp_drv.ver_res = panel_config->timings.v_res;
    // ⚫ AJOUTÉ (24/10/2026) : les 2 buffers LVGL SONT les frame buffers du panneau (double_fb).
    // esp_lcd_panel_draw_bitmap() ne copie rien dans ce cas : il bascule l'affichage sur ce buffer.
    // Sans full_refresh, LVGL n'écrit que la zone modifiée, tassée au début du buffer -> dès la 1re
    // animation, le panneau affiche un buffer quasi vide : écran noir.
    // full_refresh : chaque image est redessinée entièrement dans le buffer suivant (cf. exemple IDF rgb_panel).
    disp_drv.full_refresh = 1;
    // ⚫ AJOUTÉ (25/10/2026) : statistiques de rendu sur le port série
    disp_drv.monitor_cb = __qsmd_rgb_monitor;
    lv_disp_drv_register(&disp_drv);
}

static spi_device_handle_t g_screen_spi;

static void lcd_cmd(spi_device_handle_t spi, const uint16_t data) {
    esp_err_t ret;
    uint16_t tmp_cmd =(data | 0x0000);
    spi_transaction_ext_t trans =(spi_transaction_ext_t) {
        .base = {
            .flags = SPI_TRANS_VARIABLE_CMD,
            .cmd = tmp_cmd,
        },
        .command_bits = 9,
    };
    ret = spi_device_transmit(spi,(spi_transaction_t *)&trans);
    assert(ret == ESP_OK);          //Should have had no issues.
}

static void lcd_data(spi_device_handle_t spi, const uint16_t data) {
    esp_err_t ret;
    uint16_t tmp_cmd =(data | 0x0100);
    spi_transaction_ext_t trans =(spi_transaction_ext_t) {
        .base = {
            .flags = SPI_TRANS_VARIABLE_CMD,
            .cmd = tmp_cmd,
        },
        .command_bits = 9,
    };
    ret = spi_device_transmit(spi,(spi_transaction_t *)&trans);

    assert(ret == ESP_OK);          //Should have had no issues.
}

static void SPI_WriteComm(uint16_t cmd) {
    lcd_cmd(g_screen_spi, cmd);
}

static void SPI_WriteData(uint16_t data) {
    lcd_data(g_screen_spi, data);
}

static void rgb_driver_init(void) {
	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x13);
	SPI_WriteComm(0xEF);
	SPI_WriteData(0x08);

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x10);

	SPI_WriteComm(0xC0);
	SPI_WriteData(0x3B);//Scan line	
	SPI_WriteData(0x00);

	SPI_WriteComm(0xC1);
	SPI_WriteData(0x0B);	//VBP
	SPI_WriteData(0x02);

	SPI_WriteComm(0xC2);
	SPI_WriteData(0x07);
	SPI_WriteData(0x02);

	SPI_WriteComm(0xCC);
	SPI_WriteData(0x10);

	SPI_WriteComm(0xCD);//RGB format
	SPI_WriteData(0x08);        //?565???    666??

	SPI_WriteComm(0xB0); // IPS   
	SPI_WriteData(0x00); // 255 
	SPI_WriteData(0x11); // 251    
	SPI_WriteData(0x16); // 247  down
	SPI_WriteData(0x0e); // 239    
	SPI_WriteData(0x11); // 231    
	SPI_WriteData(0x06); // 203    
	SPI_WriteData(0x05); // 175 
	SPI_WriteData(0x09); // 147    
	SPI_WriteData(0x08); // 108    
	SPI_WriteData(0x21); // 80  
	SPI_WriteData(0x06); // 52   
	SPI_WriteData(0x13); // 24    
	SPI_WriteData(0x10); // 16    
	SPI_WriteData(0x29); // 8    down
	SPI_WriteData(0x31); // 4    
	SPI_WriteData(0x18); // 0   

	SPI_WriteComm(0xB1);//  IPS	   
	SPI_WriteData(0x00);//  255 
	SPI_WriteData(0x11);//  251
	SPI_WriteData(0x16);//  247   down
	SPI_WriteData(0x0e);//  239
	SPI_WriteData(0x11);//  231
	SPI_WriteData(0x07);//  203    
	SPI_WriteData(0x05);//  175
	SPI_WriteData(0x09);//  147  
	SPI_WriteData(0x09);//  108  
	SPI_WriteData(0x21);//  80 
	SPI_WriteData(0x05);//  52   
	SPI_WriteData(0x13);//  24 
	SPI_WriteData(0x11);//  16 
	SPI_WriteData(0x2a);//  8  down 
	SPI_WriteData(0x31);//  4  
	SPI_WriteData(0x18);//  0  

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);

	SPI_WriteComm(0xB0);  //VOP  3.5375+ *x 0.0125
	SPI_WriteData(0x6d);  //5D
	
	SPI_WriteComm(0xB1); 	//VCOM amplitude setting  
	SPI_WriteData(0x37);  //
	
	SPI_WriteComm(0xB2); 	//VGH Voltage setting  
	SPI_WriteData(0x81);	//12V

	SPI_WriteComm(0xB3);
	SPI_WriteData(0x80);

	SPI_WriteComm(0xB5); 	//VGL Voltage setting  
	SPI_WriteData(0x43);	//-8.3V

	SPI_WriteComm(0xB7);
	SPI_WriteData(0x85);

	SPI_WriteComm(0xB8);
	SPI_WriteData(0x20);

	SPI_WriteComm(0xC1);
	SPI_WriteData(0x78);

	SPI_WriteComm(0xC2);
	SPI_WriteData(0x78);

	SPI_WriteComm(0xD0);
	SPI_WriteData(0x88);

	SPI_WriteComm(0xE0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x02);

	SPI_WriteComm(0xE1);
	SPI_WriteData(0x03);	
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);
	SPI_WriteData(0x04);	
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);
	SPI_WriteData(0x20);
	SPI_WriteData(0x20);

	SPI_WriteComm(0xE2);
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	  
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);

	SPI_WriteComm(0xE3);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xE4);
	SPI_WriteData(0x22);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xE5);		
	SPI_WriteData(0x05);	
	SPI_WriteData(0xEC);	
	SPI_WriteData(0xA0);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x07);	
	SPI_WriteData(0xEE);	
	SPI_WriteData(0xA0);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xE6);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x11);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xE7);
	SPI_WriteData(0x22);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xE8);		
	SPI_WriteData(0x06);	
	SPI_WriteData(0xED);	
	SPI_WriteData(0xA0);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x08);	
	SPI_WriteData(0xEF);	
	SPI_WriteData(0xA0); 
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);	
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xEB);
	SPI_WriteData(0x00); 	
	SPI_WriteData(0x00);
	SPI_WriteData(0x40);
	SPI_WriteData(0x40);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);  

	SPI_WriteComm(0xED);  
	SPI_WriteData(0xFF); 
	SPI_WriteData(0xFF);  
	SPI_WriteData(0xFF); 	
	SPI_WriteData(0xBA); 		
	SPI_WriteData(0x0A); 	
	SPI_WriteData(0xBF); 	
	SPI_WriteData(0x45); 	
	SPI_WriteData(0xFF); 
	SPI_WriteData(0xFF);  
	SPI_WriteData(0x54); 	
	SPI_WriteData(0xFB); 	
	SPI_WriteData(0xA0); 	
	SPI_WriteData(0xAB); 	
	SPI_WriteData(0xFF); 
	SPI_WriteData(0xFF); 
	SPI_WriteData(0xFF); 

	SPI_WriteComm(0xEF);
	SPI_WriteData(0x10); 
	SPI_WriteData(0x0D); 
	SPI_WriteData(0x04); 
	SPI_WriteData(0x08); 
	SPI_WriteData(0x3F); 
	SPI_WriteData(0x1F);

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x13);

	SPI_WriteComm(0xEF);
	SPI_WriteData(0x08);

	SPI_WriteComm(0xFF);
	SPI_WriteData(0x77);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0x36);
	SPI_WriteData(0x00);

	SPI_WriteComm(0x3A);
	SPI_WriteData(0x66);  //55/50=16bit(RGB565);66=18bit(RGB666);77?????3AH?=24bit(RGB888)  

	SPI_WriteComm(0x11);

	vTaskDelay(pdMS_TO_TICKS(120));
	SPI_WriteComm(0x29);
	vTaskDelay(pdMS_TO_TICKS(20));
}

void qmsd_rgb_spi_init() {
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SPI_CLK,
        .mosi_io_num = LCD_SPI_DATA0,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10 * 1024,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M,   //Clock out at 10 MHz
        .mode = 0,                               //SPI mode 0
        .spics_io_num = LCD_SPI_CS,              //CS pin
        .queue_size = 7,                         //We want to be able to queue 7 transactions at a time
    };
    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &g_screen_spi));
    rgb_driver_init();

	spi_bus_remove_device(g_screen_spi);
	spi_bus_free(SPI2_HOST);
}

void __qmsd_encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static int16_t cont_last = 0;
    int16_t cont_now = mt8901_get_count();
    data->enc_diff = ECO_STEP(cont_now - cont_last);
    cont_last = cont_now;
    if (button_isPressed(g_btn)){
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

void __qsmd_encoder_init(void)
{
    static lv_indev_drv_t indev_drv;

    // g_btn = button_attch(3, 1, 10);
    // ⚫ MODIFIÉ (24/10/2026) : le bouton est actif à l'état BAS (GPIO3 = 1 au repos avec pull-up, 0 appuyé).
    // Avec press_value = 1, LVGL voyait le bouton appuyé en permanence et ignore alors la rotation
    // (lv_indev.c : "enc_diff valid only with released button") -> l'encodeur ne faisait rien.
    g_btn = button_attch(BTN_PIN, 0, 10);
    // mt8901_init(5,6);
    mt8901_init(ENCODER_A_PIN, ENCODER_B_PIN);

    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = __qmsd_encoder_read;
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    lv_indev_drv_register(&indev_drv);
}

void screen_init(void) {
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_BK_LIGHT
    };
    // Initialize the GPIO of backlight
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL));

    qmsd_rgb_spi_init();

    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16,
        .psram_trans_align = 64,
        .pclk_gpio_num = LCD_PCLK_GPIO,
        .vsync_gpio_num = LCD_VSYNC_GPIO,
        .hsync_gpio_num = LCD_HSYNC_GPIO,
        .de_gpio_num = LCD_DE_GPIO,
        .disp_gpio_num = LCD_DISP_EN_GPIO,
        .data_gpio_nums = {
            LCD_DATA0_GPIO,
            LCD_DATA1_GPIO,
            LCD_DATA2_GPIO,
            LCD_DATA3_GPIO,
            LCD_DATA4_GPIO,
            LCD_DATA5_GPIO,
            LCD_DATA6_GPIO,
            LCD_DATA7_GPIO,
            LCD_DATA8_GPIO,
            LCD_DATA9_GPIO,
            LCD_DATA10_GPIO,
            LCD_DATA11_GPIO,
            LCD_DATA12_GPIO,
            LCD_DATA13_GPIO,
            LCD_DATA14_GPIO,
            LCD_DATA15_GPIO,
        },
        .timings = {
            .pclk_hz = 15000000,
            .h_res = 480,
            .v_res = 480,
            .hsync_pulse_width = 10,
            .hsync_back_porch = 10,
            .hsync_front_porch = 10, 
            .vsync_pulse_width = 2,
            .vsync_back_porch = 12,
            .vsync_front_porch = 14,
        },
        .flags.fb_in_psram = 1,
        .flags.double_fb = 1,
        .flags.refresh_on_demand = 0,   // Mannually control refresh operation
        // .bounce_buffer_size_px = 0,
        // ⚫ MODIFIÉ (25/10/2026) : correction du clignotement avec le Wi-Fi. Sans bounce buffer, le DMA de
        // l'écran lit l'image directement en PSRAM (30 Mo/s en continu) ; quand LVGL + Wi-Fi saturent la
        // PSRAM, le DMA manque de données -> image qui saute / clignote. Avec 2 bounce buffers de 10 lignes
        // en RAM interne (2 x 9,6 Ko), le DMA ne lit que la RAM interne ; le CPU (interruptions, cœur 0)
        // recopie la PSRAM dans ces tampons. Recommandation Espressif pour ESP32-S3 + Wi-Fi + PSRAM.
        // (taille de l'image = multiple pair de la taille d'un bounce buffer : 480x480 / (2 x 4800) = 24)
        // .bounce_buffer_size_px = 480 * 10,
        // ⚫ RETIRÉ (25/10/2026) : bounce buffers : avec la relance du DMA à chaque VSYNC, interruptions en
        // retard (cœur 0 + Wi-Fi) -> image décalée et répétée horizontalement. Retour à la configuration
        // d'origine (DMA lisant directement les frame buffers en PSRAM), qui affichait les démos sans défaut.
        .bounce_buffer_size_px = 0,
        .clk_src = LCD_CLK_SRC_PLL160M,
    };

    qmsd_rgb_init(&panel_config);
	__qsmd_encoder_init();

    ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL));
}
