# ⚫ MODIFIÉ (24/10/2026) : démos LVGL supprimées, seuls les sources de main/ sont compilés
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    # REQUIRES esp_lcd driver lvgl esp_timer button mt8901 ws2812
    # ⚫ MODIFIÉ (24/10/2026) : RadarAvion -> Wi-Fi, NVS, client HTTP(S), serveur web du portail, cJSON
    REQUIRES esp_lcd driver lvgl esp_timer button mt8901 ws2812
             nvs_flash esp_wifi esp_netif esp_event esp_http_client esp_http_server mbedtls json
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
