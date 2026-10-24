// ⚫ AJOUTÉ (24/10/2026) : pilote minimal pour les LED adressables (WS2812/SK6812, ordre GRB) de l'encodeur.
// Basé sur l'encodeur RMT de l'exemple ESP-IDF peripherals/rmt/led_strip (led_strip_encoder.c/h).
#pragma once

#include <stdint.h>
#include "esp_err.h"

// Initialise la chaîne de LED sur `gpio` (max 64 LED). Toutes éteintes au départ.
esp_err_t ws2812_init(int gpio, int led_count);

// Fixe la couleur d'une LED en mémoire (prise en compte au prochain ws2812_show()).
void ws2812_set(int index, uint8_t r, uint8_t g, uint8_t b);

// Éteint toutes les LED en mémoire.
void ws2812_clear(void);

// Envoie les couleurs aux LED.
esp_err_t ws2812_show(void);

// Nombre de LED configurées.
int ws2812_count(void);
