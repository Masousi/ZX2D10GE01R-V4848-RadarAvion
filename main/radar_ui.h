// ⚫ AJOUTÉ (24/10/2026) : RadarAvion — interface LVGL (radar, portail, messages) et anneau de LED
#pragma once

#include <stdint.h>
#include "app.h"

// À appeler depuis la tâche LVGL, après screen_init()
void radar_ui_init(void);

// Anneau de LED (radar_led.c) : tâche indépendante, alimentée par l'interface
void radar_led_start(void);
// sel_brg_deg < 0 : aucun avion sélectionné ; sel_rgb = 0xRRGGBB
void radar_led_set(app_phase_t phase, float sweep_deg, float sel_brg_deg, uint32_t sel_rgb);
