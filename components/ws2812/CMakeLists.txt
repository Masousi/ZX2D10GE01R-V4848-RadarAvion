# ⚫ AJOUTÉ (24/10/2026) : pilote LED WS2812 (RMT) de l'encodeur
idf_component_register(
	SRCS "ws2812.c" "led_strip_encoder.c"
	INCLUDE_DIRS .
	PRIV_REQUIRES "driver"
)
