<!-- ⚫ AJOUTÉ (24/10/2026) : firmware prêt à flasher, sans avoir à compiler -->
# Firmware précompilé

Image complète (bootloader + table de partitions + application), à flasher à l'adresse `0x0`.

| Fichier | Contenu | Taille | SHA-256 |
|---|---|---|---|
| `ZX2D10GE01R_radar_avion.bin` | **Radar Avion** | 1 405 856 o | `bbd0f7ff91117f47c3ce7f1495cd0f98be693dba69ac220cc7b1decbd7f8c268` |

- Cible : ESP32-S3 N16R8 (flash 16 Mo, PSRAM octale 8 Mo), ESP-IDF v5.0.9.
- Le réseau Wi-Fi se configure au premier démarrage via le portail intégré.

## Flasher

```sh
esptool.py --chip esp32s3 -p COM4 -b 460800 write_flash 0x0 ZX2D10GE01R_radar_avion.bin
```

Remplacer `COM4` par le port de la carte.
Ou avec l'outil web : https://espressif.github.io/esptool-js/ (adresse `0x0`).

## Régénérer

1. `idf.py build`
2. Dans `build/` :
   ```sh
   esptool.py --chip esp32s3 merge_bin -o ../firmware/ZX2D10GE01R_radar_avion.bin @flash_args
   ```
