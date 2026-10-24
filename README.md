# ZX2D10GE01R
ZX2D10GE01R base on https://github.com/espressif/esp-idf/tree/release/v5.0

<!-- ⚫ MODIFIÉ (24/10/2026) : projet nettoyé, démos supprimées, main minimal -->
> **Origine :** fork de [wireless-tag-com/ZX2D10GE01R-V4848](https://github.com/wireless-tag-com/ZX2D10GE01R-V4848).
> Projet de base minimal : `main/main.c` initialise tous les périphériques de la carte et laisse un écran LVGL vide,
> prêt à recevoir une application.

<!-- ⚫ AJOUTÉ (24/10/2026) : application RadarAvion -->
## ✈️ RadarAvion

Radar de trafic aérien en temps réel, inspiré de [MatixYo/ESP32-Plane-Radar](https://github.com/MatixYo/ESP32-Plane-Radar) :
**pas de récepteur radio ni d'antenne SDR**, la carte se connecte au Wi-Fi et interroge les API ADS-B
communautaires ouvertes ([adsb.fi](https://github.com/adsbfi/opendata), secours [adsb.lol](https://api.adsb.lol)).

### Premier démarrage : configuration du Wi-Fi

1. Sans réseau enregistré, la carte crée le point d'accès ouvert **`RadarAvion_Wifi`** ; l'écran affiche un QR code.
2. Scanner le QR code (ou rejoindre `RadarAvion_Wifi` à la main) : la page de configuration s'ouvre seule
   (portail captif), sinon ouvrir **http://192.168.4.1**.
3. Choisir le réseau, saisir le mot de passe → *Enregistrer et redémarrer*.
   Option *Position* : coordonnées précises (sinon, géolocalisation par l'IP).

Le portail s'ouvre aussi si la connexion échoue (30 s) ; avec un réseau déjà enregistré, il retente
la connexion après 5 min sans visiteur (box pas encore démarrée après une coupure), ou tout de suite sur appui.

### Position

Déduite de l'**adresse IP publique** de la box ([ip-api.com](https://ip-api.com), secours [ipwho.is](https://ipwho.is)).
⚠️ La précision est celle de la ville du fournisseur d'accès (parfois à plusieurs dizaines de km) :
pour centrer le radar sur chez vous, saisir latitude/longitude dans le portail.

### Écran et commandes

| Action | Effet |
|---|---|
| Rotation de l'encodeur | zoom : rayon 2, 5, 10, 20, 40, 80, 160 km (mémorisé) |
| Appui court | sélection de l'avion suivant (du plus proche au plus lointain) : fiche indicatif, type, altitude, montée/descente, vitesse, cap, distance, immatriculation ; puis plus de sélection |
| Appui long 3 s | portail de configuration Wi-Fi (anneau orange de progression) |
| Appui moyen (relâché entre 0,6 et 3 s) | <!-- ⚫ AJOUTÉ (24/10/2026) --> écran **Code Wi-Fi** : nom du réseau, mot de passe et QR code pour y connecter un téléphone (appui = retour, fermeture auto après 1 min) |

- Balayage rotatif (4 s/tour) avec traînée, cercles de distance, points cardinaux (N en orange).
- Avions : triangle orienté selon la route, **couleur selon l'altitude** (orange < 3 000 ft, jaune < 10 000,
  vert < 20 000, cyan < 30 000, violet au-delà, gris au sol), effet phosphore ravivé au passage du balayage,
  traînée des positions précédentes, étiquette indicatif + altitude (masquées au-delà de 25 avions).
- Rafraîchissement des données toutes les 5 s ; entre deux requêtes, la position est **extrapolée**
  (vitesse sol + route) : les avions se déplacent en continu.
- Anneau de LED : comète verte synchronisée avec le balayage, LED clignotante dans la direction (et la couleur)
  de l'avion sélectionné ; bleu pendant la connexion, orange en mode configuration.
  Si la comète n'est pas en phase avec l'écran : `LED0_ANGLE_DEG` / `LED_DIR` dans `main/radar_led.c`.
- Sens du zoom : `ENC_CW_ZOOMS_IN` dans `main/radar_ui.c`.

### Fichiers RadarAvion

| Fichier | Rôle |
|---|---|
| `main/app.c`, `app.h` | tâche principale (Wi-Fi → géolocalisation → radar), état partagé |
| `main/app_config.c` | NVS : identifiants Wi-Fi, position manuelle, zoom |
| `main/wifi_portal.c` | Wi-Fi station (reconnexion auto) + portail `RadarAvion_Wifi` (page web, DNS captif) |
| `main/net_http.c` | client HTTP(S) (certificats embarqués), géolocalisation IP |
| `main/adsb.c` | requêtes ADS-B toutes les 5 s, analyse JSON, traînées |
| `main/radar_ui.c` | interface LVGL : radar, fiche avion, écrans de connexion et de configuration |
| `main/radar_led.c` | anneau de LED |
| `main/font_fr_20.c`, `font_fr_28.c` | polices avec accents (Latin-1) |

Journal série toutes les 10 s : images/s, temps de dessin, nombre d'avions, mémoire libre.

<!-- ⚫ AJOUTÉ (24/10/2026) -->
Affichage : le fond (disques, cercles, graduations) est pré-dessiné dans un canvas et redessiné seulement au
changement de zoom ; l'image est synchronisée sur le VSYNC de l'écran (`main/screen.c`) pour éviter le
clignotement (LVGL ne dessine jamais dans le buffer en cours d'affichage).

## Matériel / environnement

- ESP32-S3 N16R8 : flash 16 Mo, PSRAM **octale** 8 Mo, écran RGB 480×480 (frame buffer en PSRAM, `fb_in_psram`).
- Encodeur : rotation sur GPIO 5/6 (MT8901), bouton-poussoir sur GPIO 3 (actif bas), anneau de **13 LED WS2812** sur GPIO 4.
- Moteur sur GPIO 7, broches libres du connecteur : GPIO 1, 2, 20, 19 (`EXT_PIN_0..3`).
- ESP-IDF **v5.0.9**.

## Ce qu'initialise `main.c`

| Périphérique | Initialisation | État au démarrage |
|---|---|---|
| Écran RGB 480×480 + rétroéclairage | `screen_init()` (`main/screen.c`) : config du contrôleur en SPI 9 bits, panneau RGB double buffer, LVGL (`full_refresh`) | écran LVGL vide, rétroéclairage allumé |
| Encodeur + bouton | `mt8901_init()` / `button_attch()` déclarés comme encodeur LVGL, groupe LVGL par défaut | les widgets créés sont pilotables à l'encodeur |
| Anneau de LED | `ws2812_init()` (RMT) | éteint |
| Moteur | GPIO en sortie | à l'arrêt (niveau 0) |
| Tick LVGL | `esp_timer` périodique | — |

Les broches `EXT_PIN_0..3` ne sont pas configurées.

## Compiler, flasher

```sh
idf.py -p COM4 build flash monitor
```

Dans VS Code, sans terminal : `Ctrl+Shift+P` → *Tasks: Run Task* → **ESP: Build + Flash + Monitor**
(tâches dans `.vscode/tasks.json`, fichier local non versionné). Quitter le moniteur : `Ctrl+]`.

Journal série attendu :

```
esp_psram: Found 8MB PSRAM device
cpu_start: cpu freq: 240000000 Hz
MAIN: Peripheriques initialises
```

### Dépannage

| Symptôme | Cause probable | Solution |
|---|---|---|
| Écran noir + `no mem for frame buffer` + redémarrages | PSRAM désactivée (`sdkconfig` régénéré) | supprimer `sdkconfig`, puis `idf.py build` (reprend `sdkconfig.defaults`) |
| `SPI Flash Size : 2MB` dans le journal | flash mal configurée | *Serial flasher config → Flash size* = 16 MB |
| Erreur de build « app partition is too small » | table de partitions par défaut | *Partition Table* → `partitions_lvgl.csv` |
| Écran noir, aucune erreur | tas LVGL fixe saturé (`LV_MEM_CUSTOM` désactivé) | activer `LV_MEM_CUSTOM` (*LVGL → Memory settings*) |
| L'encodeur ne fait rien | bouton déclaré actif haut | `button_attch(BTN_PIN, 0, 10)` dans `main/screen.c` (déjà corrigé) |
| Écran noir dès la première animation | `full_refresh` désactivé alors que les buffers LVGL sont les frame buffers du panneau | `disp_drv.full_refresh = 1` dans `main/screen.c` (déjà corrigé) |
| `Rendering this widget needs LV_COLOR_SCREEN_TRANSP 1` | widgets semi-transparents | `LV_COLOR_SCREEN_TRANSP` (activé dans `sdkconfig.defaults`) |
| La carte redémarre quand les LED s'allument | 13 LED à 100 % ≈ 0,8 A : alimentation USB insuffisante | port/chargeur USB plus puissant, ou luminosité plus basse |

## Compiler depuis VS Code (extension ESP-IDF)

- Version d'ESP-IDF : **v5.0.9** (*ESP-IDF: Select Current ESP-IDF Version*), cible **esp32s3**
  (*ESP-IDF: Set Espressif Device Target*). ⚠️ Changer la cible régénère `sdkconfig` : `sdkconfig.defaults`
  remet les valeurs indispensables.
- Port **COM4** (CH343), méthode de flash **UART**.
- Dans un espace de travail multi-dossiers : *ESP-IDF: Pick a Workspace Folder* → ce projet, avant Build/Flash.
- **Un seul build à la fois** : deux builds simultanés (extension + terminal) sur le même dossier `build/`
  provoquent des erreurs `ninja: failed recompaction: Permission denied` ou des `.obj` introuvables.
- Le moniteur série occupe COM4 : le fermer (`Ctrl+]`) avant un flash lancé ailleurs.

## Organisation du code

| Fichier | Rôle |
|---|---|
| `main/main.c` | initialisation des périphériques, tâche LVGL |
| `main/screen.c` | panneau RGB 480×480 (`full_refresh`), encodeur LVGL (rotation + bouton) |
| `main/board.h` | brochage de la carte |
| `components/mt8901` | lecture de la rotation (compteur d'impulsions PCNT, pilote `driver/pulse_cnt.h`) |
| `components/button` | bouton-poussoir (anti-rebond) |
| `components/ws2812` | pilote des LED WS2812 via le périphérique RMT |
| `components/lvgl` | LVGL 8.3.3 |
| `sdkconfig.defaults` | valeurs indispensables réappliquées si `sdkconfig` est régénéré |

## ⚠️ Configuration indispensable (`sdkconfig.defaults`)

L'écran a besoin de la **PSRAM** pour son frame buffer. Sans elle, le firmware plante au démarrage
(`lcd_panel.rgb: no mem for frame buffer` puis Guru Meditation) : **écran noir**.

`idf.py set-target` (ou *ESP-IDF: Set Espressif Device Target* dans VS Code) régénère `sdkconfig`.
Les valeurs indispensables (PSRAM octale 80 MHz, flash 16 Mo, `partitions_lvgl.csv`, CPU 240 MHz)
sont donc reprises dans `sdkconfig.defaults` et réappliquées automatiquement.
