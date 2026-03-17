<!--
 * @Description: ADS-B Receiver on LILYGO T-Display-P4
 * @Author: John Stockdale / Off by One (fork of LILYGO_L original)
 * @Date: 2025-06-13 15:12:02
 * @LastEditTime: 2026-03-17
 * @License: GPL 3.0
-->
<h1 align="center">ADS-B Receiver – T-Display-P4</h1>

<p align="center">
  A portable 1090 MHz ADS-B receiver built on the LILYGO T-Display-P4, using an RTL-SDR USB dongle for RF reception and the ESP32-P4's dual RISC-V cores for real-time Mode-S decoding.
</p>

## Overview

This project turns a LILYGO T-Display-P4 development board into a standalone ADS-B receiver. An RTL-SDR dongle connected via USB Host receives 1090 MHz transponder signals, which are decoded in real-time on the ESP32-P4. Decoded aircraft are displayed on the built-in touchscreen, logged to SD card with full positional data, and can be viewed live on a map via the companion web app, ADS-B Scope.

### What works today

- **Real-time ADS-B reception** – 15–30 messages/second, 12–30+ simultaneous aircraft tracked
- **~30 nm range** from Oakland, CA with a 7" telescopic antenna on RTL-SDR
- **On-device ADS-B app** – live aircraft table sorted by distance with RTL-SDR status, message rate, GPS position, and nearest aircraft info. Radar-green-on-dark aesthetic. Draggable divider between stats and aircraft list. Rotation toggle cycles through all 4 orientations via hardware PPA acceleration.
- **Status bar indicators** – GPS (fix/searching/offline with satellite count), ADS-B (connected/aircraft count), SD card (logging/mounted/absent), battery level. Color-coded: green = active, yellow = degraded, red = error, gray = offline.
- **CIT diagnostics** – ADS-B test page in the hardware test menu shows RTL-SDR connection, message stats, aircraft count, nearest aircraft, GPS status, and UTC time.
- **GPS-based automatic timezone** – 0.1° resolution worldwide timezone grid (~165KB flash) with DST rule support for 61 regions. Correctly handles half-hour offsets (India, Nepal), southern hemisphere DST (Australia, New Zealand), and US/EU spring-forward/fall-back transitions. Manual override available via serial console.
- **SD card CSV logging** – UTC timestamps, raw Mode-S hex, decoded fields (ICAO, callsign, altitude, speed, heading, vertical rate, position, squawk), receiver GPS metadata (sats, HDOP). PSRAM-buffered writes with periodic flush for clean filesystem state.
- **GPS time sync** – L76K GNSS (5 Hz) sets the system clock with 650ms serial delay compensation, syncs the PCF8563 RTC in local time
- **Serial console** – interactive command mode (Ctrl+C) with file management, status, version query, timezone control; ADS-B output buffered and replayed on return to log mode
- **ADS-B Scope** – WebSerial-based live map viewer with firmware flashing, CSV replay, file browser ([adsb-scope.offx1.com](https://adsb-scope.offx1.com))
- **SD card reliability** – power-cycled on boot via XL9535 GPIO to reset card state machine after hard resets

### Known limitations

- **WiFi disabled** – ESP-Hosted SDIO DMA corrupts internal RAM heap metadata (Espressif Issue #17889); WiFi/NTP unavailable until resolved or AT firmware is deployed on the C6
- **LVGL table display disabled** – `adsb_update_display()` is a no-op to work around USB DMA heap corruption; the ADS-B app uses a separate label-based approach that avoids the `lv_table_set_cell_value` realloc crash
- **Settings app placeholder** – visible on home screen but not yet implemented
- **WebSerial triggers device reboot** – USB-JTAG auto-reset circuit fires on DTR toggle during port open; handled gracefully (scope reconnects, boot log is parsed)

## On-Device UI

### Home Screen

The device boots to a home screen showing local time/date (automatically timezone-adjusted), app icons (CIT, RF, Music, ADS-B), and a dock (Camera, Settings). The status bar displays time, GPS satellite count, ADS-B aircraft count, SD card status, and battery level — all color-coded for at-a-glance status.

### ADS-B App

Tap the ADS-B icon to open a live aircraft display with:
- **Stats panel** – RTL-SDR connection status, message rate, aircraft count, GPS position, nearest aircraft callsign/distance/altitude, UTC time
- **Aircraft table** – all tracked aircraft sorted by distance, showing ICAO, callsign, altitude, speed, heading, and distance in nautical miles. Scrollable.
- **Draggable divider** – resize stats vs. list by dragging the blue bar between panels
- **Rotation toggle** – cycles through 0°/90°/180°/270° using ESP32-P4's PPA hardware rotation engine. Landscape orientation gives maximum table width. Rotation restores automatically when leaving the app.

### CIT (Hardware Test) Menu

Includes an ADS-B test page alongside the stock hardware tests (touch, screen, vibration, speaker, mic, IMU, battery, GPS, ethernet, RTC). The ADS-B test shows receiver status with PASS/FAIL buttons.

## Timezone System

The device automatically detects the local timezone from GPS coordinates using an offline lookup table. The system uses a 0.1° (~11km) resolution grid covering the entire world, RLE-compressed to ~165KB in flash. It includes DST transition rules for all major regions.

### How it works
1. GPS fix provides latitude/longitude
2. Grid lookup maps coordinates to one of 61 timezone regions
3. Each region specifies a base UTC offset and an optional DST rule
4. DST rules encode "Nth weekday of month" transitions (e.g., 2nd Sunday of March)
5. The current UTC date determines whether DST is active
6. Total offset = base + DST delta

### Regenerating timezone data
If timezone rules change (rare), regenerate the data file:
```bash
pip install geopandas shapely requests numpy
python3 generate_tz_data.py
# Outputs: timezone_data.h (~165KB, copy to source directory)
```

### Serial console timezone commands
```
timezone              show current timezone info
timezone auto         use GPS-based automatic timezone (default)
timezone UTC-8        set manual offset (PST)
timezone UTC+5:30     set manual offset (IST)
tz                    alias for timezone
```

## ADS-B Scope

A self-contained HTML file that connects to the receiver over WebSerial and plots aircraft on a live map.

**Live version:** [adsb-scope.offx1.com](https://adsb-scope.offx1.com)

Features:
- Dark radar-scope aesthetic with Leaflet/CartoDB dark tiles
- Aircraft icons with correct heading rotation and position trails
- Range rings at 10, 25, and 50 nm
- GNSS receiver status and SD card logging status display
- Aircraft detail panel on selection (click icon or label, multi-select with Cmd/Ctrl+click)
- Sortable aircraft table with drag-resizable panel
- Color modes: MONO (terminal green), RAINBOW (by ICAO hash), ALT (altitude heatmap), SPD (speed heatmap)
- Label modes: altitude or distance from receiver
- Serial log panel with ADS-B/GNSS/Other filters and pause-to-inspect
- SD card file browser over serial (list, download, replay, delete)
- CSV replay with timeline scrubber and variable speed (0.25×–32×)
- OTA firmware flashing via esptool-js (WebSerial)
- Firmware version checking against `latest.version.json` with update badge
- Auto-reconnect after flash
- Responsive layout with touch-friendly targets for mobile use
- DTR/RTS deasserted on connect to minimize resets

Requirements: Chrome or Edge (WebSerial API). Connect via USB, click "Connect Serial", select the ESP32-P4 port.

## Serial Console

Press **Ctrl+C** over serial (115200 baud) to enter command mode. Type `log` to return to streaming mode. ADS-B and GNSS output is buffered in PSRAM during command mode (up to 500 lines) and replayed when returning to log mode.

```
help              show available commands
log               return to streaming log mode
ls [path]         list directory (default: /sdcard)
cat <file>        print file contents
head <file> [n]   print first n lines (default 20)
tail <file> [n]   print last n lines (default 20)
rm <file>         delete a file
mv <src> <dst>    rename/move a file
cp <src> <dst>    copy a file
df                show SD card free space
status            show receiver status
version           show firmware version
timezone [auto|UTC±N] show or set timezone
mount             mount SD card
unmount           safely unmount SD card
reboot            software reset
```

## CSV Log Format

Logged to `/sdcard/adsb_YYYY-MM-DDTHHMMSSZ.csv` (or `adsb_bootNNNN.csv` before GPS fix):

```
timestamp_utc,raw_msg,icao,callsign,altitude_ft,speed_kt,heading_deg,vrate_fpm,lat,lon,squawk,rx_lat,rx_lon,range_km,bearing_deg,rx_sats,rx_hdop
2026-03-17T06:19:14.739Z,8D0D0A08581DB4BF26B6DFB4B0B1,0D0A08,VOI1773,4875,283,255,2560,37.7492,-122.4221,0000,37.8333,-122.2739,8.7,234,8,1.2
```

Timestamps are ISO-8601 UTC with millisecond resolution. Raw Mode-S hex is the second column for easy replay. Every row includes receiver GPS sats/HDOP for data quality assessment.

## Hardware

### Board: LILYGO T-Display-P4 V1.0

| Component | Detail |
|---|---|
| **SoC** | ESP32-P4, 360 MHz dual RISC-V, 32 MB PSRAM, 16 MB flash |
| **Coprocessor** | ESP32-C6-MINI-1U (WiFi 6 / BLE 5 via SDIO) |
| **Display** | HI8561 4.05" MIPI DSI touchscreen, 540×1168, 326 PPI |
| **GPS** | Quectel L76K (UART, 9600→115200 baud auto-detect, 5 Hz) |
| **RTC** | PCF8563 (I²C, stores local time via GPS timezone lookup) |
| **LoRa** | SX1262 via SPI (HPD16A module) |
| **Audio** | ES8311 DAC + NS4150B amplifier + electret mic |
| **IMU** | ICM20948 9-axis (I²C) |
| **Battery** | BQ27220 gauge + LGS4056H charger |
| **Camera** | OV2710 MIPI-CSI |
| **IO Expander** | XL9535 (I²C) |
| **Storage** | SD card (SDMMC, 4-bit) |

### External: RTL-SDR USB Dongle

Connected via the ESP32-P4's USB 2.0 Host port. The firmware implements a custom USB host driver that initializes the RTL2832U + R820T tuner, tunes to 1090 MHz at 2 MS/s, and reads IQ samples via USB bulk transfers.

## Architecture

```
RTL-SDR (1090 MHz, 2 MS/s IQ)
    │ USB bulk transfer (PSRAM buffers)
    ▼
ESP32-P4 USB Host Driver (class_driver.c)
    │ magnitude → Mode-S demodulator (mode-s.c)
    ▼
Aircraft Table (64 slots, 60s expiry)
    │
    ├──→ On-device ADS-B app (LVGL labels, sorted by distance)
    ├──→ Status bar (aircraft count, RTL-SDR status, GPS, SD card)
    ├──→ Serial output (serial_console.c, buffered during CMD mode)
    ├──→ SD card CSV log (PSRAM buffer, periodic flush)
    └──→ ADS-B Scope (via WebSerial)

L76K GPS (UART, 5 Hz NMEA)
    │
    ├──→ Receiver position → range/bearing calculation
    ├──→ System clock (settimeofday + 650ms serial delay compensation)
    ├──→ Timezone lookup (0.1° grid + DST rules → local time)
    └──→ PCF8563 RTC (local time, every 60s)

Serial Console (serial_console.c)
    │
    ├──→ LOG mode: streaming ADS-B/GNSS output
    ├──→ CMD mode: file management, status, version, timezone
    └──→ Replay buffer: 500 lines in PSRAM, flushed on return to LOG
```

## Source Files

All in `main/examples/lvgl_9_ui/`:

| File | Description | License |
|---|---|---|
| `main.cpp` | Boot, peripheral init, GPS task, LVGL UI, task orchestration | GPL 3.0 (LILYGO) |
| `class_driver.c/h` | USB host, RTL-SDR reader, CPR decode, aircraft table, SD logging | BSD 3-Clause |
| `mode-s.c/h` | Mode S preamble detection, CRC, decode | BSD 2-Clause |
| `serial_console.c/h` | Interactive serial console with PSRAM replay buffer | BSD 3-Clause |
| `lvgl_ui.cpp/h` | LVGL UI framework, ADS-B app, status bar, CIT tests | GPL 3.0 (LILYGO) |
| `tz_lookup.c/h` | GPS-based timezone lookup with DST support | BSD 3-Clause |
| `timezone_data.h` | Generated 0.1° timezone grid (61 regions, 12 DST rules) | Public domain |
| `esp_libusb.c/h` | USB-to-librtlsdr shim | GPL 2.0 |
| `librtlsdr.c` | RTL-SDR driver (R820T2 only) | GPL 2.0 |
| `adsb_scope.html` | Web companion app (v1.0.3) | BSD 3-Clause |
| `generate_tz_data.py` | Timezone data generator (runs on host) | BSD 3-Clause |

## Memory Budget

| Stage | Internal RAM Free |
|---|---|
| Boot | 267 KB |
| After SD mount | 222 KB |
| After USB host install | 222 KB |
| Before LVGL | 213 KB |
| After UI + all tasks | ~116 KB |
| PSRAM free | ~20 MB |

## Quick Start – Pre-built Binary

If you just want to flash and go, a pre-built binary is available. No build environment needed.

### Option 1: Flash from browser (easiest)

1. Open [adsb-scope.offx1.com](https://adsb-scope.offx1.com) in Chrome or Edge
2. Click **Flash** → **Flash Latest**
3. Select the ESP32-P4 serial port when prompted
4. Wait for flash to complete (~60s), device auto-reboots

If the device doesn't enter bootloader mode automatically, hold **BOOT** + tap **RESET** before clicking Flash Latest.

### Option 2: Flash with esptool.py

- [esptool.py](https://github.com/espressif/esptool) (`pip install esptool`)
- USB cable to the T-Display-P4
- Hold **BOOT** + tap **RESET** to enter download mode

### Flash

```bash
esptool.py --chip esp32p4 --port /dev/tty.usbmodem* \
    write_flash 0x0 release.bin
```

On Windows, replace `/dev/tty.usbmodem*` with the appropriate COM port (e.g., `COM3`).

After flashing, press RESET. The device will boot, initialize all peripherals, and begin scanning 1090 MHz as soon as an RTL-SDR dongle is connected to the USB Host port. Connect to [adsb-scope.offx1.com](https://adsb-scope.offx1.com) via Chrome WebSerial to see aircraft on a live map.

### Hardware needed

- LILYGO T-Display-P4 V1.0
- RTL-SDR USB dongle (RTL2832U + R820T/R820T2)
- Antenna – the included telescopic whip works, ~30 nm range from a window

## Building from Source

### Prerequisites

- ESP-IDF v5.4.1
- Visual Studio Code + ESP-IDF extension (recommended)

### Build & Flash

```bash
git clone --recursive https://github.com/Xinyuan-LilyGO/T-Display-P4.git
cd T-Display-P4

# Select the lvgl_9_ui example in SDK Configuration Editor
idf.py set-target esp32p4
idf.py build
idf.py flash
```

If auto-reset doesn't work (USB-JTAG bridge disabled in firmware), hold BOOT + tap RESET, then flash.

### Merged firmware binary (for OTA / distribution)

```bash
cd build && esptool.py --chip esp32p4 merge_bin -o ../release.bin \
  --flash_mode dio --flash_freq 80m --flash_size 16MB \
  0x2000 bootloader/bootloader.bin \
  0x8000 partition_table/partition-table.bin \
  0x10000 t-display-p4.bin
```

### Configuration Notes

- `CONFIG_HEAP_POISONING_LIGHT=y` – enabled for heap corruption debugging
- ESP-Hosted and esp_wifi_remote are commented out in `idf_component.yml`
- `CPP_BUS_DRIVER_LOG_LEVEL_DEBUG` is commented out in `config.h` to suppress raw NMEA dumps

## Pending Work

1. **On-device ADS-B radar scope** – canvas-drawn aircraft positions on a map view (currently text table only)
2. **Settings app** – timezone manual selection UI, display brightness, SD card format, WiFi configuration
3. **Restore WiFi** – either fix ESP-Hosted SDIO DMA or flash C6 with AT firmware
4. **Fix USB DMA heap corruption** – root cause of LVGL table crash; USB bulk transfers corrupt internal RAM heap metadata; current workaround avoids lv_table realloc
5. **TDOA geolocation network** – distributed GNSS-disciplined SDR nodes for passive aircraft tracking (design phase)
6. ~~Fix screen~~ – **fixed**: `esp_lcd_panel_reset()` was wiping DSI lane config after `Screen_Init()`; removed reset, reordered `App_Video_Init()` before `Screen_Init()` to match stock LILYGO init order
7. ~~Fix timezone~~ – **fixed**: replaced hardcoded UTC+8 with GPS-based timezone lookup using 0.1° worldwide grid with DST rules
8. ~~Investigate WebSerial reset~~ – DTR race handled gracefully; scope deasserts DTR/RTS and parses reboot log
9. ~~SD card corruption on reboot~~ – fixed: power-cycle SD card via XL9535 SD_EN early in boot to reset card state machine
10. ~~Heading display~~ – fixed: ground speed heading (DF17 TC19 subtypes 1/2) now correctly applied
11. ~~ADS-B decoding~~ – fixed: removed double-decode in on_msg that zeroed all messages via memset

## Acknowledgments

This project builds on the work of several open source authors:

- **[kvhnuke](https://github.com/kvhnuke/esp32-rtl-sdr)** — Original proof of concept for RTL-SDR on ESP32 via USB Host. The ESP32 USB-to-librtlsdr shim layer (`esp_libusb.c/h`) and the adapted `librtlsdr.c` driver originated from this project. Thank you for proving it could be done.

- **[Salvatore Sanfilippo (antirez)](https://github.com/antirez/dump1090)** — Author of dump1090, the Mode S decoder for RTL-SDR devices. The `mode-s.c` decoder is derived from dump1090's signal processing, preamble detection, CRC validation, and message decoding. Released under the BSD 3-Clause License.

- **[Thomas Watson](https://github.com/watson/libmodes)** — Author of libmodes, which refactored the dump1090 decoder into a clean reusable C library with the `mode_s_init` / `mode_s_detect` / `mode_s_decode` API that this project uses directly.

- **[LILYGO](https://github.com/Xinyuan-LilyGO/T-Display-P4)** — T-Display-P4 hardware design and the base ESP-IDF project with LVGL UI framework, peripheral drivers, and board support package.

- **[osmocom / Steve Markgraf](https://github.com/steve-m/librtlsdr)** — Original librtlsdr and the R820T/R828D tuner drivers.

- **[timezone-boundary-builder](https://github.com/evansiroky/timezone-boundary-builder)** — Open timezone boundary GeoJSON data used to generate the embedded timezone lookup grid.

## License

This project contains code under multiple licenses:

- **ADS-B Scope, serial console, SD logging, timezone system, and all Off by One contributions: BSD 3-Clause**
- Mode S decoder: **BSD 3-Clause** (antirez/dump1090, watson/libmodes)
- RTL-SDR driver and tuner code: **GPL v2** (osmocom/librtlsdr)
- LILYGO board support and UI framework: **GPL v3** (LILYGO)

See individual file headers for details.

---

## Original LilyGo Documentation

<details>
<summary>Click to expand original T-Display-P4 hardware documentation</summary>

## VersionIteration:
| Version                               | Update date                       |Update description|
| :-------------------------------: | :-------------------------------: |:--------------: |
| T-Display-P4_V1.0                      | 2025-06-13                    |   Original version      |
| T-Display-P4-Keyboard_V1.0                      | 2025-09-12                    |   Original version      |

## PurchaseLink

| Product                     | SOC           |  FLASH  |  PSRAM   | Link                   |
| :------------------------: | :-----------: |:-------: | :---------: | :------------------: |
| T-Display-P4_V1.0   | NULL |   NULL   | NULL |  [NULL]()   |

## Module

### T-Display-P4 Section
### 1. Core Processor  

* Chip: ESP32-P4  
* FLASH: 16M  
* Related Documents:  
    >[Espressif](https://www.espressif.com/en/support/documents/technical-documents)  

### 2. Auxiliary Processor

* Module: ESP32-C6-MINI-1U
* Chip: ESP32-C6-FH4
* PSRAM: -
* FLASH: 4M 
* Communication Protocol: SDIO
* Other: For more information, please visit [Espressif ESP32-C6-MINI-1U datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-c6-mini-1_mini-1u_datasheet_en.pdf)

### 3. Display & Touch  

> #### Model: H0405S002T002-V0  
> * Display Size (Diagonal): 4.05 inch  
> * LCD Type: α-Si TFT  
> * Resolution: 540(H) × 1168(V) px  
> * Active Area: 41.9904(H) × 91.1040(V) mm  
> * Module Dimensions: 44(H) × 95.5(V) × 1.46(T) mm  
> * Display Colors: 16.7M  
> * Display Interface: MIPI  
> * Touch Interface: IIC
> * Display & Touch Driver IC: HI8561  
> * Maximum touch points: 10-point touch
> * Luminance on surface: 550 cd/m²
> * View Direction: All
> * Contrast ratio: 1200:1
> * Color gamut: 70%
> * PPI: 326
> * Window effect: No all-black  
> * Cover plate surface effect: No AF/AG
> * Operating Temperature: -20～70  ºC
> * Storage Temperature: -30～80 ºC
> * Related Documents:  
>    >[HI8561](./information/HI8561_Preliminary%20_DS_V0.00_20230511.pdf)  

### 4. Speaker & Microphone  

* DAC Chip: ES8311  
* Amplifier Chip: NS4150B  
* Microphone: Electret Condenser Mic  
* Communication Protocol: IIS
* Related Documents:  
    >[ES8311](./information/ES8311.pdf)  
    >[NS4150B](./information/NS4150B.pdf)

### 5. LoRa  

* Module: HPD16A  
* Chip: SX1262, SKY13453-385LF
* Communication Protocol: Standard SPI  
* Other notes: Use a dedicated RF analog switch to switch the antenna
* Related Documents:  
    >[SX1261-2](./information/DS_SX1261-2_V2_1.pdf)  
* Dependent Libraries:  
    >[cpp_bus_driver](https://github.com/Llgok/cpp_bus_driver)  

### 6. GPS  

* Module: L76K  
* Communication Protocol: Uart
* Related Documents:  
    >[L76K](./information/L76KB-A58.pdf)  
* Dependent Libraries:  
    >[cpp_bus_driver](https://github.com/Llgok/cpp_bus_driver)  

### 7. RTC  

* Chip: PCF8563  
* Communication Protocol: IIC
* Related Documents:  
    >[PCF8563](./information/PCF8563.pdf)  
* Dependent Libraries:  
    >[cpp_bus_driver](https://github.com/Llgok/cpp_bus_driver)  

### 8. Battery Gauge  

* Chip: BQ27220  
* Communication Protocol: IIC
* Related Documents:  
    >[BQ27220](./information/bq27220_en.pdf)  
* Dependent Libraries:  
    >[cpp_bus_driver](https://github.com/Llgok/cpp_bus_driver)  

### 9. Camera  

* Model: OV2710  
* Interface: MIPI  
* Related Documents:  
    >[OV2710](./information/OV2710_CSP3_DS_2.0_KING%20HORN%20ENTERPRISES%20Ltd..pdf)  

### 10. IMU

* Chip: ICM20948
* Communication Protocol: IIC
* Related Documents:  
    >[ICM20948](./information/ICM20948.pdf)

### 11. IO Expansion

* Chip: XL9535
* Communication Protocol: IIC
* Related Materials:
    > [XL9535](./information/XL95x5.pdf)

</details>

## SoftwareDeployment

### Examples Support

#### T-Display-P4 Examples
| example | `[vscode][esp-idf-v5.4.0]` | description | picture |
| ------  | ------ | ------ | ------ | 
| [afe](./main/examples/afe) |  <p align="center">![alt text][supported] | | |
| [aw86224](./main/examples/aw86224) |  <p align="center">![alt text][supported] | | |
| [bq27220](./main/examples/bq27220) |  <p align="center">![alt text][supported] | | |
| [deep_sleep](./main/examples/deep_sleep) |  <p align="center">![alt text][supported] | | |
| [es8311](./main/examples/es8311) |  <p align="center">![alt text][supported] | | |
| [es8311_sd_wav](./main/examples/es8311_sd_wav) |  <p align="center">![alt text][supported] | | |
| [esp_hosted_mcu_sdio_wifi](./main/examples/esp_hosted_mcu_sdio_wifi) |  <p align="center">![alt text][supported] | | |
| [esp32c6_at_host_sdio_uart](./main/examples/esp32c6_at_host_sdio_uart) |  <p align="center">![alt text][supported] | | |
| [esp32c6_at_host_sdio_wifi](./main/examples/esp32c6_at_host_sdio_wifi) |  <p align="center">![alt text][supported] | | |
| [icm20948](./main/examples/icm20948) |  <p align="center">![alt text][supported] | | |
| [iic_scan](./main/examples/iic_scan) |  <p align="center">![alt text][supported] | | |
| [l76k](./main/examples/l76k) |  <p align="center">![alt text][supported] | | |
| [lvgl_9_ui](./main/examples/lvgl_9_ui) |  <p align="center">![alt text][supported] |factory example / ADS-B receiver | |
| [pcf8563](./main/examples/pcf8563) |  <p align="center">![alt text][supported] | | |
| [radiolib_sx1262_send_receive](./main/examples/radiolib_sx1262_send_receive) |  <p align="center">![alt text][supported] | | |
| [screen_camera](./main/examples/screen_camera) |  <p align="center">![alt text][supported] | | |
| [screen_lvgl](./main/examples/screen_lvgl) |  <p align="center">![alt text][supported] | | |
| [screen_lvgl_touch_draw](./main/examples/screen_lvgl_touch_draw) |  <p align="center">![alt text][supported] | | |
| [sgm38121](./main/examples/sgm38121) |  <p align="center">![alt text][supported] | | |
| [sx1262_gfsk_send_receive](./main/examples/sx1262_gfsk_send_receive) |  <p align="center">![alt text][supported] | | |
| [sx1262_lora_send_receive](./main/examples/sx1262_lora_send_receive) |  <p align="center">![alt text][supported] | | |
| [sx1262_tx_continuous_wave](./main/examples/sx1262_tx_continuous_wave) |  <p align="center">![alt text][supported] | | |
| [tusb_serial_device](./main/examples/tusb_serial_device) |  <p align="center">![alt text][supported] | | |
| [xl9535](./main/examples/Vibration_Motor) |  <p align="center">![alt text][supported] | | |
| [xiaozhi](https://github.com/78/xiaozhi-esp32) |  <p align="center">![alt text][supported] | | |

[supported]: https://img.shields.io/badge/-supported-green "example"

| firmware | description | picture |
| ------  | ------  | ------ |
| [t_display_p4_lvgl_9_ui](./firmware/[T-Display-P4][lvgl_9_ui]) | factory program |  |
| [esp32c6_at](./firmware/[T-Display-P4][esp32c6_at_slave]) | esp32c6-at factory program |  |
| [esp32c6_slave_esp_hosted_mcu_network_adapter](./firmware/[T-Display-P4][esp32c6_slave_esp_hosted_mcu_network_adapter]) |  |  |

## PinOverview

For pin definitions, please refer to the configuration file: 
<br />

[t_display_p4_config.h](./components/private_library/t_display_p4_config.h)  

## FAQ

* Q. After reading the above tutorials, I still don't know how to build a programming environment. What should I do?
* A. If you still don't understand how to build an environment after reading the above tutorials, you can refer to the [LilyGo-Document](https://github.com/Xinyuan-LilyGO/LilyGo-Document) document instructions to build it.

<br />

* Q. Why is my board continuously failing to download the program?
* A. Please hold down the "BOOT" button and try downloading the program again.

<br />

* Q. Why does the device reboot every time I connect a serial monitor?
* A. The ESP32-P4's USB-JTAG peripheral has an auto-reset circuit that triggers when DTR/RTS are toggled during port open. The firmware attempts to disable this via `usb_serial_jtag_ll_phy_set_jtag_bridge(false)`. If the device still resets, use tools that suppress DTR/RTS (e.g., `pyserial` with `dtr=False, rts=False`), or use ADS-B Scope which deasserts control lines immediately after connecting.

<br />

* Q. Why is WiFi not working?
* A. ESP-Hosted SDIO DMA on the ESP32-P4 corrupts internal RAM heap metadata. This is tracked in [Espressif Issue #17889](https://github.com/espressif/esp-hosted/issues/17889). WiFi components are commented out in `idf_component.yml`. The planned fix is to flash the ESP32-C6 with AT firmware for synchronous WiFi.

