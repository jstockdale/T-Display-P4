<!--
 * @Description: ADS-B Receiver on LILYGO T-Display-P4
 * @Author: John Stockdale / Off by One (fork of LILYGO_L original)
 * @Date: 2025-06-13 15:12:02
 * @LastEditTime: 2026-03-16
 * @License: GPL 3.0
-->
<h1 align="center">ADS-B Receiver — T-Display-P4</h1>

<p align="center">
  A portable 1090 MHz ADS-B receiver built on the LILYGO T-Display-P4, using an RTL-SDR USB dongle for RF reception and the ESP32-P4's dual RISC-V cores for real-time Mode-S decoding.
</p>

## Overview

This project turns a LILYGO T-Display-P4 development board into a standalone ADS-B receiver. An RTL-SDR dongle connected via USB Host receives 1090 MHz transponder signals, which are decoded in real-time on the ESP32-P4. Decoded aircraft are logged to SD card with full positional data and can be viewed live on a map via the companion web app, ADS-B Scope.

### What works today

- **Real-time ADS-B reception** — 15–30 messages/second, 12–30+ simultaneous aircraft tracked
- **~30 nm range** from Oakland, CA with a 7" telescopic antenna on RTL-SDR
- **SD card CSV logging** — UTC timestamps, raw Mode-S hex, decoded fields (ICAO, callsign, altitude, speed, heading, vertical rate, position, squawk), receiver GPS metadata (sats, HDOP)
- **GPS time sync** — L76K GNSS sets the system clock with 650ms serial delay compensation, syncs the PCF8563 RTC
- **Serial output** — clean formatted messages with N/S E/W position indicators, raw hex for each message
- **ADS-B Scope** — WebSerial-based live map viewer ([adsb-scope.offx1.com](https://adsb-scope.offx1.com))

### Known limitations

- **Screen is black** — LVGL initializes successfully but display output is not rendering; under investigation
- **WiFi disabled** — ESP-Hosted SDIO DMA corrupts internal RAM heap metadata (Espressif Issue #17889); WiFi/NTP unavailable until resolved or AT firmware is deployed on the C6
- **LVGL display updates disabled** — `adsb_update_display()` is a no-op to work around USB DMA heap corruption; aircraft data is only on serial + SD card + ADS-B Scope
- **WebSerial triggers device reboot** — USB-JTAG auto-reset circuit fires on DTR toggle during port open; investigation in progress

## ADS-B Scope

A self-contained HTML file that connects to the receiver over WebSerial and plots aircraft on a live map.

**Live version:** [adsb-scope.offx1.com](https://adsb-scope.offx1.com)

Features:
- Dark radar-scope aesthetic with Leaflet/CartoDB dark tiles
- Aircraft icons with correct heading rotation and position trails
- Range rings at 10, 25, and 50 nm
- GNSS receiver status display
- Aircraft detail panel on selection (click icon or label)
- Sortable aircraft table with drag-resizable panel
- Color modes: MONO (terminal green), RAINBOW (by ICAO hash), ALT (altitude heatmap), SPD (speed heatmap)
- Serial log panel with ADS-B/GNSS/Other filters and pause-to-inspect
- DTR/RTS deasserted on connect to minimize resets

Requirements: Chrome or Edge (WebSerial API). Connect via USB, click "Connect Serial", select the ESP32-P4 port.

## dump1090 Bridge

`serial_to_dump1090.py` reads serial output, extracts raw Mode-S hex from `[brackets]`, and feeds dump1090 in AVR format (`*HEX;\n`) over TCP port 30001.

```bash
# Terminal 1
dump1090-fa --net-only --net-ri-port 30001

# Terminal 2
pip install pyserial
python3 serial_to_dump1090.py /dev/tty.usbmodem* -v
```

Then open `http://localhost:8080` for dump1090's built-in map UI. DTR/RTS are disabled on connect to prevent device reboot.

## CSV Log Format

Logged to `/sdcard/adsb_YYYY-MM-DDTHHMMSSZ.csv` (or `adsb_bootNNNN.csv` before GPS fix):

```
timestamp_utc,raw_msg,icao,callsign,altitude_ft,speed_kt,heading_deg,vrate_fpm,lat,lon,squawk,rx_lat,rx_lon,range_km,bearing_deg,rx_sats,rx_hdop
2026-03-16T04:13:35.123Z,8DA105E8582594BADAC53333EE95,A105E8,SKW5567,6425,285,109,-128,37.72680,-122.44730,0000,37.83328,-122.27379,11.0,233,5,2.7
```

Timestamps are ISO-8601 UTC with millisecond resolution. Raw Mode-S hex is the second column for easy replay. Every row includes receiver GPS sats/HDOP for data quality assessment.

## Hardware

### Board: LILYGO T-Display-P4 V1.0

| Component | Detail |
|---|---|
| **SoC** | ESP32-P4, 360 MHz dual RISC-V, 32 MB PSRAM, 16 MB flash |
| **Coprocessor** | ESP32-C6-MINI-1U (WiFi 6 / BLE 5 via SDIO) |
| **Display** | HI8561 4.05" MIPI touchscreen, 540×1168, 326 PPI |
| **GPS** | Quectel L76K (UART, 9600→115200 baud auto-detect) |
| **RTC** | PCF8563 (I²C, stores UTC+8 for LilyGo UI convention) |
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
    │ USB bulk transfer
    ▼
ESP32-P4 USB Host Driver (class_driver.c)
    │ magnitude → Mode-S demodulator (mode-s.c)
    ▼
Aircraft Table (64 slots, 60s expiry)
    │
    ├──→ Serial output (printf, formatted + raw hex)
    ├──→ SD card CSV log (fsync every write)
    └──→ LVGL table (disabled — heap corruption workaround)

L76K GPS (UART, 1 Hz NMEA)
    │
    ├──→ Receiver position → range/bearing calculation
    ├──→ System clock (settimeofday + 650ms serial delay compensation)
    └──→ PCF8563 RTC (UTC+8, every 60s)
```

## Memory Budget

| Stage | Internal RAM Free |
|---|---|
| Boot | 271 KB |
| After SD mount | 226 KB |
| After USB host install | 226 KB |
| Before LVGL | 217 KB |
| After UI + all tasks | ~131 KB |
| PSRAM free | ~20 MB |

## Building

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

### Configuration Notes

- `CONFIG_HEAP_POISONING_LIGHT=y` — enabled for heap corruption debugging
- ESP-Hosted and esp_wifi_remote are commented out in `idf_component.yml`
- `CPP_BUS_DRIVER_LOG_LEVEL_DEBUG` is commented out in `config.h` to suppress raw NMEA dumps

## Pending Work

1. **Fix USB DMA heap corruption** — root cause of LVGL crash; USB bulk transfers corrupt internal RAM heap metadata
2. **Re-enable LVGL aircraft display** — blocked by #1
3. **Fix screen** — display hardware initializes but screen is black; may be wallpaper/draw buffer issue
4. **Restore WiFi** — either fix ESP-Hosted SDIO DMA or flash C6 with AT firmware
5. **Investigate WebSerial reset** — DTR→reset path may be in ROM code, not disableable from app_main

## Credits

- **Hardware & base firmware:** [LILYGO](https://github.com/Xinyuan-LilyGO/T-Display-P4)
- **Mode-S decoder:** Based on [dump1090](https://github.com/antirez/dump1090) by Salvatore Sanfilippo
- **ADS-B Scope:** [adsb-scope.offx1.com](https://adsb-scope.offx1.com) — Off by One
- **RTL-SDR driver:** Custom ESP32-P4 USB host implementation based on librtlsdr

## License

GPL 3.0 (inherited from LILYGO base project)

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

<br />

* Q. Why is the screen black?
* A. The MIPI DSI panel and LVGL both initialize successfully, but display output is not rendering. This may be related to wallpaper loading, draw buffer configuration, or a side effect of the USB DMA workaround. Testing with an unmodified LilyGo build is the next diagnostic step.
