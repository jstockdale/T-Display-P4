/**
 * meshtastic_task.cpp — FreeRTOS Meshtastic RX/TX for T-Display-P4.
 *
 * Uses the Cpp_Bus_Driver SX1262 API (not RadioLib directly) to interface
 * with the HPD16A LoRa module. Meshtastic protocol handling via meshtastic-lite.
 *
 * Hardware notes:
 *   - SPI1 (GPIO 2/3/4) is dedicated to SX1262 — no SPI mutex needed
 *   - DIO1 routed through XL9535 I2C expander (XL9535_SX1262_DIO1) — polled
 *   - DIO2 used as RF switch internally by HPD16A (set via chip config)
 *   - SKY13453 RF switch: VCTL HIGH = TX path, LOW = RX path
 *
 * Part of ADS-B Receiver / Meshy — T-Display-P4.
 */

#include "meshtastic_task.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_heap_caps.h"
#include "device_settings.h"

// GPS fix epoch — set in GPS task on first quality fix.
extern volatile time_t g_gps_fix_epoch;
extern "C" bool sd_msc_is_active_fn(void);
extern "C" bool sd_is_mounted(void);
extern "C" void sd_safe_shutdown(void);
extern "C" bool sd_remount(void);
#include "meshy_channels.h"
#include "serial_console.h"
#include "sd_config.h"

// Cpp_Bus_Driver includes
#include "cpp_bus_driver_library.h"
#include "t_display_p4_config.h"

// Meshtastic protocol library
#define MESH_CRYPTO_USE_MBEDTLS 1
#include "meshtastic.h"

static const char *TAG = "MESHY";

// Cyan info log — matches ESP_LOG format but routes through serial_console_print
#define MESHY_LOGI(fmt, ...) do { \
    char _ts[32]; log_format_timestamp(_ts, sizeof(_ts)); \
    serial_console_print("\033[0;36m%s%s: " fmt "\033[0m\n", _ts, TAG, ##__VA_ARGS__); \
} while(0)

// ─── Hardware References (set by meshy_init_hw) ─────────────────────────────

static Cpp_Bus_Driver::Sx126x  *s_sx1262  = nullptr;
static Cpp_Bus_Driver::Xl95x5  *s_xl9535  = nullptr;

// ─── Meshtastic Session ─────────────────────────────────────────────────────

static MeshSession s_session;
static bool s_running = false;

// ─── Task Handles ───────────────────────────────────────────────────────────

static TaskHandle_t s_rx_task_hdl = nullptr;
static TaskHandle_t s_tx_task_hdl = nullptr;

// ─── Queues ─────────────────────────────────────────────────────────────────

static QueueHandle_t s_rx_queue  = nullptr;  // meshy_msg_t → UI
static QueueHandle_t s_tx_queue  = nullptr;  // tx_request_t → TX task

// SPI bus mutex — shared between RX and TX tasks for SX1262 access
static SemaphoreHandle_t s_spi_mutex = nullptr;

typedef struct {
    uint32_t to;
    uint8_t  channel_idx;
    char     text[MESHY_MAX_TEXT];
} tx_request_t;

// ─── Node Table ─────────────────────────────────────────────────────────────

static meshy_node_t *s_nodes = nullptr;  // allocated in PSRAM by meshy_start()
static int s_node_count = 0;
static SemaphoreHandle_t s_node_mutex = nullptr;

// ─── Stats ──────────────────────────────────────────────────────────────────

static meshy_stats_t s_stats;

// ─── Message History (ring buffer for UI display, PSRAM-backed) ─────────────

#define MESHY_MSG_HISTORY 1000
static meshy_msg_t *s_msg_history = nullptr;  // allocated in PSRAM by meshy_start()
static int s_msg_write_idx = 0;
static int s_msg_count = 0;

// ─── SD Card Logging ────────────────────────────────────────────────────────
// Follows the same pattern as ADS-B CSV logging: PSRAM buffer, periodic flush,
// file stays closed between flushes for FAT32 safety on hard reset.

#define MESHY_SD_BUFSIZE     (32 * 1024)
#define MESHY_SD_SYNC_US     5000000LL    // flush every 5s
#define MESHY_SD_SYNC_COUNT  50           // or every 50 messages

static char   meshy_sd_filename[64] = {0};
static bool   meshy_sd_initialized = false;
static char  *meshy_sd_buf = nullptr;
static int    meshy_sd_buf_pos = 0;
static int64_t meshy_sd_last_sync = 0;
static int    meshy_sd_writes = 0;

static void meshy_sd_pick_filename(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    gmtime_r(&now, &timeinfo);
    if (now < 1704067200LL) {
        snprintf(meshy_sd_filename, sizeof(meshy_sd_filename),
            "/sdcard/mesh_boot%s.csv", sd_config_boot_id());
    } else {
        strftime(meshy_sd_filename, sizeof(meshy_sd_filename),
            "/sdcard/mesh_%Y-%m-%dT%H%M%SZ.csv", &timeinfo);
    }
}

static void meshy_sd_create(void) {
    meshy_sd_pick_filename();
    FILE *f = fopen(meshy_sd_filename, "w");
    if (!f) {
        ESP_LOGW(TAG, "Failed to open meshy SD log: %s", meshy_sd_filename);
        return;
    }
    fprintf(f, "timestamp_utc,from_hex,to_hex,packet_id,channel,rssi,snr,"
               "hop_limit,hop_start,portnum,port_name,"
               "text,lat,lon,altitude_m,"
               "long_name,short_name,"
               "battery_pct,voltage,ch_util_pct,air_util_pct,"
               "rx_count,is_pki\n");
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    meshy_sd_initialized = true;
    meshy_sd_last_sync = esp_timer_get_time();
    meshy_sd_writes = 0;
    meshy_sd_buf_pos = 0;
    MESHY_LOGI("Meshy SD log: %s", meshy_sd_filename);
}

static void meshy_sd_flush(void) {
    if (!meshy_sd_initialized || meshy_sd_buf_pos == 0) return;

    static int s_meshy_flush_fail_count = 0;

    FILE *f = fopen(meshy_sd_filename, "a");
    if (!f) {
        s_meshy_flush_fail_count++;
        ESP_LOGW(TAG, "Meshy SD flush: cannot open %s (%d consecutive)", meshy_sd_filename, s_meshy_flush_fail_count);
        if (s_meshy_flush_fail_count >= 5) {
            // SD card likely removed — mark log uninitialized and unmount.
            // Set uninitialized FIRST to prevent recursion:
            // sd_safe_shutdown → meshy_sd_close → meshy_sd_flush → returns immediately
            ESP_LOGW(TAG, "SD card unresponsive — closing meshy log and unmounting");
            meshy_sd_initialized = false;
            meshy_sd_filename[0] = '\0';
            meshy_sd_buf_pos = 0;
            s_meshy_flush_fail_count = 0;
            if (sd_is_mounted()) {
                sd_safe_shutdown();  // unmounts card, sets sd_card_handle = NULL
            }
        }
        return;
    }
    s_meshy_flush_fail_count = 0;  // reset on success
    size_t written = fwrite(meshy_sd_buf, 1, meshy_sd_buf_pos, f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    if ((int)written == meshy_sd_buf_pos) {
        meshy_sd_buf_pos = 0;
    } else {
        int remain = meshy_sd_buf_pos - (int)written;
        memmove(meshy_sd_buf, meshy_sd_buf + written, remain);
        meshy_sd_buf_pos = remain;
    }
    meshy_sd_last_sync = esp_timer_get_time();
    meshy_sd_writes = 0;
}

// Close the meshy log permanently (for MSC mode / shutdown).
extern "C" void meshy_sd_close(void) {
    if (!meshy_sd_initialized) return;  // already closed or never opened
    meshy_sd_flush();
    meshy_sd_initialized = false;
    MESHY_LOGI("Meshy SD log closed: %s", meshy_sd_filename);
    meshy_sd_filename[0] = '\0';
}

// Close current log and create a fresh one with a new timestamp.
extern "C" void meshy_sd_create_new(void) {
    if (meshy_sd_initialized) meshy_sd_close();
    meshy_sd_create();
}

// Escape commas and quotes in text for CSV
static int meshy_csv_escape(char *dst, int dst_size, const char *src) {
    bool needs_quote = false;
    for (const char *p = src; *p; p++) {
        if (*p == ',' || *p == '"' || *p == '\n' || *p == '\r') { needs_quote = true; break; }
    }
    int pos = 0;
    if (needs_quote && pos < dst_size) dst[pos++] = '"';
    for (const char *p = src; *p && pos < dst_size - 2; p++) {
        if (*p == '"' && pos < dst_size - 3) { dst[pos++] = '"'; dst[pos++] = '"'; }
        else if (*p == '\n' || *p == '\r') { /* skip newlines */ }
        else dst[pos++] = *p;
    }
    if (needs_quote && pos < dst_size) dst[pos++] = '"';
    dst[pos] = '\0';
    return pos;
}

static void meshy_sd_log_msg(const meshy_msg_t *msg) {
    if (!g_settings.meshy_sd_logging) return;

    // Don't write to SD while USB MSC owns the card
    if (sd_msc_is_active_fn()) return;

    // Lazy init — allocate buffer and create file on first message
    if (!meshy_sd_buf) {
        meshy_sd_buf = (char *)heap_caps_malloc(MESHY_SD_BUFSIZE, MALLOC_CAP_SPIRAM);
        if (!meshy_sd_buf) {
            ESP_LOGW(TAG, "Meshy SD: failed to alloc %d bytes PSRAM", MESHY_SD_BUFSIZE);
            return;
        }
        meshy_sd_buf_pos = 0;
    }
    if (!meshy_sd_initialized) {
        static int64_t last_meshy_mount_retry = 0;
        int64_t now_us = esp_timer_get_time();
        meshy_sd_create();
        if (!meshy_sd_initialized && (now_us - last_meshy_mount_retry > 30000000LL)) {
            last_meshy_mount_retry = now_us;
            if (sd_remount()) {
                meshy_sd_create();
            }
        }
        if (!meshy_sd_initialized) return;
    }

    // Check if GPS fix arrived and we should rename the file
    if (g_gps_fix_epoch > 0 && strstr(meshy_sd_filename, "_boot")) {
        meshy_sd_flush();
        char old_name[64];
        strncpy(old_name, meshy_sd_filename, sizeof(old_name));
        // Use GPS fix epoch for filename — matches ADS-B log timestamp
        struct tm timeinfo;
        time_t fix_time = g_gps_fix_epoch;
        gmtime_r(&fix_time, &timeinfo);
        strftime(meshy_sd_filename, sizeof(meshy_sd_filename),
            "/sdcard/mesh_%Y-%m-%dT%H%M%SZ.csv", &timeinfo);
        rename(old_name, meshy_sd_filename);
        MESHY_LOGI("Meshy SD log renamed: %s → %s", old_name, meshy_sd_filename);
    }

    // Format timestamp (static to keep off 4KB stack)
    static char ts[32];
    struct tm timeinfo;
    time_t now;
    time(&now);
    gmtime_r(&now, &timeinfo);
    if (now >= 1704067200LL) {
        strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    } else {
        snprintf(ts, sizeof(ts), "boot+%lums", (unsigned long)msg->rx_time_ms);
    }

    // Port name
    const char *port = "OTHER";
    switch (msg->portnum) {
        case PORT_TEXT_MESSAGE: port = "TEXT"; break;
        case PORT_POSITION:    port = "POS"; break;
        case PORT_NODEINFO:    port = "NODE"; break;
        case PORT_TELEMETRY:   port = "TELEM"; break;
    }

    // Escape text content (static — only called from single RX task)
    static char escaped_text[MESHY_MAX_TEXT + 4];
    escaped_text[0] = '\0';
    if (msg->text_len > 0) {
        meshy_csv_escape(escaped_text, sizeof(escaped_text), msg->text);
    }

    static char escaped_long[44];
    escaped_long[0] = '\0';
    if (msg->has_nodeinfo && msg->long_name[0]) {
        meshy_csv_escape(escaped_long, sizeof(escaped_long), msg->long_name);
    }

    // Format CSV row (static — single-threaded access from RX task)
    static char row[512];
    int n = snprintf(row, sizeof(row),
        "%s,%08lx,%08lx,%lu,%d,%.1f,%.1f,%d,%d,%d,%s,"
        "%s,"                                           // text
        "%.6f,%.6f,%ld,"                                // lat,lon,alt
        "%s,%s,"                                        // long_name,short_name
        "%lu,%.2f,%.1f,%.1f,"                           // battery,voltage,ch_util,air_util
        "%d,%d\n",
        ts,
        (unsigned long)msg->from, (unsigned long)msg->to,
        (unsigned long)msg->id, msg->channel_idx,
        msg->rssi, msg->snr,
        msg->hop_limit, msg->hop_start,
        msg->portnum, port,
        escaped_text,
        msg->has_position ? msg->lat : 0.0,
        msg->has_position ? msg->lon : 0.0,
        msg->has_position ? (long)msg->altitude : 0L,
        escaped_long,
        msg->has_nodeinfo ? msg->short_name : "",
        msg->has_telemetry ? (unsigned long)msg->battery_level : 0UL,
        msg->has_telemetry ? msg->voltage : 0.0f,
        msg->has_telemetry ? msg->channel_util : 0.0f,
        msg->has_telemetry ? msg->air_util_tx : 0.0f,
        msg->rx_count,
        msg->is_pki ? 1 : 0);

    // Append to buffer
    if (meshy_sd_buf_pos + n < MESHY_SD_BUFSIZE) {
        memcpy(meshy_sd_buf + meshy_sd_buf_pos, row, n);
        meshy_sd_buf_pos += n;
        meshy_sd_writes++;
    }

    // Flush if buffer getting full or time/count threshold hit
    int64_t now_us = esp_timer_get_time();
    if (meshy_sd_buf_pos > MESHY_SD_BUFSIZE - 512 ||
        meshy_sd_writes >= MESHY_SD_SYNC_COUNT ||
        (now_us - meshy_sd_last_sync) > MESHY_SD_SYNC_US) {
        meshy_sd_flush();
    }
}

// ─── Node Table Helpers ─────────────────────────────────────────────────────

static meshy_node_t* find_or_create_node(uint32_t node_num) {
    // Find existing
    for (int i = 0; i < s_node_count; i++) {
        if (s_nodes[i].node_num == node_num) return &s_nodes[i];
    }
    // Create new
    if (s_node_count < MESHY_MAX_NODES) {
        meshy_node_t *n = &s_nodes[s_node_count++];
        memset(n, 0, sizeof(meshy_node_t));
        n->node_num = node_num;
        return n;
    }
    // Table full — evict oldest
    int oldest_idx = 0;
    uint32_t oldest_time = UINT32_MAX;
    for (int i = 0; i < s_node_count; i++) {
        if (s_nodes[i].last_seen_ms < oldest_time) {
            oldest_time = s_nodes[i].last_seen_ms;
            oldest_idx = i;
        }
    }
    meshy_node_t *n = &s_nodes[oldest_idx];
    memset(n, 0, sizeof(meshy_node_t));
    n->node_num = node_num;
    return n;
}

static void update_node_from_packet(const MeshRxResult *result) {
    xSemaphoreTake(s_node_mutex, portMAX_DELAY);

    meshy_node_t *node = find_or_create_node(result->packet.from);
    node->last_rssi = result->packet.rssi;
    node->last_snr  = result->packet.snr;
    node->last_seen_ms = esp_log_timestamp();

    // Update from decoded content
    if (result->data.portnum == PORT_NODEINFO) {
        MeshUser user;
        if (meshDecodeUser(result->data.payload, result->data.payload_len, &user)) {
            strncpy(node->long_name, user.long_name, sizeof(node->long_name) - 1);
            strncpy(node->short_name, user.short_name, sizeof(node->short_name) - 1);
            node->hw_model = user.hw_model;
            // Store public key for PKI DM decryption
            if (user.public_key_len == 32) {
                memcpy(node->public_key, user.public_key, 32);
                node->public_key_len = 32;
            }
            MESHY_LOGI("Node !%08lx: %s (%s)%s", (unsigned long)node->node_num,
                     node->long_name, node->short_name,
                     node->public_key_len == 32 ? " [PKI]" : "");
        }
    } else if (result->data.portnum == PORT_POSITION) {
        MeshPosition pos;
        if (meshDecodePosition(result->data.payload, result->data.payload_len, &pos)) {
            if (pos.latitude_i != 0 || pos.longitude_i != 0) {
                node->has_position = true;
                node->lat = pos.latitude();
                node->lon = pos.longitude();
                node->altitude = pos.altitude;
            }
        }
    } else if (result->data.portnum == PORT_TELEMETRY) {
        MeshTelemetry tel;
        if (meshDecodeTelemetry(result->data.payload, result->data.payload_len, &tel)) {
            if (tel.has_device_metrics) {
                node->battery_level = tel.device_metrics.battery_level;
            }
        }
    }

    s_stats.known_nodes = s_node_count;
    xSemaphoreGive(s_node_mutex);
}

// ─── PKI DM Decryption Fallback ─────────────────────────────────────────────
//
// Called when channel-based decryption fails. Checks if the packet is
// addressed to us, looks up the sender's public key, and attempts
// XChaCha20-Poly1305 decryption with the X25519 shared secret.

static bool try_pki_decrypt(const uint8_t *raw, size_t raw_len,
                             float rssi, float snr,
                             MeshRxResult *result)
{
    // Need our private key
    const uint8_t *our_priv = meshy_pki_private_key();
    if (!our_priv) return false;

    // Parse header (minimum check)
    if (raw_len < sizeof(MeshPacketHeader)) return false;
    if (!meshParsePacket(raw, raw_len, &result->packet)) return false;

    result->packet.rssi = rssi;
    result->packet.snr  = snr;

    // Only attempt PKI for packets addressed specifically to us (not broadcast)
    if (result->packet.to != s_session.node_num) return false;
    if (result->packet.to == MESH_ADDR_BROADCAST) return false;

    // Look up sender's public key from node table
    uint8_t sender_pubkey[32];
    bool found_key = false;

    xSemaphoreTake(s_node_mutex, portMAX_DELAY);
    for (int i = 0; i < s_node_count; i++) {
        if (s_nodes[i].node_num == result->packet.from && s_nodes[i].public_key_len == 32) {
            memcpy(sender_pubkey, s_nodes[i].public_key, 32);
            found_key = true;
            break;
        }
    }
    xSemaphoreGive(s_node_mutex);

    if (!found_key) {
        ESP_LOGD(TAG, "PKI: no public key for !%08lx", (unsigned long)result->packet.from);
        return false;
    }

    // Attempt XChaCha20-Poly1305 decryption
    uint8_t plaintext[240];
    size_t  plain_len = 0;

    if (!meshDecryptPki(our_priv, sender_pubkey, result->packet.from, result->packet.id,
                         result->packet.payload, result->packet.payload_len,
                         plaintext, &plain_len)) {
        ESP_LOGD(TAG, "PKI: decrypt failed for !%08lx id=%lu",
                 (unsigned long)result->packet.from, (unsigned long)result->packet.id);
        return false;
    }

    // Decode Data protobuf envelope
    if (!meshDecodeData(plaintext, plain_len, &result->data)) {
        ESP_LOGD(TAG, "PKI: Data decode failed after decrypt");
        return false;
    }

    result->channel_idx = -1;  // not a channel-based packet
    result->decrypted = true;
    result->is_pki = true;
    result->packet.channel_index = -1;

    ESP_LOGI(TAG, "PKI DM decrypted from !%08lx (port=%d, %zu bytes)",
             (unsigned long)result->packet.from, result->data.portnum, plain_len);

    return true;
}

// ─── Build meshy_msg_t from MeshRxResult ────────────────────────────────────

static void build_msg(const MeshRxResult *result, meshy_msg_t *msg) {
    memset(msg, 0, sizeof(meshy_msg_t));

    msg->from        = result->packet.from;
    msg->to          = result->packet.to;
    msg->id          = result->packet.id;
    msg->channel_idx = result->channel_idx;
    msg->rssi        = result->packet.rssi;
    msg->snr         = result->packet.snr;
    msg->hop_limit   = result->packet.hop_limit;
    msg->hop_start   = result->packet.hop_start;
    msg->portnum     = result->data.portnum;
    msg->rx_time_ms  = esp_log_timestamp();
    msg->rx_count    = 1;
    msg->is_pki      = result->is_pki;

    switch (result->data.portnum) {
        case PORT_TEXT_MESSAGE: {
            size_t tlen = result->data.payload_len;
            if (tlen >= MESHY_MAX_TEXT) tlen = MESHY_MAX_TEXT - 1;
            memcpy(msg->text, result->data.payload, tlen);
            msg->text[tlen] = '\0';
            msg->text_len = tlen;
            break;
        }
        case PORT_POSITION: {
            MeshPosition pos;
            if (meshDecodePosition(result->data.payload, result->data.payload_len, &pos)) {
                msg->has_position = true;
                msg->lat = pos.latitude();
                msg->lon = pos.longitude();
                msg->altitude = pos.altitude;
            }
            break;
        }
        case PORT_NODEINFO: {
            MeshUser user;
            if (meshDecodeUser(result->data.payload, result->data.payload_len, &user)) {
                msg->has_nodeinfo = true;
                strncpy(msg->long_name, user.long_name, sizeof(msg->long_name) - 1);
                strncpy(msg->short_name, user.short_name, sizeof(msg->short_name) - 1);
            }
            break;
        }
        case PORT_TELEMETRY: {
            MeshTelemetry tel;
            if (meshDecodeTelemetry(result->data.payload, result->data.payload_len, &tel)) {
                if (tel.has_device_metrics) {
                    msg->has_telemetry = true;
                    msg->battery_level = tel.device_metrics.battery_level;
                    msg->voltage = tel.device_metrics.voltage;
                    msg->channel_util = tel.device_metrics.channel_utilization;
                    msg->air_util_tx = tel.device_metrics.air_util_tx;
                }
            }
            break;
        }
        default:
            break;
    }
}

// ─── SX1262 Configuration for Meshtastic ────────────────────────────────────

// Map meshtastic BW (kHz) → Cpp_Bus_Driver enum
static Cpp_Bus_Driver::Sx126x::Lora_Bw meshBwToEnum(float bw_khz) {
    if (bw_khz >= 500.0f) return Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_500000HZ;
    if (bw_khz >= 250.0f) return Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_250000HZ;
    return Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_125000HZ;
}

// Map meshtastic SF (7-12) → Cpp_Bus_Driver enum
static Cpp_Bus_Driver::Sx126x::Sf meshSfToEnum(uint8_t sf) {
    switch (sf) {
        case 7:  return Cpp_Bus_Driver::Sx126x::Sf::SF7;
        case 8:  return Cpp_Bus_Driver::Sx126x::Sf::SF8;
        case 9:  return Cpp_Bus_Driver::Sx126x::Sf::SF9;
        case 10: return Cpp_Bus_Driver::Sx126x::Sf::SF10;
        case 11: return Cpp_Bus_Driver::Sx126x::Sf::SF11;
        case 12: return Cpp_Bus_Driver::Sx126x::Sf::SF12;
        default: return Cpp_Bus_Driver::Sx126x::Sf::SF9;
    }
}

// Map meshtastic CR denominator (5-8) → Cpp_Bus_Driver enum
static Cpp_Bus_Driver::Sx126x::Cr meshCrToEnum(uint8_t cr) {
    switch (cr) {
        case 5:  return Cpp_Bus_Driver::Sx126x::Cr::CR_4_5;
        case 6:  return Cpp_Bus_Driver::Sx126x::Cr::CR_4_6;
        case 7:  return Cpp_Bus_Driver::Sx126x::Cr::CR_4_7;
        case 8:  return Cpp_Bus_Driver::Sx126x::Cr::CR_4_8;
        default: return Cpp_Bus_Driver::Sx126x::Cr::CR_4_5;
    }
}

// Convert single-byte LoRa sync word to SX126x 2-byte register format
static uint16_t meshSyncWordToSx126x(uint8_t sw) {
    uint8_t hi = (sw & 0xF0) | 0x04;
    uint8_t lo = ((sw & 0x0F) << 4) | 0x04;
    return ((uint16_t)hi << 8) | lo;
}

static bool configure_radio(void) {
    if (!s_sx1262 || !s_xl9535) return false;

    const char *ch_name = s_session.channels.effectiveName(0);
    int8_t tx_pwr = (int8_t)g_settings.meshy_tx_power;
    MeshRadioConfig rc = s_session.radioConfig(tx_pwr);

    MESHY_LOGI("Configuring SX1262: %.3f MHz, BW%.0f, SF%d, CR4/%d, SW 0x%02X, %ddBm",
             rc.frequency_mhz, rc.bandwidth_khz, rc.spreading_factor,
             rc.coding_rate, rc.sync_word, rc.tx_power_dbm);

    // RF switch: LOW = RX path for the SKY13453
    s_xl9535->pin_write(XL9535_SKY13453_VCTL, Cpp_Bus_Driver::Xl95x5::Value::LOW);

    // Configure LoRa parameters via Cpp_Bus_Driver API
    if (!s_sx1262->config_lora_params(
            rc.frequency_mhz,
            meshBwToEnum(rc.bandwidth_khz),
            140.0f,                    // current limit mA
            rc.tx_power_dbm,
            meshSfToEnum(rc.spreading_factor),
            meshCrToEnum(rc.coding_rate),
            Cpp_Bus_Driver::Sx126x::Lora_Crc_Type::ON,
            rc.preamble_length,
            meshSyncWordToSx126x(rc.sync_word)))
    {
        ESP_LOGE(TAG, "config_lora_params failed");
        return false;
    }

    // Put radio into continuous RX
    s_sx1262->clear_buffer();
    s_sx1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
    s_sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
    s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

    s_stats.freq_mhz = rc.frequency_mhz;
    MESHY_LOGI("Radio configured, listening on %.3f MHz (ch: %s)", rc.frequency_mhz, ch_name);
    return true;
}

// ─── RX Task ────────────────────────────────────────────────────────────────

static void meshy_rx_task(void *arg) {
    ESP_LOGI(TAG, "RX task started");

    while (s_running) {
        // Poll DIO1 through XL9535 GPIO expander (I2C — no mutex needed)
        if (s_xl9535->pin_read(XL9535_SX1262_DIO1) == 1) {
            // ── SPI critical section: read IRQ, read data, read metrics ──
            xSemaphoreTake(s_spi_mutex, portMAX_DELAY);

            Cpp_Bus_Driver::Sx126x::Irq_Status irq;
            if (!s_sx1262->parse_irq_status(s_sx1262->get_irq_flag(), irq)) {
                s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
                xSemaphoreGive(s_spi_mutex);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (irq.all_flag.crc_error) {
                s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::CRC_ERROR);
                xSemaphoreGive(s_spi_mutex);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (irq.all_flag.tx_rx_timeout) {
                s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TIMEOUT);
                // Re-enter RX
                s_sx1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
                s_sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
                s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
                xSemaphoreGive(s_spi_mutex);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            if (irq.lora_reg_flag.header_error) {
                s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::HEADER_ERROR);
                xSemaphoreGive(s_spi_mutex);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // Read received data
            static uint8_t raw[256];
            uint8_t len = s_sx1262->receive_data(raw);

            // Clear RX_DONE IRQ — must happen after read, or DIO1 stays asserted
            // and we re-read the same buffer every poll cycle
            s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

            // Get RSSI/SNR
            float rssi = 0, snr = 0;
            Cpp_Bus_Driver::Sx126x::Packet_Metrics pm;
            if (s_sx1262->get_lora_packet_metrics(pm)) {
                rssi = pm.lora.rssi_instantaneous;
                snr  = pm.lora.snr;
            }

            // ── End SPI critical section ──
            xSemaphoreGive(s_spi_mutex);

            if (len == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            s_stats.rx_packets++;

            // Process through Meshtastic protocol
            MeshRxResult result;
            memset(&result, 0, sizeof(result));

            bool decoded = s_session.processRx(raw, len, rssi, snr, &result);

            // If channel decrypt failed, try PKI decryption for DMs addressed to us
            if (!decoded) {
                decoded = try_pki_decrypt(raw, len, rssi, snr, &result);
            }

            if (decoded) {
                s_stats.rx_decoded++;

                // Update node table
                update_node_from_packet(&result);

                // Build message for UI
                static meshy_msg_t msg;
                build_msg(&result, &msg);

                // Log to SD card
                meshy_sd_log_msg(&msg);

                // Push to queue (non-blocking — drop if full)
                xQueueSend(s_rx_queue, &msg, 0);

                // Store in ring buffer — dedup by packet ID
                bool is_dup = false;
                for (int i = 0; i < s_msg_count; i++) {
                    int idx = (s_msg_count < MESHY_MSG_HISTORY)
                            ? i
                            : (s_msg_write_idx + i) % MESHY_MSG_HISTORY;
                    if (s_msg_history[idx].id == msg.id && s_msg_history[idx].from == msg.from) {
                        s_msg_history[idx].rx_count++;
                        is_dup = true;
                        break;
                    }
                }
                if (!is_dup) {
                    s_msg_history[s_msg_write_idx] = msg;
                    s_msg_write_idx = (s_msg_write_idx + 1) % MESHY_MSG_HISTORY;
                    if (s_msg_count < MESHY_MSG_HISTORY) s_msg_count++;
                }

                // Log decoded messages
                const char *port_name = "?";
                switch (result.data.portnum) {
                    case PORT_TEXT_MESSAGE: port_name = "TEXT"; break;
                    case PORT_POSITION:    port_name = "POS";  break;
                    case PORT_NODEINFO:    port_name = "NODE"; break;
                    case PORT_TELEMETRY:   port_name = "TELEM"; break;
                    case PORT_ROUTING:     port_name = "ROUTE"; break;
                    default: break;
                }

                // Look up sender name
                const char *sender = "?";
                xSemaphoreTake(s_node_mutex, portMAX_DELAY);
                for (int i = 0; i < s_node_count; i++) {
                    if (s_nodes[i].node_num == result.packet.from && s_nodes[i].short_name[0]) {
                        sender = s_nodes[i].short_name;
                        break;
                    }
                }
                xSemaphoreGive(s_node_mutex);

                if (result.is_pki) {
                    MESHY_LOGI("[%s] !%08lx (%s) id:%08lx RSSI:%.0f SNR:%.1f hop:%d/%d ch:PKI",
                             port_name, (unsigned long)result.packet.from, sender,
                             (unsigned long)result.packet.id,
                             rssi, snr, result.packet.hop_limit, result.packet.hop_start);
                } else {
                    MESHY_LOGI("[%s] !%08lx (%s) id:%08lx RSSI:%.0f SNR:%.1f hop:%d/%d ch:%d",
                             port_name, (unsigned long)result.packet.from, sender,
                             (unsigned long)result.packet.id,
                             rssi, snr, result.packet.hop_limit, result.packet.hop_start,
                             result.channel_idx);
                }

                if (result.data.portnum == PORT_TEXT_MESSAGE) {
                    MESHY_LOGI("  \"%.*s\"", (int)msg.text_len, msg.text);
                } else if (result.data.portnum == PORT_POSITION && msg.has_position) {
                    MESHY_LOGI("  pos:%.6f,%.6f alt:%d", msg.lat, msg.lon, (int)msg.altitude);
                } else if (result.data.portnum == PORT_TELEMETRY && msg.has_telemetry) {
                    MESHY_LOGI("  bat:%lu%% %.2fV chUtil:%.1f%% airTx:%.1f%%",
                             (unsigned long)msg.battery_level, msg.voltage,
                             msg.channel_util, msg.air_util_tx);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms poll — SF11/BW250 min airtime is ~230ms
    }

    ESP_LOGI(TAG, "RX task exiting");
    s_rx_task_hdl = nullptr;
    vTaskDeleteWithCaps(NULL);
}

// ─── TX Task ────────────────────────────────────────────────────────────────

// NODEINFO state
static bool     s_nodeinfo_sent = false;
static uint32_t s_nodeinfo_next_ms = 0;

/**
 * Transmit a raw frame over SX1262.
 * Handles the full send_data → set_tx → poll → return-to-RX sequence.
 * Returns true on success (TX_DONE received within 5s).
 */
static bool transmit_frame(const uint8_t *frame, size_t frame_len) {
    // CSMA/CA random backoff (before taking mutex — don't block RX during backoff)
    uint32_t delay = meshTxDelayMs(s_session.preset);
    if (delay > 0) {
        ESP_LOGI(TAG, "CSMA/CA backoff: %lu ms", (unsigned long)delay);
        vTaskDelay(pdMS_TO_TICKS(delay));
    }

    // ── SPI critical section: FIFO write → TX → poll → return to RX ──
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);

    // Step 1: Write frame to SX1262 FIFO
    // send_data takes non-const uint8_t* (bus driver API), but does not modify
    if (!s_sx1262->send_data(const_cast<uint8_t *>(frame), (uint8_t)frame_len)) {
        ESP_LOGE(TAG, "send_data (buffer write) failed");
        xSemaphoreGive(s_spi_mutex);
        return false;
    }

    // Step 2: Switch RF antenna to TX path (SKY13453: HIGH = TX)
    s_xl9535->pin_write(XL9535_SKY13453_VCTL, Cpp_Bus_Driver::Xl95x5::Value::HIGH);

    // Step 3: Configure IRQ for TX_DONE on DIO1
    s_sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);
    s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);

    // Step 4: Issue SetTx — radio starts transmitting from buffer NOW
    s_sx1262->set_tx(0);

    // Step 5: Wait for TX_DONE (poll DIO1)
    uint16_t timeout = 0;
    bool tx_ok = false;
    while (timeout < 500) {  // 5 second max
        if (s_xl9535->pin_read(XL9535_SX1262_DIO1) == 1) {
            Cpp_Bus_Driver::Sx126x::Irq_Status irq;
            if (s_sx1262->parse_irq_status(s_sx1262->get_irq_flag(), irq)) {
                if (irq.all_flag.tx_done) {
                    tx_ok = true;
                    break;
                }
            }
        }
        timeout++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);

    if (!tx_ok) {
        ESP_LOGE(TAG, "TX timeout after %d ms", timeout * 10);
    }

    // Step 6: Return to RX
    s_xl9535->pin_write(XL9535_SKY13453_VCTL, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    s_sx1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
    s_sx1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
    s_sx1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

    // ── End SPI critical section ──
    xSemaphoreGive(s_spi_mutex);

    return tx_ok;
}

/**
 * Build and transmit a NODEINFO broadcast.
 * Uses identity from NVS store (meshy_channels).
 */
static bool transmit_nodeinfo(void) {
    const char *long_name  = meshy_channels_long_name();
    const char *short_name = meshy_channels_short_name();
    bool ok_to_mqtt = g_settings.meshy_ok_to_mqtt;

    // Build "!XXXXXXXX" id string from our node_num
    char id_str[16];
    snprintf(id_str, sizeof(id_str), "!%08lx", (unsigned long)s_session.node_num);

    // Get PKI public key (NULL if not yet generated)
    const uint8_t *pub_key = meshy_pki_public_key();
    uint8_t pub_key_len = pub_key ? 32 : 0;

    static uint8_t frame[256];
    size_t frame_len = s_session.buildNodeInfoTx(
        0, MESH_ADDR_BROADCAST,
        id_str, long_name, short_name,
        255,  // PRIVATE_HW — not an official Meshtastic device
        false, ok_to_mqtt, frame,
        pub_key, pub_key_len,
        g_settings.meshy_hop_limit);

    if (frame_len == 0) {
        ESP_LOGE(TAG, "Failed to build NODEINFO frame");
        return false;
    }

    ESP_LOGI(TAG, "NODEINFO frame built: %zu bytes", frame_len);

    if (transmit_frame(frame, frame_len)) {
        s_stats.tx_packets++;
        MESHY_LOGI("[TX:NODE] \"%s\" (%s)", long_name, short_name);
        return true;
    }
    return false;
}

static void meshy_tx_task(void *arg) {
    ESP_LOGI(TAG, "TX task started");

    while (s_running) {
        static tx_request_t req;
        if (xQueueReceive(s_tx_queue, &req, pdMS_TO_TICKS(200)) == pdTRUE) {
            // ── First TX: send NODEINFO before the text message ──
            if (!s_nodeinfo_sent) {
                if (transmit_nodeinfo()) {
                    s_nodeinfo_sent = true;
                    uint32_t period_ms = (uint32_t)g_settings.meshy_nodeinfo_period_m * 60 * 1000;
                    s_nodeinfo_next_ms = esp_log_timestamp() + period_ms;
                }
                vTaskDelay(pdMS_TO_TICKS(4000));  // let NODEINFO propagate before text
            }

            // ── Text message TX ──
            bool ok_to_mqtt = g_settings.meshy_ok_to_mqtt;

            ESP_LOGI(TAG, "TX build: ch=%d to=0x%08lx text_len=%d hop=%d mqtt=%d",
                     req.channel_idx, (unsigned long)req.to,
                     (int)strlen(req.text), g_settings.meshy_hop_limit, ok_to_mqtt);

            static uint8_t frame[256];
            size_t frame_len = s_session.buildTextTx(
                req.channel_idx, req.to, req.text, false, ok_to_mqtt, frame,
                g_settings.meshy_hop_limit);

            if (frame_len == 0) {
                ESP_LOGE(TAG, "Failed to build TX frame");
                continue;
            }

            ESP_LOGI(TAG, "TX frame built: %zu bytes", frame_len);

            if (transmit_frame(frame, frame_len)) {
                s_stats.tx_packets++;
                MESHY_LOGI("[TX] !%08lx id:%08lx \"%s\"",
                          (unsigned long)s_session.node_num,
                          (unsigned long)s_session.last_tx_id, req.text);

                // Echo sent message into history so it appears on screen
                if (s_msg_history) {
                    meshy_msg_t echo;
                    memset(&echo, 0, sizeof(echo));
                    echo.from = s_session.node_num;
                    echo.to = req.to;
                    echo.id = s_session.last_tx_id;  // match actual TX packet ID
                    echo.portnum = PORT_TEXT_MESSAGE;
                    strncpy(echo.text, req.text, MESHY_MAX_TEXT - 1);
                    echo.text_len = strlen(req.text);
                    echo.rx_time_ms = esp_log_timestamp();
                    echo.rx_count = 1;

                    s_msg_history[s_msg_write_idx] = echo;
                    s_msg_write_idx = (s_msg_write_idx + 1) % MESHY_MSG_HISTORY;
                    if (s_msg_count < MESHY_MSG_HISTORY) s_msg_count++;

                    // Also log to SD
                    meshy_sd_log_msg(&echo);
                }
            }
        } else {
            // ── Queue timeout (200ms) — check periodic NODEINFO ──
            if (s_nodeinfo_sent && esp_log_timestamp() >= s_nodeinfo_next_ms) {
                if (transmit_nodeinfo()) {
                    uint32_t period_ms = (uint32_t)g_settings.meshy_nodeinfo_period_m * 60 * 1000;
                    s_nodeinfo_next_ms = esp_log_timestamp() + period_ms;
                }
            }
        }
    }

    ESP_LOGI(TAG, "TX task exiting");
    s_tx_task_hdl = nullptr;
    vTaskDeleteWithCaps(NULL);
}

// ─── Public API ─────────────────────────────────────────────────────────────

// Map device_settings indices to meshtastic-lite enums.
// The settings UI presents a curated subset in display order;
// the radio library uses a different numbering.

static MeshModemPreset settings_to_preset(uint8_t idx) {
    static const MeshModemPreset map[] = {
        MODEM_LONG_FAST,    // 0: LongFast
        MODEM_LONG_SLOW,    // 1: LongSlow
        MODEM_MEDIUM_FAST,  // 2: MediumFast
        MODEM_MEDIUM_SLOW,  // 3: MediumSlow
        MODEM_SHORT_FAST,   // 4: ShortFast
        MODEM_SHORT_SLOW,   // 5: ShortSlow
    };
    return (idx < sizeof(map)/sizeof(map[0])) ? map[idx] : MODEM_MEDIUM_FAST;
}

static MeshRegion settings_to_region(uint8_t idx) {
    static const MeshRegion map[] = {
        REGION_US,       // 0:  US
        REGION_EU_868,   // 1:  EU_868
        REGION_EU_433,   // 2:  EU_433
        REGION_CN,       // 3:  CN
        REGION_JP,       // 4:  JP
        REGION_ANZ,      // 5:  ANZ
        REGION_KR,       // 6:  KR
        REGION_TW,       // 7:  TW
        REGION_RU,       // 8:  RU
        REGION_IN,       // 9:  IN
        REGION_NZ_865,   // 10: NZ
        REGION_TH,       // 11: TH
        REGION_UNSET,    // 12: UA (no dedicated band plan)
        REGION_UNSET,    // 13: MY
        REGION_UNSET,    // 14: SG
    };
    return (idx < sizeof(map)/sizeof(map[0])) ? map[idx] : REGION_US;
}

static MeshRole settings_to_role(uint8_t idx) {
    static const MeshRole map[] = {
        ROLE_CLIENT,       // 0: Client
        ROLE_CLIENT_MUTE,  // 1: Client Mute
        ROLE_ROUTER_LATE,  // 2: Router Late
    };
    return (idx < sizeof(map)/sizeof(map[0])) ? map[idx] : ROLE_CLIENT;
}

// These must be set before calling meshy_start().
// Call from main.cpp: meshy_set_hw(SX1262.get(), XL9535.get());
extern "C" void meshy_set_hw(void *sx1262_ptr, void *xl9535_ptr) {
    s_sx1262 = static_cast<Cpp_Bus_Driver::Sx126x *>(sx1262_ptr);
    s_xl9535 = static_cast<Cpp_Bus_Driver::Xl95x5 *>(xl9535_ptr);
}

extern "C" bool meshy_start(void) {
    if (s_running) return true;
    if (!s_sx1262 || !s_xl9535) {
        ESP_LOGE(TAG, "Hardware not initialized — call meshy_set_hw() first");
        return false;
    }

    // Read settings
    MeshRegion      region = settings_to_region(g_settings.meshy_region);
    MeshModemPreset preset = settings_to_preset(g_settings.meshy_preset);
    MeshRole        role   = settings_to_role(g_settings.meshy_role);
    int8_t          tx_pwr = (int8_t)g_settings.meshy_tx_power;

    const char *region_name = settings_region_name(g_settings.meshy_region);
    const char *preset_name = settings_preset_name(g_settings.meshy_preset);
    const char *role_name   = settings_role_name(g_settings.meshy_role);

    // Initialize Meshtastic session
    // Node number: use lower 4 bytes of base MAC
    uint8_t mac[8];
    esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);
    uint32_t node_num = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                        ((uint32_t)mac[4] << 8) | mac[5];

    // freq_slot: 0=auto (hash-based), 1+=explicit override
    int32_t freq_slot = g_settings.meshy_freq_slot > 0
        ? (int32_t)(g_settings.meshy_freq_slot - 1)  // settings 1-based → 0-based slot
        : -1;                                          // -1 = hash-based auto

    s_session.init(region, preset, role, node_num, freq_slot);
    s_session.channels.addDefaultChannel();

    // Add extra channels from NVS store
    int extra_added = 0;
    for (int i = 0; i < meshy_channels_count(); i++) {
        const meshy_channel_cfg_t *ch = meshy_channels_get(i);
        if (!ch || !ch->enabled) continue;
        int idx = s_session.channels.addChannel(
            ch->name, ch->psk, ch->psk_len, false);
        if (idx >= 0) extra_added++;
    }

    MESHY_LOGI("Session initialized: node=!%08lx region=%s preset=%s role=%s tx=%ddBm hop=%d slot=%s +%dch",
             (unsigned long)node_num, region_name, preset_name, role_name,
             tx_pwr, g_settings.meshy_hop_limit,
             g_settings.meshy_freq_slot > 0 ? "manual" : "auto",
             extra_added);

    // Ensure PKI keypair exists (generates on first boot, loads from NVS after)
    meshy_pki_ensure();

    // Create queues and allocate message history in PSRAM
    if (!s_rx_queue) s_rx_queue = xQueueCreate(16, sizeof(meshy_msg_t));
    if (!s_tx_queue) s_tx_queue = xQueueCreate(8, sizeof(tx_request_t));
    if (!s_node_mutex) s_node_mutex = xSemaphoreCreateMutex();
    if (!s_spi_mutex)  s_spi_mutex  = xSemaphoreCreateMutex();
    if (!s_msg_history) {
        s_msg_history = (meshy_msg_t *)heap_caps_calloc(MESHY_MSG_HISTORY, sizeof(meshy_msg_t), MALLOC_CAP_SPIRAM);
        if (!s_msg_history) {
            ESP_LOGE(TAG, "Failed to allocate message history in PSRAM (%zu bytes)",
                     MESHY_MSG_HISTORY * sizeof(meshy_msg_t));
            return false;
        }
        MESHY_LOGI("Message history: %d slots in PSRAM (%zu bytes)",
                 MESHY_MSG_HISTORY, MESHY_MSG_HISTORY * sizeof(meshy_msg_t));
    }
    if (!s_nodes) {
        s_nodes = (meshy_node_t *)heap_caps_calloc(MESHY_MAX_NODES, sizeof(meshy_node_t), MALLOC_CAP_SPIRAM);
        if (!s_nodes) {
            ESP_LOGE(TAG, "Failed to allocate node table in PSRAM (%zu bytes)",
                     MESHY_MAX_NODES * sizeof(meshy_node_t));
            return false;
        }
        MESHY_LOGI("Node table: %d slots in PSRAM (%zu bytes)",
                 MESHY_MAX_NODES, MESHY_MAX_NODES * sizeof(meshy_node_t));
    }

    // Reset stats
    memset(&s_stats, 0, sizeof(s_stats));

    // Configure radio
    if (!configure_radio()) {
        ESP_LOGE(TAG, "Radio configuration failed");
        return false;
    }

    s_running = true;
    s_stats.running = true;

    // Meshy tasks do SPI transactions to the SX1262.
    // Previously required internal RAM stacks because SPI DMA accessed stack buffers.
    // Now safe for PSRAM stacks: hardware_spi.cpp pre-allocates DMA bounce buffers
    // in internal RAM and copies to/from them — task stack location doesn't matter.
    // Using PSRAM stacks frees ~10KB of internal RAM for other allocations.
    xTaskCreateWithCaps(meshy_rx_task, "meshy_rx", 6144, NULL, 3, &s_rx_task_hdl, MALLOC_CAP_SPIRAM);
    xTaskCreateWithCaps(meshy_tx_task, "meshy_tx", 4096, NULL, 2, &s_tx_task_hdl, MALLOC_CAP_SPIRAM);

    MESHY_LOGI("Meshy started — listening on %.3f MHz", s_stats.freq_mhz);
    return true;
}

extern "C" void meshy_stop(void) {
    if (!s_running) return;
    s_running = false;
    s_stats.running = false;

    // Wait for tasks to exit
    for (int i = 0; i < 20 && (s_rx_task_hdl || s_tx_task_hdl); i++)
        vTaskDelay(pdMS_TO_TICKS(100));

    // Put radio to standby (actual power-down, not back into RX)
    if (s_sx1262) {
        s_sx1262->set_standby(Cpp_Bus_Driver::Sx126x::Stdby_Config::STDBY_RC);
    }

    // Flush any buffered SD log data
    meshy_sd_flush();

    MESHY_LOGI("Meshy stopped");
}

extern "C" void meshy_restart(void) {
    MESHY_LOGI("Restarting with new settings...");
    meshy_stop();
    vTaskDelay(pdMS_TO_TICKS(100));  // let tasks fully clean up
    meshy_start();
}

extern "C" bool meshy_send_text(const char *text) {
    if (!s_running || !s_tx_queue) return false;

    tx_request_t req;
    req.to = MESH_ADDR_BROADCAST;
    req.channel_idx = 0;
    strncpy(req.text, text, MESHY_MAX_TEXT - 1);
    req.text[MESHY_MAX_TEXT - 1] = '\0';

    return xQueueSend(s_tx_queue, &req, pdMS_TO_TICKS(100)) == pdTRUE;
}

extern "C" bool meshy_send_dm(uint32_t dest_node, const char *text) {
    if (!s_running || !s_tx_queue) return false;

    tx_request_t req;
    req.to = dest_node;
    req.channel_idx = 0;
    strncpy(req.text, text, MESHY_MAX_TEXT - 1);
    req.text[MESHY_MAX_TEXT - 1] = '\0';

    return xQueueSend(s_tx_queue, &req, pdMS_TO_TICKS(100)) == pdTRUE;
}

extern "C" bool meshy_recv(meshy_msg_t *msg) {
    if (!s_rx_queue) return false;
    return xQueueReceive(s_rx_queue, msg, 0) == pdTRUE;
}

extern "C" int meshy_get_nodes(meshy_node_t *out, int max_count) {
    if (!s_node_mutex) return 0;
    xSemaphoreTake(s_node_mutex, portMAX_DELAY);
    int count = (s_node_count < max_count) ? s_node_count : max_count;
    memcpy(out, s_nodes, count * sizeof(meshy_node_t));
    xSemaphoreGive(s_node_mutex);
    return count;
}

extern "C" meshy_stats_t meshy_get_stats(void) {
    return s_stats;
}

extern "C" int meshy_format_nodes(char *buf, int bufsize) {
    int pos = 0;
    if (!s_node_mutex) return 0;

    xSemaphoreTake(s_node_mutex, portMAX_DELAY);
    for (int i = 0; i < s_node_count && pos < bufsize - 80; i++) {
        meshy_node_t *n = &s_nodes[i];
        const char *name = n->short_name[0] ? n->short_name : "???";
        uint32_t age_s = (esp_log_timestamp() - n->last_seen_ms) / 1000;

        pos += snprintf(buf + pos, bufsize - pos,
            "!%08lX %-4s %-16s RSSI:%.0f %lus ago\n",
            (unsigned long)n->node_num, name,
            n->long_name[0] ? n->long_name : "",
            n->last_rssi, (unsigned long)age_s);
    }
    xSemaphoreGive(s_node_mutex);

    if (pos == 0) pos += snprintf(buf + pos, bufsize - pos, "No nodes heard");
    return s_node_count;
}

extern "C" int meshy_format_messages(char *buf, int bufsize) {
    int pos = 0;
    int count = 0;

    if (!s_msg_history || s_msg_count == 0) {
        pos += snprintf(buf + pos, bufsize - pos, "No messages yet\nListening on %.3f MHz...", s_stats.freq_mhz);
        return 0;
    }

    // Compute wall-clock offset: current epoch minus boot ms
    time_t now_epoch;
    time(&now_epoch);
    uint32_t now_ms = esp_log_timestamp();
    bool have_time = (now_epoch > 1700000000);  // sanity: after ~2023

    // Walk ring buffer from oldest to newest
    int start = (s_msg_count < MESHY_MSG_HISTORY)
              ? 0
              : s_msg_write_idx;

    for (int i = 0; i < s_msg_count && pos < bufsize - 120; i++) {
        int idx = (start + i) % MESHY_MSG_HISTORY;
        meshy_msg_t *m = &s_msg_history[idx];

        // Compute message timestamp
        char ts[8] = "";
        if (have_time) {
            time_t msg_epoch = now_epoch - (int32_t)(now_ms - m->rx_time_ms) / 1000;
            struct tm t;
            gmtime_r(&msg_epoch, &t);
            snprintf(ts, sizeof(ts), "%02d:%02d ", t.tm_hour, t.tm_min);
        }

        // Look up sender name — show "You" for our own TX echoes
        const char *name = "???";
        if (m->from == s_session.node_num) {
            name = "You";
        } else if (s_node_mutex) {
            xSemaphoreTake(s_node_mutex, portMAX_DELAY);
            for (int j = 0; j < s_node_count; j++) {
                if (s_nodes[j].node_num == m->from && s_nodes[j].short_name[0]) {
                    name = s_nodes[j].short_name;
                    break;
                }
            }
            xSemaphoreGive(s_node_mutex);
        }

        // Repeat indicator suffix
        const char *rx_suffix = "";
        char rx_buf[12] = "";
        if (m->rx_count > 1) {
            snprintf(rx_buf, sizeof(rx_buf), " x%d", m->rx_count);
            rx_suffix = rx_buf;
        }

        switch (m->portnum) {
            case PORT_TEXT_MESSAGE:
                pos += snprintf(buf + pos, bufsize - pos,
                    "%s[%s] %s%s\n", ts, name, m->text, rx_suffix);
                count++;
                break;
            case PORT_POSITION:
                if (m->has_position) {
                    pos += snprintf(buf + pos, bufsize - pos,
                        "%s[%s] @ %.4f,%.4f %dm%s\n",
                        ts, name, m->lat, m->lon, (int)m->altitude, rx_suffix);
                    count++;
                }
                break;
            case PORT_NODEINFO:
                if (m->has_nodeinfo) {
                    pos += snprintf(buf + pos, bufsize - pos,
                        "%s[%s] joined: %s%s\n", ts, m->short_name, m->long_name, rx_suffix);
                    count++;
                }
                break;
            case PORT_TELEMETRY:
                if (m->has_telemetry) {
                    pos += snprintf(buf + pos, bufsize - pos,
                        "%s[%s] bat:%lu%% %.1fV ch:%.0f%% air:%.0f%%%s\n",
                        ts, name, (unsigned long)m->battery_level, m->voltage,
                        m->channel_util, m->air_util_tx, rx_suffix);
                    count++;
                }
                break;
            default:
                break;
        }
    }

    if (pos == 0) pos += snprintf(buf + pos, bufsize - pos, "No messages yet\nListening on %.3f MHz...", s_stats.freq_mhz);
    return count;
}
