/*
 * meshy_channels.h — Meshtastic channel configuration store
 *
 * Stores up to 7 additional channels (beyond the default primary channel)
 * in a separate NVS blob. Channels include a name and PSK per entry.
 *
 * The default channel (PSK index 1, empty name) is always channel 0 and
 * is NOT stored here — it's implicit. This store holds channels 1–7.
 *
 * Usage:
 *   meshy_channels_load();              // call once at boot
 *   int n = meshy_channels_count();     // how many extra channels configured
 *   const meshy_channel_cfg_t *ch = meshy_channels_get(i);  // get channel i
 *   meshy_channels_set(0, "MyChannel", psk, 32, true);      // add/update
 *   meshy_channels_remove(0);           // remove
 *   meshy_channels_save();              // persist to NVS
 *   meshy_channels_reset();             // clear all extra channels
 *
 * NVS key: "meshy_ch" in namespace "meshy_settings"
 * Version: bumped when struct layout changes (same migration pattern as device_settings)
 *
 * Part of ADS-B Scope — T-Display-P4.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"

#ifdef __cplusplus
extern "C" {
#endif

// ─── Configuration ──────────────────────────────────────────────────────────

#define MESHY_CH_VERSION           3     // bump when struct changes
#define MESHY_CH_MAX_EXTRA         7     // + 1 default = 8 total
#define MESHY_CH_NAME_LEN         12     // Meshtastic channel names are ≤11 chars + NUL
#define MESHY_CH_PSK_MAX          32     // AES-256 max key size

#define MESHY_CH_NVS_NAMESPACE    "meshy_settings"
#define MESHY_CH_NVS_KEY          "meshy_ch"

#define MESHY_LONG_NAME_LEN       40
#define MESHY_SHORT_NAME_LEN       5

// ─── Per-channel config (stored) ────────────────────────────────────────────

typedef struct {
    char    name[MESHY_CH_NAME_LEN];    // channel name (empty = inherit preset name)
    uint8_t psk[MESHY_CH_PSK_MAX];      // raw PSK bytes
    uint8_t psk_len;                     // 0=no encrypt, 1=short index, 16=AES-128, 32=AES-256
    bool    enabled;                     // channel active for RX/TX
} meshy_channel_cfg_t;  // 46 bytes

// ─── Channel store blob (NVS) ───────────────────────────────────────────────

typedef struct {
    uint8_t             count;                             // number of extra channels (0–7)
    meshy_channel_cfg_t channels[MESHY_CH_MAX_EXTRA];      // extra channels (index 0 here = channel 1 overall)
    char                node_long_name[MESHY_LONG_NAME_LEN];  // persisted identity (e.g., "ADS-B Scope a3f7")
    char                node_short_name[MESHY_SHORT_NAME_LEN]; // 4-char abbreviation (e.g., "a3f7")
    // PKI keypair (v3+) — X25519 for encrypted DMs
    uint8_t             pki_private_key[32];   // X25519 private key (never transmitted)
    uint8_t             pki_public_key[32];    // X25519 public key (sent in NODEINFO)
    uint8_t             pki_valid;             // 1 = keypair generated, 0 = not yet
    uint8_t             _reserved[15];         // future fields
} meshy_channel_store_t;

// ─── Runtime state ──────────────────────────────────────────────────────────

// Global instance — valid after meshy_channels_load()
extern meshy_channel_store_t g_meshy_channels;

// ─── API ────────────────────────────────────────────────────────────────────

/**
 * Load channel config from NVS. Call once at boot.
 * If no saved config exists, initializes to empty (0 extra channels).
 */
static inline void meshy_channels_load(void) {
    // Apply defaults
    memset(&g_meshy_channels, 0, sizeof(g_meshy_channels));

    nvs_handle_t nvs;
    if (nvs_open(MESHY_CH_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI("MESHY_CH", "No NVS — using defaults (0 extra channels)");
        return;
    }

    uint8_t stored_version = 0;
    nvs_get_u8(nvs, "ch_ver", &stored_version);

    // Read raw blob
    uint8_t raw[sizeof(meshy_channel_store_t)] = {0};
    size_t len = sizeof(raw);
    if (nvs_get_blob(nvs, MESHY_CH_NVS_KEY, raw, &len) != ESP_OK) {
        ESP_LOGI("MESHY_CH", "No channel blob — using defaults");
        nvs_close(nvs);
        return;
    }
    nvs_close(nvs);

    if (stored_version > MESHY_CH_VERSION) {
        ESP_LOGW("MESHY_CH", "Channel config v%d > firmware v%d — using defaults",
                 stored_version, MESHY_CH_VERSION);
        return;
    }

    // Migration: copy old blob over defaults (same pattern as device_settings)
    size_t copy = (len < sizeof(g_meshy_channels)) ? len : sizeof(g_meshy_channels);
    memcpy(&g_meshy_channels, raw, copy);

    if (stored_version < MESHY_CH_VERSION) {
        ESP_LOGI("MESHY_CH", "Migrated channels v%d -> v%d (%zu bytes)",
                 stored_version, MESHY_CH_VERSION, len);
        // Future: add explicit field fixups here
    } else {
        ESP_LOGI("MESHY_CH", "Loaded %d extra channel(s) (v%d)",
                 g_meshy_channels.count, stored_version);
    }

    // Validate count
    if (g_meshy_channels.count > MESHY_CH_MAX_EXTRA) {
        g_meshy_channels.count = 0;
    }

    // Validate each channel
    for (int i = 0; i < g_meshy_channels.count; i++) {
        meshy_channel_cfg_t *ch = &g_meshy_channels.channels[i];
        ch->name[MESHY_CH_NAME_LEN - 1] = '\0';  // ensure NUL terminated
        if (ch->psk_len > MESHY_CH_PSK_MAX) ch->psk_len = 0;
    }

    // Ensure NUL termination on identity strings
    g_meshy_channels.node_long_name[MESHY_LONG_NAME_LEN - 1] = '\0';
    g_meshy_channels.node_short_name[MESHY_SHORT_NAME_LEN - 1] = '\0';
}

/**
 * Save channel config to NVS. Call after any modification.
 */
static inline void meshy_channels_save(void) {
    nvs_handle_t nvs;
    if (nvs_open(MESHY_CH_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE("MESHY_CH", "Failed to open NVS for writing");
        return;
    }

    nvs_set_u8(nvs, "ch_ver", MESHY_CH_VERSION);
    if (nvs_set_blob(nvs, MESHY_CH_NVS_KEY, &g_meshy_channels,
                     sizeof(g_meshy_channels)) != ESP_OK) {
        ESP_LOGE("MESHY_CH", "Failed to write channel blob");
    } else {
        nvs_commit(nvs);
        ESP_LOGI("MESHY_CH", "Saved %d extra channel(s)", g_meshy_channels.count);
    }
    nvs_close(nvs);
}

/**
 * Clear all extra channels and save.
 */
static inline void meshy_channels_reset(void) {
    memset(&g_meshy_channels, 0, sizeof(g_meshy_channels));
    meshy_channels_save();
    ESP_LOGI("MESHY_CH", "Channel config reset — default channel only");
}

/**
 * Get count of extra channels.
 */
static inline int meshy_channels_count(void) {
    return g_meshy_channels.count;
}

/**
 * Get extra channel by index (0-based, where 0 = first extra = channel 1 overall).
 * Returns NULL if index out of range.
 */
static inline const meshy_channel_cfg_t *meshy_channels_get(int idx) {
    if (idx < 0 || idx >= g_meshy_channels.count) return NULL;
    return &g_meshy_channels.channels[idx];
}

/**
 * Set/update an extra channel. If idx == count, appends (if room).
 * Does NOT auto-save — call meshy_channels_save() after.
 * Returns true on success.
 */
static inline bool meshy_channels_set(int idx, const char *name,
                                       const uint8_t *psk, uint8_t psk_len,
                                       bool enabled) {
    if (idx < 0 || idx > MESHY_CH_MAX_EXTRA) return false;
    if (idx >= MESHY_CH_MAX_EXTRA) return false;

    meshy_channel_cfg_t *ch = &g_meshy_channels.channels[idx];
    memset(ch, 0, sizeof(*ch));
    if (name) strncpy(ch->name, name, MESHY_CH_NAME_LEN - 1);
    if (psk && psk_len > 0 && psk_len <= MESHY_CH_PSK_MAX) {
        memcpy(ch->psk, psk, psk_len);
        ch->psk_len = psk_len;
    }
    ch->enabled = enabled;

    if (idx >= g_meshy_channels.count) {
        g_meshy_channels.count = idx + 1;
    }

    return true;
}

/**
 * Remove an extra channel by index, shifting remaining channels down.
 * Does NOT auto-save — call meshy_channels_save() after.
 */
static inline bool meshy_channels_remove(int idx) {
    if (idx < 0 || idx >= g_meshy_channels.count) return false;

    // Shift remaining channels down
    for (int i = idx; i < g_meshy_channels.count - 1; i++) {
        g_meshy_channels.channels[i] = g_meshy_channels.channels[i + 1];
    }
    g_meshy_channels.count--;
    memset(&g_meshy_channels.channels[g_meshy_channels.count], 0,
           sizeof(meshy_channel_cfg_t));
    return true;
}

// ─── Node Identity ──────────────────────────────────────────────────────────

/**
 * Ensure node identity exists. If long_name is empty, generate a random one.
 * Call after meshy_channels_load(). Auto-saves if identity was generated.
 */
static inline void meshy_channels_ensure_identity(void) {
    if (g_meshy_channels.node_long_name[0] != '\0') {
        ESP_LOGI("MESHY_CH", "Identity: \"%s\" (%s)",
                 g_meshy_channels.node_long_name, g_meshy_channels.node_short_name);
        return;
    }

    // Generate random 4-byte hex identity
    uint32_t rnd = esp_random();
    char hex[9];
    snprintf(hex, sizeof(hex), "%08lx", (unsigned long)rnd);

    // short_name = last 4 hex chars
    snprintf(g_meshy_channels.node_short_name, MESHY_SHORT_NAME_LEN, "%.4s", hex + 4);

    // long_name = "ADS-B Scope XXXX"
    snprintf(g_meshy_channels.node_long_name, MESHY_LONG_NAME_LEN, "ADS-B Scope %.4s", hex + 4);

    meshy_channels_save();
    ESP_LOGI("MESHY_CH", "Generated identity: \"%s\" (%s)",
             g_meshy_channels.node_long_name, g_meshy_channels.node_short_name);
}

/**
 * Set node identity. Saves to NVS immediately.
 */
static inline void meshy_channels_set_identity(const char *long_name, const char *short_name) {
    strncpy(g_meshy_channels.node_long_name, long_name, MESHY_LONG_NAME_LEN - 1);
    g_meshy_channels.node_long_name[MESHY_LONG_NAME_LEN - 1] = '\0';
    strncpy(g_meshy_channels.node_short_name, short_name, MESHY_SHORT_NAME_LEN - 1);
    g_meshy_channels.node_short_name[MESHY_SHORT_NAME_LEN - 1] = '\0';
    meshy_channels_save();
    ESP_LOGI("MESHY_CH", "Identity set: \"%s\" (%s)",
             g_meshy_channels.node_long_name, g_meshy_channels.node_short_name);
}

/**
 * Get current long name (never NULL, may be empty before ensure_identity).
 */
static inline const char *meshy_channels_long_name(void) {
    return g_meshy_channels.node_long_name;
}

/**
 * Get current short name (never NULL, may be empty before ensure_identity).
 */
static inline const char *meshy_channels_short_name(void) {
    return g_meshy_channels.node_short_name;
}

// ─── PKI Keypair (X25519) ──────────────────────────────────────────────────

/**
 * RNG callback for mbedtls — wraps ESP32-P4 hardware TRNG.
 * esp_fill_random() uses the digital oscillator jitter source,
 * which passes NIST SP 800-22 even without WiFi/BT RF noise.
 */
static int meshy_pki_rng(void *ctx, unsigned char *buf, size_t len) {
    (void)ctx;
    esp_fill_random(buf, len);
    return 0;
}

/**
 * Generate an X25519 keypair using mbedtls ECP with hardware RNG.
 * All mbedtls allocations are temporary (stack + internal heap) and freed before return.
 * Returns true on success.
 */
static inline bool meshy_pki_generate(uint8_t priv_out[32], uint8_t pub_out[32]) {
    mbedtls_ecp_group grp;
    mbedtls_mpi d;
    mbedtls_ecp_point Q;

    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);

    int ret;

    // Load Curve25519 group parameters
    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519);
    if (ret != 0) {
        ESP_LOGE("MESHY_PKI", "ecp_group_load failed: -0x%04x", (unsigned)-ret);
        goto done;
    }

    // Generate keypair: d = clamped random scalar, Q = d * basepoint
    // mbedtls_ecdh_gen_public handles RFC 7748 clamping internally
    ret = mbedtls_ecdh_gen_public(&grp, &d, &Q, meshy_pki_rng, NULL);
    if (ret != 0) {
        ESP_LOGE("MESHY_PKI", "ecdh_gen_public failed: -0x%04x", (unsigned)-ret);
        goto done;
    }

    // Export private key as 32-byte little-endian (X25519 wire format)
    ret = mbedtls_mpi_write_binary_le(&d, priv_out, 32);
    if (ret != 0) {
        ESP_LOGE("MESHY_PKI", "priv export failed: -0x%04x", (unsigned)-ret);
        goto done;
    }

    // Export public key as 32-byte little-endian X coordinate.
    // For Montgomery curves (Curve25519), ecp_point_write_binary outputs
    // just the raw 32-byte X coordinate — no format prefix byte.
    {
        size_t olen = 0;
        ret = mbedtls_ecp_point_write_binary(&grp, &Q, MBEDTLS_ECP_PF_COMPRESSED,
                                              &olen, pub_out, 32);
        if (ret != 0 || olen != 32) {
            ESP_LOGE("MESHY_PKI", "pub export failed: -0x%04x (olen=%u)", (unsigned)-ret, (unsigned)olen);
            if (ret == 0) ret = -1;  // olen mismatch
            goto done;
        }
    }

done:
    // Securely wipe private scalar from mbedtls heap
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);
    mbedtls_ecp_group_free(&grp);
    return ret == 0;
}

/**
 * Ensure PKI keypair exists. If not yet generated, create one and persist to NVS.
 * Call after meshy_channels_load(). Only runs keygen once per device lifetime.
 */
static inline void meshy_pki_ensure(void) {
    if (g_meshy_channels.pki_valid) {
        ESP_LOGI("MESHY_PKI", "X25519 keypair loaded from NVS");
        // Log first 4 bytes of public key for identification
        ESP_LOGI("MESHY_PKI", "Public key: %02x%02x%02x%02x...",
                 g_meshy_channels.pki_public_key[0], g_meshy_channels.pki_public_key[1],
                 g_meshy_channels.pki_public_key[2], g_meshy_channels.pki_public_key[3]);
        return;
    }

    ESP_LOGI("MESHY_PKI", "Generating X25519 keypair...");

    if (!meshy_pki_generate(g_meshy_channels.pki_private_key,
                             g_meshy_channels.pki_public_key)) {
        ESP_LOGE("MESHY_PKI", "Keypair generation failed!");
        return;
    }

    g_meshy_channels.pki_valid = 1;
    meshy_channels_save();

    ESP_LOGI("MESHY_PKI", "Keypair generated and saved to NVS");
    ESP_LOGI("MESHY_PKI", "Public key: %02x%02x%02x%02x...",
             g_meshy_channels.pki_public_key[0], g_meshy_channels.pki_public_key[1],
             g_meshy_channels.pki_public_key[2], g_meshy_channels.pki_public_key[3]);
}

/**
 * Get the 32-byte X25519 public key. Returns NULL if not yet generated.
 */
static inline const uint8_t *meshy_pki_public_key(void) {
    return g_meshy_channels.pki_valid ? g_meshy_channels.pki_public_key : NULL;
}

/**
 * Get the 32-byte X25519 private key. Returns NULL if not yet generated.
 * WARNING: Only use for DM decryption — never transmit.
 */
static inline const uint8_t *meshy_pki_private_key(void) {
    return g_meshy_channels.pki_valid ? g_meshy_channels.pki_private_key : NULL;
}

#ifdef __cplusplus
}
#endif
