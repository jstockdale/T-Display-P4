/**
 * aircraft_db.h — Aircraft database lookup for ADS-B Scope.
 *
 * Reads a bucket-indexed binary file from SD card (built by build_aircraft_db.py).
 * Loads a 512 KB bucket header into PSRAM at init, then does single-seek lookups
 * with a 512-entry PSRAM cache so each ICAO is only looked up once per session.
 *
 * File format:
 *   Header:  65536 × 8 bytes — { uint32_t offset; uint32_t count; } per bucket
 *   Data:    Variable-length records grouped by bucket (top 16 bits of ICAO)
 *            Record: icao_lo(1) + reg\0 + typecode\0 + mfr\0 + model\0 + owner\0 + opIcao\0
 *
 * Usage:
 *   aircraft_db_init();                          // background task loads header
 *   aircraft_db_entry_t entry;
 *   if (aircraft_db_lookup(0xA1099F, &entry)) {  // returns from cache or SD
 *       printf("Reg: %s  Type: %s\n", entry.reg, entry.typecode);
 *   }
 *
 * Part of ADS-B Scope — T-Display-P4.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// SD I/O mutex + DMA fence — protects SPI bus from AXI contention
extern bool sd_io_take(uint32_t timeout_ms);
extern void sd_io_give(void);

// ─── Result struct ──────────────────────────────────────────────────────────

typedef struct {
    uint32_t icao;
    char ac_class;      // L=landplane, H=helicopter, A=amphibian, G=gyrocopter, T=tilt-rotor, ?=unknown
    char reg[12];       // registration / tail number
    char typecode[8];   // ICAO type designator (B738, C172, etc.)
    char mfr[24];       // manufacturer name
    char model[28];     // model name
    char owner[28];     // registered owner
    char op_icao[8];    // operator ICAO code (UAL, DAL, etc.)
} aircraft_db_entry_t;

// ─── Public API ─────────────────────────────────────────────────────────────

/**
 * Start background loading of the aircraft database header.
 * Call after SD card is mounted. Non-blocking — spawns a one-shot task.
 * Lookups before loading completes gracefully return false.
 */
void aircraft_db_init(void);

/**
 * Look up an aircraft by ICAO24 address.
 * Returns true if found, populating *out. Returns false if not found,
 * database not loaded yet, or database file not present.
 * Thread-safe (internal mutex).
 */
bool aircraft_db_lookup(uint32_t icao, aircraft_db_entry_t *out);

/**
 * Check if the database header is loaded and ready for lookups.
 */
bool aircraft_db_ready(void);

/**
 * Get database stats for status display.
 */
void aircraft_db_stats(int *total_records, int *cache_entries, int *cache_hits, int *cache_misses);

#ifdef __cplusplus
}
#endif

// ═══════════════════════════════════════════════════════════════════════════
// Implementation (header-only, include from one .c/.cpp file)
// ═══════════════════════════════════════════════════════════════════════════

#ifdef AIRCRAFT_DB_IMPLEMENTATION

static const char *ACDB_TAG = "ACDB";

#define ACDB_PATH         "/sdcard/data/aircraft.db"
#define ACDB_MAGIC        "ADSBDB02"
#define ACDB_MAGIC_SIZE   8
#define ACDB_NUM_BUCKETS  65536
#define ACDB_HEADER_SIZE  (ACDB_NUM_BUCKETS * 8)  // 512 KB
#define ACDB_CACHE_SIZE   512
#define ACDB_BUCKET_BUF   20480  // 20 KB read buffer for bucket data

// ─── Bucket header (PSRAM) ──────────────────────────────────────────────────

typedef struct {
    uint32_t offset;
    uint32_t count;
} acdb_bucket_t;

static acdb_bucket_t *s_acdb_header = NULL;  // PSRAM, 512 KB
static volatile bool s_acdb_ready = false;
static SemaphoreHandle_t s_acdb_mutex = NULL;

// ─── Lookup cache (PSRAM) ───────────────────────────────────────────────────

typedef struct {
    uint32_t icao;              // 0 = empty slot
    aircraft_db_entry_t entry;
} acdb_cache_slot_t;

static acdb_cache_slot_t *s_acdb_cache = NULL;  // PSRAM
static int s_acdb_cache_hits = 0;
static int s_acdb_cache_misses = 0;
static int s_acdb_total_records = 0;

// ─── Helper: read a null-terminated string from buffer ──────────────────────

static int acdb_read_str(const uint8_t *buf, int buf_len, int pos, char *out, int out_size) {
    int i = 0;
    while (pos + i < buf_len && buf[pos + i] != '\0' && i < out_size - 1) {
        out[i] = (char)buf[pos + i];
        i++;
    }
    out[i] = '\0';
    // Skip past the null terminator in buffer
    while (pos + i < buf_len && buf[pos + i] != '\0') i++;  // overflow: skip excess
    if (pos + i < buf_len) i++;  // skip the \0
    return i;
}

// ─── Cache operations ───────────────────────────────────────────────────────

static acdb_cache_slot_t *acdb_cache_find(uint32_t icao) {
    if (!s_acdb_cache) return NULL;
    uint32_t idx = (icao * 2654435761U) % ACDB_CACHE_SIZE;  // Knuth multiplicative hash
    for (int probe = 0; probe < ACDB_CACHE_SIZE; probe++) {
        uint32_t slot = (idx + probe) % ACDB_CACHE_SIZE;
        if (s_acdb_cache[slot].icao == icao) return &s_acdb_cache[slot];
        if (s_acdb_cache[slot].icao == 0) return NULL;  // empty = not cached
    }
    return NULL;
}

static void acdb_cache_insert(uint32_t icao, const aircraft_db_entry_t *entry) {
    if (!s_acdb_cache) return;
    uint32_t idx = (icao * 2654435761U) % ACDB_CACHE_SIZE;
    for (int probe = 0; probe < ACDB_CACHE_SIZE; probe++) {
        uint32_t slot = (idx + probe) % ACDB_CACHE_SIZE;
        if (s_acdb_cache[slot].icao == 0 || s_acdb_cache[slot].icao == icao) {
            s_acdb_cache[slot].icao = icao;
            s_acdb_cache[slot].entry = *entry;
            return;
        }
    }
    // Table full — overwrite the target slot (rare with 512 slots and 256 aircraft)
    uint32_t slot = idx % ACDB_CACHE_SIZE;
    s_acdb_cache[slot].icao = icao;
    s_acdb_cache[slot].entry = *entry;
}

// Also cache "not found" so we don't re-seek for unknown ICAOs
#define ACDB_NOT_FOUND_ICAO 0xFFFFFFFF

static void acdb_cache_insert_miss(uint32_t icao) {
    aircraft_db_entry_t empty;
    memset(&empty, 0, sizeof(empty));
    empty.icao = ACDB_NOT_FOUND_ICAO;  // sentinel: looked up, not in DB
    acdb_cache_insert(icao, &empty);
}

// ─── SD card lookup ─────────────────────────────────────────────────────────

static bool acdb_sd_lookup(uint32_t icao, aircraft_db_entry_t *out) {
    uint32_t bucket_id = icao >> 8;
    uint8_t icao_lo = icao & 0xFF;

    if (!s_acdb_header) return false;
    acdb_bucket_t *bkt = &s_acdb_header[bucket_id];
    if (bkt->count == 0) return false;

    int read_size = bkt->count * 64;
    if (read_size > ACDB_BUCKET_BUF) read_size = ACDB_BUCKET_BUF;

    uint8_t *buf = (uint8_t *)heap_caps_malloc(read_size, MALLOC_CAP_SPIRAM);
    if (!buf) return false;

    // Hold sd_io only for the SD access — parse from PSRAM after release
    if (!sd_io_take(200)) { heap_caps_free(buf); return false; }
    FILE *f = fopen(ACDB_PATH, "rb");
    if (!f) { sd_io_give(); heap_caps_free(buf); return false; }
    fseek(f, bkt->offset, SEEK_SET);
    int got = fread(buf, 1, read_size, f);
    fclose(f);
    sd_io_give();

    // Linear scan for matching icao_lo
    // Record format v2: icao_lo(1) + ac_class(1) + 6 null-terminated strings
    int pos = 0;
    bool found = false;
    for (uint32_t i = 0; i < bkt->count && pos + 1 < got; i++) {
        uint8_t lo = buf[pos++];
        uint8_t ac_class = buf[pos++];

        // Parse 6 null-terminated strings
        char fields[6][28];
        int field_maxes[] = {
            (int)sizeof(out->reg), (int)sizeof(out->typecode),
            (int)sizeof(out->mfr), (int)sizeof(out->model),
            (int)sizeof(out->owner), (int)sizeof(out->op_icao)
        };
        for (int fi = 0; fi < 6; fi++) {
            pos += acdb_read_str(buf, got, pos, fields[fi], field_maxes[fi]);
        }

        if (lo == icao_lo) {
            out->icao = icao;
            out->ac_class = (char)ac_class;
            strncpy(out->reg, fields[0], sizeof(out->reg) - 1);
            out->reg[sizeof(out->reg) - 1] = '\0';
            strncpy(out->typecode, fields[1], sizeof(out->typecode) - 1);
            out->typecode[sizeof(out->typecode) - 1] = '\0';
            strncpy(out->mfr, fields[2], sizeof(out->mfr) - 1);
            out->mfr[sizeof(out->mfr) - 1] = '\0';
            strncpy(out->model, fields[3], sizeof(out->model) - 1);
            out->model[sizeof(out->model) - 1] = '\0';
            strncpy(out->owner, fields[4], sizeof(out->owner) - 1);
            out->owner[sizeof(out->owner) - 1] = '\0';
            strncpy(out->op_icao, fields[5], sizeof(out->op_icao) - 1);
            out->op_icao[sizeof(out->op_icao) - 1] = '\0';
            found = true;
            break;
        }
    }

    heap_caps_free(buf);
    return found;
}

// ─── Public API ─────────────────────────────────────────────────────────────

bool aircraft_db_ready(void) {
    return s_acdb_ready;
}

bool aircraft_db_lookup(uint32_t icao, aircraft_db_entry_t *out) {
    if (!s_acdb_ready || !s_acdb_mutex) return false;

    xSemaphoreTake(s_acdb_mutex, portMAX_DELAY);

    // Check cache first
    acdb_cache_slot_t *cached = acdb_cache_find(icao);
    if (cached) {
        if (cached->entry.icao == ACDB_NOT_FOUND_ICAO) {
            // Previously looked up, not in DB
            s_acdb_cache_hits++;
            xSemaphoreGive(s_acdb_mutex);
            return false;
        }
        *out = cached->entry;
        s_acdb_cache_hits++;
        xSemaphoreGive(s_acdb_mutex);
        return true;
    }

    // Cache miss — look up on SD
    s_acdb_cache_misses++;
    aircraft_db_entry_t entry;
    memset(&entry, 0, sizeof(entry));
    bool found = acdb_sd_lookup(icao, &entry);

    if (found) {
        acdb_cache_insert(icao, &entry);
        *out = entry;
    } else {
        acdb_cache_insert_miss(icao);
    }

    xSemaphoreGive(s_acdb_mutex);
    return found;
}

void aircraft_db_stats(int *total_records, int *cache_entries, int *cache_hits, int *cache_misses) {
    if (total_records) *total_records = s_acdb_total_records;
    if (cache_entries) {
        int count = 0;
        if (s_acdb_cache) {
            for (int i = 0; i < ACDB_CACHE_SIZE; i++)
                if (s_acdb_cache[i].icao != 0) count++;
        }
        *cache_entries = count;
    }
    if (cache_hits) *cache_hits = s_acdb_cache_hits;
    if (cache_misses) *cache_misses = s_acdb_cache_misses;
}

// ─── Background loader task ─────────────────────────────────────────────────

static void acdb_loader_task(void *arg) {
    int64_t t0 = esp_timer_get_time();

    // Allocate header in PSRAM (512 KB)
    s_acdb_header = (acdb_bucket_t *)heap_caps_malloc(ACDB_HEADER_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_acdb_header) {
        ESP_LOGW(ACDB_TAG, "Failed to allocate header (%d KB PSRAM)", ACDB_HEADER_SIZE / 1024);
        vTaskDelete(NULL);
        return;
    }

    // Allocate cache in PSRAM
    s_acdb_cache = (acdb_cache_slot_t *)heap_caps_calloc(
        ACDB_CACHE_SIZE, sizeof(acdb_cache_slot_t), MALLOC_CAP_SPIRAM);
    if (!s_acdb_cache) {
        ESP_LOGW(ACDB_TAG, "Failed to allocate cache");
        heap_caps_free(s_acdb_header);
        s_acdb_header = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Read header from SD — hold sd_io only for the file access
    if (!sd_io_take(5000)) {
        ESP_LOGW(ACDB_TAG, "SD busy — cannot load database");
        heap_caps_free(s_acdb_header);
        heap_caps_free(s_acdb_cache);
        s_acdb_header = NULL;
        s_acdb_cache = NULL;
        vTaskDelete(NULL);
        return;
    }

    FILE *f = fopen(ACDB_PATH, "rb");
    if (!f) {
        sd_io_give();
        ESP_LOGW(ACDB_TAG, "Database file not found: %s", ACDB_PATH);
        heap_caps_free(s_acdb_header);
        heap_caps_free(s_acdb_cache);
        s_acdb_header = NULL;
        s_acdb_cache = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Validate magic / version
    char magic[ACDB_MAGIC_SIZE + 1] = {0};
    if (fread(magic, 1, ACDB_MAGIC_SIZE, f) != ACDB_MAGIC_SIZE ||
        memcmp(magic, ACDB_MAGIC, ACDB_MAGIC_SIZE) != 0) {
        ESP_LOGE(ACDB_TAG, "Bad magic: \"%s\" (expected \"%s\") — wrong DB version?",
                 magic, ACDB_MAGIC);
        fclose(f);
        sd_io_give();
        heap_caps_free(s_acdb_header);
        heap_caps_free(s_acdb_cache);
        s_acdb_header = NULL;
        s_acdb_cache = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Read bucket header (immediately after magic)
    size_t read = fread(s_acdb_header, 1, ACDB_HEADER_SIZE, f);
    fclose(f);
    sd_io_give();

    if (read != ACDB_HEADER_SIZE) {
        ESP_LOGE(ACDB_TAG, "Header read: %u / %u bytes", (unsigned)read, ACDB_HEADER_SIZE);
        heap_caps_free(s_acdb_header);
        heap_caps_free(s_acdb_cache);
        s_acdb_header = NULL;
        s_acdb_cache = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Count total records
    int total = 0;
    for (int i = 0; i < ACDB_NUM_BUCKETS; i++)
        total += s_acdb_header[i].count;
    s_acdb_total_records = total;

    s_acdb_ready = true;

    int64_t dt = (esp_timer_get_time() - t0) / 1000;
    ESP_LOGI(ACDB_TAG, "Loaded %d records from %s (%lld ms, %d KB header)",
             total, ACDB_PATH, (long long)dt, ACDB_HEADER_SIZE / 1024);

    vTaskDelete(NULL);
}

void aircraft_db_init(void) {
    if (s_acdb_mutex) return;  // already initialized
    s_acdb_mutex = xSemaphoreCreateMutex();

    // Spawn background loader — self-deletes when done
    xTaskCreateWithCaps(acdb_loader_task, "acdb_load", 4096, NULL, 1,
                        NULL, MALLOC_CAP_SPIRAM);
}

#endif // AIRCRAFT_DB_IMPLEMENTATION
