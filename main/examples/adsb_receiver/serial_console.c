/*
 * serial_console.c – Interactive serial console for ADS-B receiver
 *
 * Architecture:
 *   - A FreeRTOS task polls stdin for input (non-blocking, 50ms timeout)
 *   - Two modes: LOG (streaming output) and CMD (interactive prompt)
 *   - Ctrl+C (0x03) switches from LOG → CMD at any time
 *   - "log" command switches from CMD → LOG
 *   - In CMD mode, ADS-B/GNSS output is suppressed
 *
 * Commands (CMD mode):
 *   help              – show available commands
 *   log               – return to streaming log mode
 *   ls [path]         – list directory (default: /sdcard)
 *   cat <file>        – print file contents to serial
 *   head <file> [n]   – print first n lines (default 20)
 *   tail <file> [n]   – print last n lines (default 20)
 *   rm <file>         – delete a file
 *   mv <src> <dst>    – rename/move a file
 *   cp <src> <dst>    – copy a file
 *   df                – show SD card free space
 *   status            – show receiver status (memory, uptime, GPS, aircraft count)
 *   version           – show firmware version and build info
 *   mount             – mount SD card
 *   unmount           – safely unmount SD card (aliases: eject)
 *   webapp            – dump built-in adsb_scope.htm to serial
 *   meshy_tx <text>   – send Meshtastic text broadcast (aliases: tx)
 *   nvs               – NVS management (list, dump, clear)
 *   reboot            – software reset
 */

#include "serial_console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "esp_vfs_fat.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_app_desc.h"

#include <math.h>

#include "class_driver.h"  // for adsb_get_receiver_pos, aircraft count
#include "tz_lookup.h"     // for timezone lookup and manual override
#include "device_settings.h"
#include "meshtastic_task.h"
#include "meshy_channels.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *CONSOLE_TAG = "CONSOLE";

// Embedded adsb_scope.htm — built into firmware via EMBED_TXTFILES in CMakeLists.txt
extern const uint8_t adsb_scope_htm_start[] asm("_binary_adsb_scope_htm_start");
extern const uint8_t adsb_scope_htm_end[]   asm("_binary_adsb_scope_htm_end");

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
typedef enum { MODE_LOG, MODE_CMD } console_mode_t;

static volatile console_mode_t s_mode = MODE_LOG;
static SemaphoreHandle_t s_mode_mutex = NULL;

#define CMD_BUF_SIZE 256
static char s_cmd_buf[CMD_BUF_SIZE];
static int  s_cmd_len = 0;

#define CTRL_C 0x03

// ---------------------------------------------------------------------------
// CMD-mode line buffer — stores log output produced while in CMD mode
// so the client gets caught up when returning to LOG mode.
// ---------------------------------------------------------------------------
#define REPLAY_MAX_LINES  500
#define REPLAY_LINE_MAX   384

static char **s_replay_buf = NULL;   // array of REPLAY_MAX_LINES char* pointers
static int    s_replay_head = 0;     // next write slot (circular)
static int    s_replay_count = 0;    // number of lines stored
static SemaphoreHandle_t s_replay_mutex = NULL;

static void replay_init(void) {
    if (!s_replay_buf) {
        s_replay_buf = heap_caps_calloc(REPLAY_MAX_LINES, sizeof(char *), MALLOC_CAP_SPIRAM);
    }
    if (!s_replay_mutex) {
        s_replay_mutex = xSemaphoreCreateMutex();
    }
}

static void replay_push(const char *line) {
    if (!s_replay_buf || !s_replay_mutex) return;
    xSemaphoreTake(s_replay_mutex, portMAX_DELAY);

    // Free old entry if slot is occupied (circular overwrite)
    if (s_replay_buf[s_replay_head]) {
        free(s_replay_buf[s_replay_head]);
        s_replay_buf[s_replay_head] = NULL;
    }

    // Allocate in PSRAM — truncate if too long
    int len = strlen(line);
    if (len > REPLAY_LINE_MAX - 1) len = REPLAY_LINE_MAX - 1;
    char *copy = heap_caps_malloc(len + 1, MALLOC_CAP_SPIRAM);
    if (copy) {
        memcpy(copy, line, len);
        copy[len] = '\0';
        s_replay_buf[s_replay_head] = copy;
    }

    s_replay_head = (s_replay_head + 1) % REPLAY_MAX_LINES;
    if (s_replay_count < REPLAY_MAX_LINES) s_replay_count++;

    xSemaphoreGive(s_replay_mutex);
}

static void replay_flush(void) {
    if (!s_replay_buf || !s_replay_mutex || s_replay_count == 0) return;
    xSemaphoreTake(s_replay_mutex, portMAX_DELAY);

    // Read from oldest to newest
    int start = (s_replay_count < REPLAY_MAX_LINES)
              ? 0
              : s_replay_head;  // oldest slot when buffer wrapped

    int printed = 0;
    for (int i = 0; i < s_replay_count; i++) {
        int idx = (start + i) % REPLAY_MAX_LINES;
        if (s_replay_buf[idx]) {
            fputs(s_replay_buf[idx], stdout);
            free(s_replay_buf[idx]);
            s_replay_buf[idx] = NULL;
            printed++;
        }
    }
    s_replay_head = 0;
    s_replay_count = 0;

    xSemaphoreGive(s_replay_mutex);

    if (printed > 0) {
        printf("[%d buffered lines replayed]\n", printed);
        fflush(stdout);
    }
}

// ---------------------------------------------------------------------------
// ESP_LOG gating — intercept all ESP_LOGI/W/E output in CMD mode
// ---------------------------------------------------------------------------
static vprintf_like_t s_original_log_vprintf = NULL;

static int gated_log_vprintf(const char *fmt, va_list ap) {
    if (s_mode == MODE_LOG) {
        return s_original_log_vprintf(fmt, ap);
    }
    // CMD mode — buffer to replay ring
    static char log_tmp[REPLAY_LINE_MAX];
    static SemaphoreHandle_t log_mutex = NULL;
    if (!log_mutex) log_mutex = xSemaphoreCreateMutex();
    if (log_mutex) xSemaphoreTake(log_mutex, portMAX_DELAY);
    vsnprintf(log_tmp, sizeof(log_tmp), fmt, ap);
    replay_push(log_tmp);
    if (log_mutex) xSemaphoreGive(log_mutex);
    return 0;
}

// ---------------------------------------------------------------------------
// Mode query (called from other tasks)
// ---------------------------------------------------------------------------
bool serial_console_log_enabled(void) {
    return s_mode == MODE_LOG;
}

bool serial_console_print(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    if (s_mode == MODE_LOG) {
        vprintf(fmt, ap);
        va_end(ap);
        return true;
    }

    // CMD mode — format into static buffer (avoids 384 bytes on caller's stack,
    // which matters for Meshy RX/TX tasks with tight 6KB internal RAM stacks)
    static char tmp[REPLAY_LINE_MAX];
    static SemaphoreHandle_t print_mutex = NULL;
    if (!print_mutex) print_mutex = xSemaphoreCreateMutex();
    if (print_mutex) xSemaphoreTake(print_mutex, portMAX_DELAY);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    replay_push(tmp);
    if (print_mutex) xSemaphoreGive(print_mutex);
    return false;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void print_prompt(void) {
    printf("\033[33madsb>\033[0m ");  // yellow prompt
    fflush(stdout);
}

// Resolve path: if it doesn't start with /, prepend /sdcard/
static void resolve_path(const char *input, char *out, size_t out_sz) {
    if (input[0] == '/') {
        snprintf(out, out_sz, "%s", input);
    } else {
        snprintf(out, out_sz, "/sdcard/%s", input);
    }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
static void cmd_help(void) {
    printf(
        "\n"
        "  \033[1mADS-B Receiver Serial Console\033[0m\n"
        "\n"
        "  \033[32mlog\033[0m               return to streaming log mode\n"
        "  \033[32mls\033[0m [path]         list directory (default: /sdcard)\n"
        "  \033[32mcat\033[0m <file>        print file contents\n"
        "  \033[32mhead\033[0m <file> [n]   print first n lines (default 20)\n"
        "  \033[32mtail\033[0m <file> [n]   print last n lines (default 20)\n"
        "  \033[32mrm\033[0m <file>         delete a file\n"
        "  \033[32mmv\033[0m <src> <dst>    rename/move a file\n"
        "  \033[32mcp\033[0m <src> <dst>    copy a file\n"
        "  \033[32mdf\033[0m                show SD card free space\n"
        "  \033[32mstatus\033[0m            show receiver status\n"
        "  \033[32mversion\033[0m           show firmware version\n"
        "  \033[32mtimezone\033[0m [auto|UTC±N] show or set timezone\n"
        "  \033[32mheartbeat\033[0m [on|off|5-255] show/set console heartbeat\n"
        "  \033[32mmount\033[0m             mount SD card\n"
        "  \033[32munmount\033[0m           safely unmount SD card\n"
        "  \033[32mwebapp\033[0m            dump built-in adsb_scope.htm to serial\n"
        "  \033[32mmeshy_tx\033[0m <text>   send Meshtastic text message (broadcast)\n"
        "  \033[32mchannels\033[0m [list|reset|add|remove]  manage Meshtastic channels\n"
        "  \033[32midentity\033[0m [<long> [<short>]]  show/set Meshy node identity\n"
        "  \033[32mnvs\033[0m [list|dump|clear] [device|meshy|pki|all]  NVS management\n"
        "  \033[32mreboot\033[0m            software reset\n"
        "\n"
        "  Ctrl+C to escape from log mode to command mode\n"
        "  Paths default to /sdcard/ if no leading /\n"
        "\n"
    );
}

static void cmd_ls(const char *path) {
    char resolved[128];
    resolve_path(path && path[0] ? path : "/sdcard", resolved, sizeof(resolved));

    DIR *dir = opendir(resolved);
    if (!dir) {
        printf("  error: cannot open %s: %s\n", resolved, strerror(errno));
        return;
    }

    printf("  \033[1m%s/\033[0m\n", resolved);

    struct dirent *entry;
    int count = 0;
    long long total_size = 0;

    while ((entry = readdir(dir)) != NULL) {
        char fpath[384];
        snprintf(fpath, sizeof(fpath), "%s/%s", resolved, entry->d_name);

        struct stat st;
        if (stat(fpath, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                printf("  \033[34m%-32s\033[0m  <dir>\n", entry->d_name);
            } else {
                const char *unit = "B";
                float size = st.st_size;
                if (size > 1024*1024) { size /= 1024*1024; unit = "MB"; }
                else if (size > 1024) { size /= 1024; unit = "KB"; }
                printf("  %-32s  %8.1f %s\n", entry->d_name, size, unit);
                total_size += st.st_size;
            }
            count++;
        } else {
            printf("  %-32s  (stat failed)\n", entry->d_name);
            count++;
        }
    }
    closedir(dir);

    const char *tunit = "B";
    float tsize = total_size;
    if (tsize > 1024*1024) { tsize /= 1024*1024; tunit = "MB"; }
    else if (tsize > 1024) { tsize /= 1024; tunit = "KB"; }
    printf("  ─── %d entries, %.1f %s total\n", count, tsize, tunit);
}

static void cmd_cat(const char *path) {
    if (!path || !path[0]) { printf("  usage: cat <file>\n"); return; }

    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));

    FILE *f = fopen(resolved, "r");
    if (!f) {
        printf("  error: cannot open %s: %s\n", resolved, strerror(errno));
        return;
    }

    char buf[4096];
    size_t n, total = 0;
    int chunks = 0;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        fwrite(buf, 1, n, stdout);
        total += n;
        // Yield every 8 chunks (~32KB) to let other tasks breathe
        if (++chunks % 8 == 0) {
            fflush(stdout);
            taskYIELD();
        }
    }
    fflush(stdout);
    fclose(f);
    printf("\n");
}

static void cmd_head(const char *path, int n) {
    if (!path || !path[0]) { printf("  usage: head <file> [n]\n"); return; }

    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));

    FILE *f = fopen(resolved, "r");
    if (!f) {
        printf("  error: cannot open %s: %s\n", resolved, strerror(errno));
        return;
    }

    char line[512];
    int count = 0;
    while (count < n && fgets(line, sizeof(line), f)) {
        fputs(line, stdout);
        count++;
    }
    fclose(f);
    if (count == n) printf("  ─── (showing first %d lines)\n", n);
}

static void cmd_tail(const char *path, int n) {
    if (!path || !path[0]) { printf("  usage: tail <file> [n]\n"); return; }

    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));

    FILE *f = fopen(resolved, "r");
    if (!f) {
        printf("  error: cannot open %s: %s\n", resolved, strerror(errno));
        return;
    }

    // Count total lines
    char line[512];
    int total = 0;
    while (fgets(line, sizeof(line), f)) total++;

    // Seek back and print last n
    rewind(f);
    int skip = total > n ? total - n : 0;
    int cur = 0;
    while (fgets(line, sizeof(line), f)) {
        if (cur >= skip) fputs(line, stdout);
        cur++;
    }
    fclose(f);
    if (skip > 0) printf("  ─── (showing last %d of %d lines)\n", n, total);
}

static void cmd_rm(const char *path) {
    if (!path || !path[0]) { printf("  usage: rm <file>\n"); return; }

    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));

    if (remove(resolved) == 0) {
        printf("  deleted %s\n", resolved);
    } else {
        printf("  error: %s: %s\n", resolved, strerror(errno));
    }
}

static void cmd_mv(const char *src, const char *dst) {
    if (!src || !src[0] || !dst || !dst[0]) { printf("  usage: mv <src> <dst>\n"); return; }

    char rsrc[128], rdst[128];
    resolve_path(src, rsrc, sizeof(rsrc));
    resolve_path(dst, rdst, sizeof(rdst));

    if (rename(rsrc, rdst) == 0) {
        printf("  %s → %s\n", rsrc, rdst);
    } else {
        printf("  error: %s\n", strerror(errno));
    }
}

static void cmd_cp(const char *src, const char *dst) {
    if (!src || !src[0] || !dst || !dst[0]) { printf("  usage: cp <src> <dst>\n"); return; }

    char rsrc[128], rdst[128];
    resolve_path(src, rsrc, sizeof(rsrc));
    resolve_path(dst, rdst, sizeof(rdst));

    FILE *fin = fopen(rsrc, "rb");
    if (!fin) { printf("  error: cannot open %s: %s\n", rsrc, strerror(errno)); return; }

    FILE *fout = fopen(rdst, "wb");
    if (!fout) { printf("  error: cannot create %s: %s\n", rdst, strerror(errno)); fclose(fin); return; }

    char buf[4096];
    size_t n, total = 0;
    int chunks = 0;
    while ((n = fread(buf, 1, sizeof(buf), fin)) > 0) {
        fwrite(buf, 1, n, fout);
        total += n;
        if (++chunks % 8 == 0) taskYIELD();
    }
    fclose(fin);
    fclose(fout);

    const char *unit = "B";
    float sz = total;
    if (sz > 1024*1024) { sz /= 1024*1024; unit = "MB"; }
    else if (sz > 1024) { sz /= 1024; unit = "KB"; }
    printf("  copied %.1f %s → %s\n", sz, unit, rdst);
}

static void cmd_df(void) {
    FATFS *fs;
    DWORD fre_clust;
    if (f_getfree("0:", &fre_clust, &fs) != FR_OK) {
        printf("  error: cannot stat /sdcard\n");
        return;
    }

    unsigned long long total = (unsigned long long)(fs->n_fatent - 2) * fs->csize * 512;
    unsigned long long free_bytes = (unsigned long long)fre_clust * fs->csize * 512;
    unsigned long long used = total - free_bytes;

    printf("  /sdcard:\n");
    printf("    Total: %llu MB\n", total / (1024*1024));
    printf("    Used:  %llu MB\n", used  / (1024*1024));
    printf("    Free:  %llu MB\n", free_bytes / (1024*1024));
}

static void cmd_status(void) {
    printf("\n");
    printf("  \033[1mADS-B Receiver Status\033[0m\n");
    printf("  ─────────────────────\n");

    // Uptime
    int64_t uptime_us = esp_timer_get_time();
    int secs = (int)(uptime_us / 1000000);
    int hrs = secs / 3600; int mins = (secs % 3600) / 60; secs %= 60;
    printf("  Uptime:    %02d:%02d:%02d\n", hrs, mins, secs);

    // Memory
    printf("  Internal:  %u bytes free\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    printf("  PSRAM:     %u bytes free\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // GPS
    receiver_pos_t rx = adsb_get_receiver_pos();
    if (rx.fix_valid) {
        printf("  GPS:       fix, %.4f%c %.4f%c, %d sats, HDOP %.1f\n",
            fabs(rx.lat), rx.lat >= 0 ? 'N' : 'S',
            fabs(rx.lon), rx.lon >= 0 ? 'E' : 'W',
            rx.sats, rx.hdop);
    } else {
        printf("  GPS:       no fix\n");
    }

    printf("\n");
}

static void cmd_version(void) {
    const esp_app_desc_t *app = esp_app_get_description();
    // Print in the same format as ESP-IDF boot log so the scope's
    // existing parser picks it up without any special handling.
    printf("App version:      %s\n", app->version);
    printf("Compile time:     %s %s\n", app->date, app->time);
    printf("IDF version:      %s\n", app->idf_ver);
}

// ---------------------------------------------------------------------------
// NVS management
// ---------------------------------------------------------------------------

static const char *nvs_ns_device = SETTINGS_NVS_NAMESPACE;
static const char *nvs_ns_meshy  = MESHY_CH_NVS_NAMESPACE;

static const char *nvs_resolve_namespace(const char *name) {
    if (!name) return NULL;
    if (strcmp(name, "device") == 0) return nvs_ns_device;
    if (strcmp(name, "meshy") == 0)  return nvs_ns_meshy;
    return NULL;
}

static void nvs_list_keys(const char *ns_name, const char *ns_label) {
    printf("  \033[1m%s\033[0m (namespace: \"%s\")\n", ns_label, ns_name);

    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find("nvs", ns_name, NVS_TYPE_ANY, &it);
    if (err == ESP_ERR_NVS_NOT_FOUND || !it) {
        printf("    (empty)\n");
        return;
    }
    if (err != ESP_OK) {
        printf("    error: 0x%x\n", err);
        return;
    }

    int count = 0;
    while (it) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        const char *type_str = "?";
        switch (info.type) {
            case NVS_TYPE_U8:   type_str = "u8";   break;
            case NVS_TYPE_I8:   type_str = "i8";   break;
            case NVS_TYPE_U16:  type_str = "u16";  break;
            case NVS_TYPE_I16:  type_str = "i16";  break;
            case NVS_TYPE_U32:  type_str = "u32";  break;
            case NVS_TYPE_I32:  type_str = "i32";  break;
            case NVS_TYPE_U64:  type_str = "u64";  break;
            case NVS_TYPE_I64:  type_str = "i64";  break;
            case NVS_TYPE_STR:  type_str = "str";  break;
            case NVS_TYPE_BLOB: type_str = "blob"; break;
            default: break;
        }
        printf("    %-16s  %s\n", info.key, type_str);
        count++;

        err = nvs_entry_next(&it);
        if (err != ESP_OK) break;
    }
    nvs_release_iterator(it);
    printf("    (%d key%s)\n", count, count == 1 ? "" : "s");
}

static void nvs_dump_blob(const char *ns_name, const char *ns_label, const char *key) {
    nvs_handle_t nvs;
    if (nvs_open(ns_name, NVS_READONLY, &nvs) != ESP_OK) {
        printf("  Failed to open namespace \"%s\"\n", ns_name);
        return;
    }

    size_t len = 0;
    if (nvs_get_blob(nvs, key, NULL, &len) != ESP_OK || len == 0) {
        printf("  No blob \"%s\" in %s\n", key, ns_label);
        nvs_close(nvs);
        return;
    }

    // Read onto stack (blobs are < 1KB)
    uint8_t buf[512];
    size_t read_len = len < sizeof(buf) ? len : sizeof(buf);
    if (nvs_get_blob(nvs, key, buf, &read_len) != ESP_OK) {
        printf("  Failed to read blob\n");
        nvs_close(nvs);
        return;
    }
    nvs_close(nvs);

    printf("  \033[1m%s\033[0m: \"%s\" (%zu bytes)\n", ns_label, key, read_len);
    for (size_t i = 0; i < read_len; i += 16) {
        printf("  %04x: ", (unsigned)i);
        for (size_t j = 0; j < 16; j++) {
            if (i + j < read_len)
                printf("%02x ", buf[i + j]);
            else
                printf("   ");
        }
        printf(" ");
        for (size_t j = 0; j < 16 && (i + j) < read_len; j++) {
            uint8_t c = buf[i + j];
            printf("%c", (c >= 0x20 && c < 0x7f) ? c : '.');
        }
        printf("\n");
    }
    if (len > sizeof(buf)) {
        printf("  ... (truncated, %zu total)\n", len);
    }
}

static void cmd_nvs(const char *subcmd, const char *target) {
    if (!subcmd) {
        printf(
            "\n"
            "  \033[1mNVS Management\033[0m\n"
            "\n"
            "  \033[32mnvs list\033[0m [device|meshy]   list keys in namespace(s)\n"
            "  \033[32mnvs dump\033[0m [device|meshy]   hex dump settings blob\n"
            "  \033[32mnvs clear device\033[0m          reset device settings to defaults\n"
            "  \033[32mnvs clear meshy\033[0m           clear Meshy identity + channels (keeps PKI)\n"
            "  \033[32mnvs clear pki\033[0m             clear PKI keypair (new key on reboot)\n"
            "  \033[32mnvs clear all\033[0m             factory reset everything\n"
            "\n"
            "  Namespaces:\n"
            "    device  = \"%s\"  (display, radio, timezone, etc.)\n"
            "    meshy   = \"%s\"  (identity, channels, PKI keypair)\n"
            "\n",
            nvs_ns_device, nvs_ns_meshy
        );
        return;
    }

    // ── nvs list ──
    if (strcmp(subcmd, "list") == 0) {
        if (!target || strcmp(target, "device") == 0) {
            nvs_list_keys(nvs_ns_device, "Device Settings");
        }
        if (!target || strcmp(target, "meshy") == 0) {
            nvs_list_keys(nvs_ns_meshy, "Meshy Settings");
        }
        if (target && !nvs_resolve_namespace(target)) {
            printf("  Unknown namespace: \"%s\" (use device or meshy)\n", target);
        }
        return;
    }

    // ── nvs dump ──
    if (strcmp(subcmd, "dump") == 0) {
        if (!target || strcmp(target, "device") == 0) {
            nvs_dump_blob(nvs_ns_device, "Device Settings", "cfg");
        }
        if (!target || strcmp(target, "meshy") == 0) {
            nvs_dump_blob(nvs_ns_meshy, "Meshy Settings", MESHY_CH_NVS_KEY);
        }
        if (target && !nvs_resolve_namespace(target)) {
            printf("  Unknown namespace: \"%s\" (use device or meshy)\n", target);
        }
        return;
    }

    // ── nvs clear ──
    if (strcmp(subcmd, "clear") == 0) {
        if (!target) {
            printf("  Usage: nvs clear [device|meshy|pki|all]\n");
            return;
        }

        if (strcmp(target, "device") == 0) {
            settings_reset();
            printf("  Device settings reset to defaults.\n");
            printf("  (takes effect immediately, saved on next settings_save_if_pending cycle)\n");

        } else if (strcmp(target, "meshy") == 0) {
            // Preserve PKI across the clear
            uint8_t saved_priv[32] = {0}, saved_pub[32] = {0};
            uint8_t saved_valid = 0;
            if (g_meshy_channels.pki_valid) {
                memcpy(saved_priv, g_meshy_channels.pki_private_key, 32);
                memcpy(saved_pub, g_meshy_channels.pki_public_key, 32);
                saved_valid = g_meshy_channels.pki_valid;
            }

            // Erase NVS
            nvs_handle_t nvs;
            if (nvs_open(nvs_ns_meshy, NVS_READWRITE, &nvs) == ESP_OK) {
                nvs_erase_all(nvs);
                nvs_commit(nvs);
                nvs_close(nvs);
            }

            // Rebuild with PKI preserved
            memset(&g_meshy_channels, 0, sizeof(g_meshy_channels));
            if (saved_valid) {
                memcpy(g_meshy_channels.pki_private_key, saved_priv, 32);
                memcpy(g_meshy_channels.pki_public_key, saved_pub, 32);
                g_meshy_channels.pki_valid = saved_valid;
            }
            memset(saved_priv, 0, sizeof(saved_priv));
            meshy_channels_save();
            printf("  Meshy identity + channels cleared (PKI keypair preserved).\n");
            printf("  New identity will be generated on reboot.\n");

        } else if (strcmp(target, "pki") == 0) {
            memset(g_meshy_channels.pki_private_key, 0, sizeof(g_meshy_channels.pki_private_key));
            memset(g_meshy_channels.pki_public_key, 0, sizeof(g_meshy_channels.pki_public_key));
            g_meshy_channels.pki_valid = 0;
            meshy_channels_save();
            printf("  PKI keypair cleared. New key generated on reboot.\n");
            printf("  Other Meshtastic nodes will need to re-learn your public key.\n");

        } else if (strcmp(target, "all") == 0) {
            // Device settings
            settings_reset();

            // Meshy: full erase
            nvs_handle_t nvs;
            if (nvs_open(nvs_ns_meshy, NVS_READWRITE, &nvs) == ESP_OK) {
                nvs_erase_all(nvs);
                nvs_commit(nvs);
                nvs_close(nvs);
            }
            memset(&g_meshy_channels, 0, sizeof(g_meshy_channels));
            printf("  Full factory reset. All settings, identity, and PKI cleared.\n");
            printf("  Reboot to apply: type 'reboot'\n");

        } else {
            printf("  Unknown target: \"%s\" (use device, meshy, pki, or all)\n", target);
        }
        return;
    }

    printf("  Unknown subcommand: \"%s\" (type 'nvs' for help)\n", subcmd);
}

// ---------------------------------------------------------------------------
// Command dispatcher
// ---------------------------------------------------------------------------
static void dispatch_command(char *line) {
    // Trim leading/trailing whitespace
    while (*line == ' ' || *line == '\t') line++;
    char *end = line + strlen(line) - 1;
    while (end > line && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) *end-- = '\0';

    if (line[0] == '\0') return;

    // Parse command and args
    char *cmd = line;
    char *arg1 = NULL, *arg2 = NULL;
    char *full_args = NULL;   // everything after cmd, unsplit
    char *arg_split = NULL;   // position of null byte between arg1/arg2 (restorable)

    char *sp = strchr(line, ' ');
    if (sp) {
        *sp = '\0';
        arg1 = sp + 1;
        while (*arg1 == ' ') arg1++;
        full_args = arg1;     // save before further splitting

        sp = strchr(arg1, ' ');
        if (sp) {
            arg_split = sp;   // save so we can restore for commands needing full text
            *sp = '\0';
            arg2 = sp + 1;
            while (*arg2 == ' ') arg2++;
            if (*arg2 == '\0') arg2 = NULL;
        }
        if (arg1[0] == '\0') arg1 = NULL;
    }

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        cmd_help();
    } else if (strcmp(cmd, "log") == 0) {
        printf("  Entering log mode. Press Ctrl+C to return.\n");
        s_mode = MODE_LOG;
        replay_flush();  // replay any ADS-B/GNSS lines buffered during CMD mode
        sd_log_print_status();  // announce current log file for scope
        return;  // Don't print prompt
    } else if (strcmp(cmd, "ls") == 0 || strcmp(cmd, "dir") == 0) {
        cmd_ls(arg1);
    } else if (strcmp(cmd, "cat") == 0) {
        cmd_cat(arg1);
    } else if (strcmp(cmd, "head") == 0) {
        int n = arg2 ? atoi(arg2) : 20;
        if (n <= 0) n = 20;
        cmd_head(arg1, n);
    } else if (strcmp(cmd, "tail") == 0) {
        int n = arg2 ? atoi(arg2) : 20;
        if (n <= 0) n = 20;
        cmd_tail(arg1, n);
    } else if (strcmp(cmd, "rm") == 0 || strcmp(cmd, "del") == 0) {
        cmd_rm(arg1);
    } else if (strcmp(cmd, "mv") == 0 || strcmp(cmd, "rename") == 0) {
        cmd_mv(arg1, arg2);
    } else if (strcmp(cmd, "cp") == 0 || strcmp(cmd, "copy") == 0) {
        cmd_cp(arg1, arg2);
    } else if (strcmp(cmd, "df") == 0) {
        cmd_df();
    } else if (strcmp(cmd, "status") == 0) {
        cmd_status();
    } else if (strcmp(cmd, "version") == 0 || strcmp(cmd, "ver") == 0) {
        cmd_version();
    } else if (strcmp(cmd, "mount") == 0) {
        printf("  Mounting SD card...\n");
        extern bool sd_remount(void);
        if (sd_remount()) {
            printf("  SD card mounted. Logging will resume on next ADS-B message.\n");
        } else {
            printf("  Failed to mount SD card. Check that a card is inserted.\n");
        }
    } else if (strcmp(cmd, "unmount") == 0 || strcmp(cmd, "eject") == 0) {
        printf("  Closing log and unmounting SD card...\n");
        extern void sd_safe_shutdown(void);
        sd_safe_shutdown();
        printf("  SD card safely unmounted. You can remove it now.\n");
        printf("  (Logging will stop until reboot)\n");
    } else if (strcmp(cmd, "timezone") == 0 || strcmp(cmd, "tz") == 0) {
        if (!arg1) {
            // Show current timezone status
            printf("  Mode: %s\n", g_settings.tz_auto ? "auto (GPS)" : "manual");
            if (!g_settings.tz_auto) {
                int16_t total = g_settings.tz_offset_h * 60 + g_settings.tz_offset_m;
                if (g_settings.dst_enabled) total += 60;
                printf("  Manual offset: %s\n", settings_format_offset(total, g_settings.dst_enabled));
            }
            receiver_pos_t rx = adsb_get_receiver_pos();
            if (rx.fix_valid) {
                struct timeval tv;
                gettimeofday(&tv, NULL);
                struct tm tm_utc;
                gmtime_r(&tv.tv_sec, &tm_utc);
                tz_result_t tz = tz_lookup(rx.lat, rx.lon,
                    tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
                printf("  GPS position: %.4f, %.4f\n", rx.lat, rx.lon);
                printf("  Auto offset:  %s\n", settings_format_offset(tz.total_offset_min, tz.dst_active));
                if (tz.has_dst)
                    printf("  DST:          %s (%+d min)\n", tz.dst_active ? "ACTIVE" : "inactive", tz.dst_offset_min);
                else
                    printf("  DST:          not observed\n");
            } else {
                printf("  No GPS fix — auto timezone unavailable\n");
            }
        } else if (strcmp(arg1, "auto") == 0) {
            g_settings.tz_auto = true;
            settings_apply_timezone();
            settings_save();
            printf("  Timezone set to automatic (GPS-based)\n");
        } else {
            // Parse UTC±N or UTC±N:MM — sets manual mode
            const char *p = arg1;
            if (strncasecmp(p, "UTC", 3) == 0) p += 3;
            int hours = 0, minutes = 0;
            char sign = '+';
            if (*p == '+' || *p == '-') { sign = *p; p++; }
            hours = atoi(p);
            const char *colon = strchr(p, ':');
            if (colon) minutes = atoi(colon + 1);
            int total = hours * 60 + minutes;
            if (sign == '-') total = -total;
            if (total < -720 || total > 840) {
                printf("  Invalid offset (range: UTC-12 to UTC+14)\n");
            } else {
                g_settings.tz_auto = false;
                g_settings.tz_offset_h = (int8_t)(total / 60);
                g_settings.tz_offset_m = (int8_t)(abs(total) % 60);
                if (total < 0 && g_settings.tz_offset_m > 0)
                    g_settings.tz_offset_m = -g_settings.tz_offset_m;
                g_settings.dst_enabled = false;
                settings_apply_timezone();
                settings_save();
                printf("  Timezone set to UTC%c%d", total >= 0 ? '+' : '-', abs(total) / 60);
                if (abs(total) % 60) printf(":%02d", abs(total) % 60);
                printf(" (manual, persisted)\n");
            }
        }
    } else if (strcmp(cmd, "heartbeat") == 0 || strcmp(cmd, "hb") == 0) {
        if (!arg1) {
            printf("  Heartbeat: %s, period: %ds\n",
                   g_settings.heartbeat_enabled ? "on" : "off",
                   g_settings.heartbeat_period_s);
        } else if (strcmp(arg1, "on") == 0) {
            g_settings.heartbeat_enabled = true;
            settings_save();
            printf("  Heartbeat enabled (%ds)\n", g_settings.heartbeat_period_s);
        } else if (strcmp(arg1, "off") == 0) {
            g_settings.heartbeat_enabled = false;
            settings_save();
            printf("  Heartbeat disabled\n");
        } else {
            int p = atoi(arg1);
            if (p >= 5 && p <= 255) {
                g_settings.heartbeat_period_s = (uint8_t)p;
                g_settings.heartbeat_enabled = true;
                settings_save();
                printf("  Heartbeat period set to %ds\n", p);
            } else {
                printf("  Usage: heartbeat [on|off|5-255]\n");
            }
        }
    } else if (strcmp(cmd, "webapp") == 0) {
        size_t len = adsb_scope_htm_end - adsb_scope_htm_start;
        printf("  Sending adsb_scope.htm (%zu bytes)...\n", len);
        const uint8_t *p = adsb_scope_htm_start;
        size_t remaining = len;
        while (remaining > 0) {
            size_t chunk = remaining > 4096 ? 4096 : remaining;
            fwrite(p, 1, chunk, stdout);
            p += chunk;
            remaining -= chunk;
            fflush(stdout);
            taskYIELD();
        }
        printf("\n");
    } else if (strcmp(cmd, "meshy_tx") == 0 || strcmp(cmd, "tx") == 0) {
        // Restore full text after command (arg split may have inserted NUL)
        if (arg_split) *arg_split = ' ';
        if (!full_args || !full_args[0]) {
            printf("  usage: meshy_tx <message text>\n");
        } else if (!meshy_send_text(full_args)) {
            printf("  TX failed (radio not running?)\n");
        } else {
            printf("  TX queued: \"%s\"\n", full_args);
        }
    } else if (strcmp(cmd, "channels") == 0) {
        if (!arg1 || strcmp(arg1, "list") == 0) {
            printf("  Ch 0: (default) PSK index 1, enabled\n");
            int n = meshy_channels_count();
            for (int i = 0; i < n; i++) {
                const meshy_channel_cfg_t *ch = meshy_channels_get(i);
                printf("  Ch %d: \"%s\" psk_len=%d %s\n", i + 1,
                       ch->name[0] ? ch->name : "(unnamed)",
                       ch->psk_len, ch->enabled ? "ON" : "OFF");
            }
            if (n == 0) printf("  (no extra channels configured)\n");
        } else if (strcmp(arg1, "reset") == 0) {
            meshy_channels_reset();
            printf("  All extra channels cleared\n");
            if (g_settings.meshy_enabled) meshy_restart();
        } else if (strcmp(arg1, "add") == 0) {
            // channels add <name> -- creates channel with default PSK (index 1)
            // For custom PSKs, use the webapp channel manager.
            if (!arg2) {
                printf("  usage: channels add <name>\n");
                printf("  Creates a named channel with default PSK. Use webapp for custom PSKs.\n");
            } else {
                int count = meshy_channels_count();
                if (count >= MESHY_CH_MAX_EXTRA) {
                    printf("  Channel table full (%d extra channels)\n", MESHY_CH_MAX_EXTRA);
                } else {
                    uint8_t default_psk = 1;
                    meshy_channels_set(count, arg2, &default_psk, 1, true);
                    meshy_channels_save();
                    printf("  Added channel %d: \"%s\" (default PSK)\n", count + 1, arg2);
                    if (g_settings.meshy_enabled) meshy_restart();
                }
            }
        } else if (strcmp(arg1, "remove") == 0) {
            if (!arg2) {
                printf("  usage: channels remove <index>  (1-based, 0=default cannot be removed)\n");
            } else {
                int idx = atoi(arg2) - 1;  // user gives 1-based, API is 0-based
                if (idx < 0 || idx >= meshy_channels_count()) {
                    printf("  Invalid index (have %d extra channels)\n", meshy_channels_count());
                } else {
                    const meshy_channel_cfg_t *ch = meshy_channels_get(idx);
                    printf("  Removing channel %d: \"%s\"\n", idx + 1,
                           ch && ch->name[0] ? ch->name : "(unnamed)");
                    meshy_channels_remove(idx);
                    meshy_channels_save();
                    if (g_settings.meshy_enabled) meshy_restart();
                }
            }
        } else {
            printf("  usage: channels [list|reset|add <name> <psk>|remove <idx>]\n");
        }
    } else if (strcmp(cmd, "identity") == 0) {
        if (!arg1) {
            // Show current identity
            printf("  long_name:  \"%s\"\n", meshy_channels_long_name());
            printf("  short_name: \"%s\"\n", meshy_channels_short_name());
        } else if (arg2) {
            // identity <long_name> <short_name>
            // Restore the split to get full long_name up to last space-separated token
            // Actually: arg1=long_name, arg2=short_name
            meshy_channels_set_identity(arg1, arg2);
            printf("  Identity set: \"%s\" (%s)\n", arg1, arg2);
        } else {
            // identity <long_name> — auto-derive short_name from first 4 chars
            char short_auto[5] = {0};
            strncpy(short_auto, arg1, 4);
            meshy_channels_set_identity(arg1, short_auto);
            printf("  Identity set: \"%s\" (%s)\n", arg1, short_auto);
        }
    } else if (strcmp(cmd, "nvs") == 0) {
        cmd_nvs(arg1, arg2);
    } else if (strcmp(cmd, "reboot") == 0) {
        printf("  Shutting down SD card...\n");
        extern void sd_safe_shutdown(void);
        sd_safe_shutdown();
        printf("  Rebooting...\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else {
        printf("  unknown command: '%s' (type 'help' for commands)\n", cmd);
    }

    print_prompt();
}

// ---------------------------------------------------------------------------
// Console task
// ---------------------------------------------------------------------------
static void serial_console_task(void *arg) {
    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(CONSOLE_TAG, "Serial console ready (Ctrl+C for command mode)");
    printf("[CONSOLE] Serial console ready\n");

    uint32_t heartbeat_next = esp_log_timestamp() + (g_settings.heartbeat_period_s * 1000);

    while (1) {
        uint8_t ch;
        int n = read(STDIN_FILENO, &ch, 1);

        if (n <= 0) {
            // Heartbeat in log mode
            if (s_mode == MODE_LOG && g_settings.heartbeat_enabled &&
                esp_log_timestamp() >= heartbeat_next) {
                printf("[CONSOLE] Serial console heartbeat\n");
                printf("[MEM] internal=%u largest=%u PSRAM=%u\n",
                       heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                       heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                       heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
                heartbeat_next = esp_log_timestamp() + (g_settings.heartbeat_period_s * 1000);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (s_mode == MODE_LOG) {
            // In log mode, only listen for Ctrl+C
            if (ch == CTRL_C) {
                s_mode = MODE_CMD;
                printf("\n\033[33m── Command mode ── (type 'help' for commands, 'log' to resume)\033[0m\n");
                s_cmd_len = 0;
                print_prompt();
            }
            // All other input ignored in log mode
            continue;
        }

        // CMD mode: line editing
        if (ch == CTRL_C) {
            // Ctrl+C in cmd mode: cancel current line
            printf("\n");
            s_cmd_len = 0;
            print_prompt();
        } else if (ch == '\r' || ch == '\n') {
            printf("\n");
            s_cmd_buf[s_cmd_len] = '\0';
            dispatch_command(s_cmd_buf);
            s_cmd_len = 0;
        } else if (ch == 0x7F || ch == 0x08) {
            // Backspace
            if (s_cmd_len > 0) {
                s_cmd_len--;
                printf("\b \b");
                fflush(stdout);
            }
        } else if (ch >= 0x20 && s_cmd_len < CMD_BUF_SIZE - 1) {
            // Printable character
            s_cmd_buf[s_cmd_len++] = (char)ch;
            putchar(ch);
            fflush(stdout);
        }
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void serial_console_init(void) {
    s_mode_mutex = xSemaphoreCreateMutex();
    replay_init();

    // Hook ESP_LOG output — gates all ESP_LOGI/W/E in CMD mode
    s_original_log_vprintf = esp_log_set_vprintf(gated_log_vprintf);

    // Console task does file I/O and stdin reads — no SPI/DMA, safe for PSRAM stack.
    // Must use PSRAM because internal RAM is exhausted by meshy SPI tasks.
    BaseType_t ret = xTaskCreateWithCaps(serial_console_task, "console", 8192, NULL, 2, NULL, MALLOC_CAP_SPIRAM);
    if (ret != pdPASS) {
        // Fallback to internal RAM (smaller stack)
        xTaskCreate(serial_console_task, "console", 4096, NULL, 2, NULL);
    }
}
