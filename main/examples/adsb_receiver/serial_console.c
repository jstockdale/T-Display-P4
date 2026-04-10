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
#include "mqtt_feeder.h"   // for mqtt_feeder_update_status
#include "tz_lookup.h"     // for timezone lookup and manual override
#include "device_settings.h"
#include "meshtastic_task.h"
#include "meshy_channels.h"
#include "sd_logger.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *CONSOLE_TAG = "CONSOLE";

// Shared device timestamp — same format as ADS-B log lines.
// "[2026-03-28 20:45:36.127Z] " after trusted time, "[boot+818.793] " before.
// Trusted = GPS, NTP, or PPS has synced (not just RTC boot seed).
extern bool time_manager_is_trusted_fn(void);
extern bool sd_io_take(uint32_t timeout_ms);
extern void sd_io_give(void);

int log_format_timestamp(char *buf, int bufsize) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (time_manager_is_trusted_fn()) {
        struct tm t;
        gmtime_r(&tv.tv_sec, &t);
        return snprintf(buf, bufsize, "[%04d-%02d-%02d %02d:%02d:%02d.%03ldZ] ",
            t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
            t.tm_hour, t.tm_min, t.tm_sec, (long)(tv.tv_usec / 1000));
    } else {
        int64_t boot_ms = esp_timer_get_time() / 1000;
        return snprintf(buf, bufsize, "[boot+%lld.%03lld] ",
            (long long)(boot_ms / 1000), (long long)(boot_ms % 1000));
    }
}

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

// Current working directory for file commands
static char s_cwd[128] = "/sdcard";

// Resolve path: absolute paths pass through, relative paths prepend cwd
static void resolve_path(const char *input, char *out, size_t out_sz) {
    if (input[0] == '/') {
        snprintf(out, out_sz, "%s", input);
    } else {
        // Append to cwd
        if (strcmp(s_cwd, "/") == 0)
            snprintf(out, out_sz, "/%s", input);
        else
            snprintf(out, out_sz, "%s/%s", s_cwd, input);
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
        "  \033[33m── Modes ──\033[0m\n"
        "  \033[32mlog\033[0m               return to streaming log mode\n"
        "\n"
        "  \033[33m── Filesystem ──\033[0m\n"
        "  \033[32mls\033[0m [path]         list directory (default: cwd)\n"
        "  \033[32mcat\033[0m <file>        print file contents\n"
        "  \033[32mhead\033[0m <file> [n]   print first n lines (default 20)\n"
        "  \033[32mtail\033[0m <file> [n]   print last n lines (default 20)\n"
        "  \033[32mrm\033[0m <file>         delete a file\n"
        "  \033[32mmv\033[0m <src> <dst>    rename/move a file\n"
        "  \033[32mcp\033[0m <src> <dst>    copy a file\n"
        "  \033[32mcd\033[0m [path]         change directory (default: /sdcard)\n"
        "  \033[32mpwd\033[0m               print working directory\n"
        "  \033[32mmkdir\033[0m <path>      create directory\n"
        "  \033[32mrmdir\033[0m <path>      remove empty directory\n"
        "  \033[32mdf\033[0m                show SD card free space\n"
        "  \033[32mmount\033[0m             mount SD card\n"
        "  \033[32munmount\033[0m           safely unmount SD card\n"
        "\n"
        "  \033[33m── Services ──\033[0m\n"
        "  \033[32madsb\033[0m [on|off|status]  ADS-B receiver control\n"
        "  \033[32mmeshy\033[0m [on|off|status] Meshtastic radio control\n"
        "  \033[32musb\033[0m [status]          USB host status\n"
        "  \033[32mwifi\033[0m [on|off|status|ssid <n>|pass <pw>]  WiFi control\n"
        "  \033[32mmqtt\033[0m [on|off|status|server <h>|port <n>|user <u>|pass <p>|id <id>]\n"
        "\n"
        "  \033[33m── Info ──\033[0m\n"
        "  \033[32mstatus\033[0m            show receiver status\n"
        "  \033[32mversion\033[0m           show firmware version\n"
        "  \033[32mtime\033[0m              show time source status\n"
        "  \033[32mgain\033[0m [auto|<dB>|band [<low> <high>]|restart]  tuner gain control\n"
        "\n"
        "  \033[33m── Config ──\033[0m\n"
        "  \033[32mtimezone\033[0m [auto|UTC±N] show or set timezone\n"
        "  \033[32mheartbeat\033[0m [on|off|5-255] show/set console heartbeat\n"
        "  \033[32mntp\033[0m [server] [poll_s]  show/set NTP config\n"
        "  \033[32mgps\033[0m [basic|strict]  show/set GPS time integrity mode\n"
        "\n"
        "  \033[33m── Meshtastic ──\033[0m\n"
        "  \033[32mmeshy_tx\033[0m <text>   send text message (broadcast)\n"
        "  \033[32mchannels\033[0m [list|reset|add|remove]  manage channels\n"
        "  \033[32midentity\033[0m [<long> [<short>]]  show/set node identity\n"
        "\n"
        "  \033[33m── Advanced ──\033[0m\n"
        "  \033[32mnvs\033[0m [list|dump|clear] [device|meshy|pki|all]  NVS management\n"
        "  \033[32mmsc\033[0m               enter USB Mass Storage mode (stops logging)\n"
        "  \033[32mcdc\033[0m               exit USB Mass Storage mode (resumes logging)\n"
        "  \033[32mwebapp\033[0m            dump built-in adsb_scope.htm to serial\n"
        "  \033[32mscreenshot\033[0m        save screen as PNG to SD card\n"
        "  \033[32mscreenshot send\033[0m   send last screenshot as base64 over serial\n"
        "  \033[32mrecv\033[0m <path> <size>  receive file via base64 (used by webapp)\n"
        "  \033[32mverify_crc32\033[0m <path> <hex>  verify file CRC32 on SD card\n"
        "  \033[32mstat\033[0m <path>         show file size (for webapp pre-check)\n"
        "  \033[32mreboot\033[0m            software reset\n"
        "\n"
        "  Ctrl+C to escape from log mode to command mode\n"
        "  Relative paths resolve from cwd (default: /sdcard)\n"
        "\n"
    );
}

static void cmd_ls(const char *path) {
    char resolved[128];
    resolve_path(path && path[0] ? path : s_cwd, resolved, sizeof(resolved));

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
            vTaskDelay(1);  // must be ≥1 tick to let IDLE feed WDT
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
        if (++chunks % 8 == 0) vTaskDelay(1);
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

static void cmd_cd(const char *path) {
    if (!path || path[0] == '\0') {
        // cd with no args → go to /sdcard
        strncpy(s_cwd, "/sdcard", sizeof(s_cwd));
        printf("  %s\n", s_cwd);
        return;
    }
    char resolved[128];
    if (strcmp(path, "..") == 0) {
        // Go up one level
        char *last = strrchr(s_cwd, '/');
        if (last && last != s_cwd) {
            *last = '\0';
        } else {
            strncpy(s_cwd, "/", sizeof(s_cwd));
        }
        printf("  %s\n", s_cwd);
        return;
    }
    resolve_path(path, resolved, sizeof(resolved));
    struct stat st;
    if (stat(resolved, &st) != 0) {
        printf("  no such directory: %s\n", resolved);
        return;
    }
    if (!S_ISDIR(st.st_mode)) {
        printf("  not a directory: %s\n", resolved);
        return;
    }
    strncpy(s_cwd, resolved, sizeof(s_cwd));
    s_cwd[sizeof(s_cwd) - 1] = '\0';
    // Strip trailing slash (unless root)
    size_t len = strlen(s_cwd);
    if (len > 1 && s_cwd[len - 1] == '/') s_cwd[len - 1] = '\0';
    printf("  %s\n", s_cwd);
}

static void cmd_pwd(void) {
    printf("  %s\n", s_cwd);
}

static void cmd_mkdir(const char *path) {
    if (!path) { printf("  usage: mkdir <path>\n"); return; }
    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));
    if (mkdir(resolved, 0755) == 0) {
        printf("  created %s\n", resolved);
    } else {
        printf("  error: %s (%s)\n", resolved, strerror(errno));
    }
}

static void cmd_rmdir(const char *path) {
    if (!path) { printf("  usage: rmdir <path>\n"); return; }
    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));
    if (rmdir(resolved) == 0) {
        printf("  removed %s\n", resolved);
    } else {
        printf("  error: %s (%s)\n", resolved, strerror(errno));
    }
}

// ---------------------------------------------------------------------------
// Service control commands
// ---------------------------------------------------------------------------

static void cmd_adsb(const char *subcmd) {
    extern adsb_stats_t adsb_get_stats(void);
    adsb_stats_t st = adsb_get_stats();

    if (!subcmd || strcmp(subcmd, "status") == 0) {
        extern bool adsb_is_stopped(void);
        bool host_stopped = adsb_is_stopped();
        printf("  ADS-B:     %s\n", st.rtlsdr_connected ? "running" :
                                     host_stopped ? "stopped (USB host down)" :
                                     st.rtlsdr_error ? "error" : "waiting for dongle");
        if (st.rtlsdr_connected) {
            printf("  Dongle:    %s\n", st.dongle_model);
            printf("  Gain:      %.1f dB (%s)\n", st.gain_tenths / 10.0,
                   st.gain_auto ? "adaptive" : "manual");
            printf("  Aircraft:  %d active\n", st.active_aircraft);
            printf("  Messages:  %lu total, %.1f/s\n",
                   (unsigned long)st.total_messages, st.msg_rate);
            printf("  Positions: %.1f/s\n", st.pos_rate);
            printf("  CRC error: %.1f%%\n", st.crc_error_rate * 100.0);
            if (st.farthest_dist_nm > 0)
                printf("  Max range: %.1f nm (%06lX)\n",
                       st.farthest_dist_nm, (unsigned long)st.farthest_icao);
            extern bool sd_log_is_active(void);
            printf("  SD log:    %s%s\n",
                   sd_log_is_active() ? "active" : "inactive",
                   sd_log_bus_failed() ? " *** SPI BUS FAILED ***" : "");
        }
    } else if (strcmp(subcmd, "on") == 0) {
        extern bool adsb_is_stopped(void);
        if (!adsb_is_stopped()) {
            printf("  ADS-B USB host is already running\n");
        } else {
            printf("  Starting ADS-B USB host...\n");
            extern void adsb_start_cmd(void);
            adsb_start_cmd();
            printf("  ADS-B started — waiting for dongle\n");
        }
    } else if (strcmp(subcmd, "off") == 0) {
        extern bool adsb_is_stopped(void);
        if (adsb_is_stopped()) {
            printf("  ADS-B USB host is not running\n");
        } else {
            printf("  Stopping ADS-B USB host...\n");
            extern void adsb_stop_cmd(void);
            adsb_stop_cmd();
            printf("  ADS-B stopped. USB host shut down.\n");
        }
    } else {
        printf("  usage: adsb [on|off|status]\n");
    }
}

static void cmd_meshy(const char *subcmd) {
    extern meshy_stats_t meshy_get_stats(void);
    meshy_stats_t st = meshy_get_stats();

    if (!subcmd || strcmp(subcmd, "status") == 0) {
        printf("  Meshy:     %s\n", st.running ? "running" : "stopped");
        if (st.running) {
            printf("  Frequency: %.3f MHz\n", st.freq_mhz);
            printf("  Nodes:     %lu known\n", (unsigned long)st.known_nodes);
            printf("  RX:        %lu packets (%lu decoded)\n",
                   (unsigned long)st.rx_packets, (unsigned long)st.rx_decoded);
            printf("  TX:        %lu packets\n", (unsigned long)st.tx_packets);
            extern bool meshy_sd_is_active(void);
            printf("  SD log:    %s%s\n",
                   meshy_sd_is_active() ? "active" : "inactive",
                   sd_log_bus_failed() ? " *** SPI BUS FAILED ***" : "");
        }
    } else if (strcmp(subcmd, "on") == 0) {
        if (st.running) {
            printf("  Meshy is already running\n");
        } else {
            printf("  Starting Meshtastic radio...\n");
            extern bool meshy_start(void);
            if (meshy_start()) {
                printf("  Meshy started\n");
            } else {
                printf("  Failed to start Meshy — check SX1262 init\n");
            }
        }
    } else if (strcmp(subcmd, "off") == 0) {
        if (!st.running) {
            printf("  Meshy is not running\n");
        } else {
            printf("  Stopping Meshtastic radio...\n");
            extern void meshy_stop(void);
            meshy_stop();
            printf("  Meshy stopped\n");
        }
    } else {
        printf("  usage: meshy [on|off|status]\n");
    }
}

static void cmd_usb(const char *subcmd) {
    if (!subcmd || strcmp(subcmd, "status") == 0) {
        extern adsb_stats_t adsb_get_stats(void);
        extern bool adsb_is_stopped(void);
        adsb_stats_t st = adsb_get_stats();
        bool host_stopped = adsb_is_stopped();
        printf("  USB host:  %s\n", host_stopped ? "stopped" :
                                     st.rtlsdr_connected ? "device connected" :
                                     st.rtlsdr_error ? "error" : "running (no device)");
        if (st.rtlsdr_connected) {
            printf("  Device:    %s\n", st.dongle_model);
        }
        extern bool sd_msc_is_active_fn(void);
        printf("  MSC mode:  %s\n", sd_msc_is_active_fn() ? "active" : "inactive");
    } else {
        printf("  usage: usb [status]\n");
        printf("  (usb reset/power-cycle coming soon)\n");
    }
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
    printf("  Internal:  %u bytes free (largest=%u, min_ever=%u)\n",
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
        heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    printf("  DMA:       %u bytes free (largest=%u, min_ever=%u)\n",
        heap_caps_get_free_size(MALLOC_CAP_DMA),
        heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
        heap_caps_get_minimum_free_size(MALLOC_CAP_DMA));
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

    // RTL-SDR dongle
    adsb_stats_t st = adsb_get_stats();
    printf("  RTL-SDR:   %s\n", st.dongle_model);

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
// File receive via base64 (used by webapp to upload files to SD card)
// ---------------------------------------------------------------------------
// Protocol (per chunk):
//   Webapp sends: recv <path> <total_size>
//   Firmware:     RECV_READY <current_offset>
//   Webapp sends: base64 lines (76 chars each)
//   Webapp sends: empty line (end of base64 data)
//   Webapp sends: CHUNK_CRC32 <hex>\n
//   Firmware:     RECV_CHUNK <new_offset>   (chunk OK)
//           or:   RECV_DONE <final_offset>  (transfer complete)
//           or:   RECV_ERR crc_mismatch ... (webapp retries)
//           or:   RECV_ERR <description>    (fatal)
//   Webapp sends: log\n                     (return to log mode)
//
// sd_io (SPI bus mutex) is held ONLY during file I/O — not while blocking
// on stdin.  DMA fence applied around all SD operations (#18235).
// Entire chunk is buffered in PSRAM and CRC32-verified before writing.
// ---------------------------------------------------------------------------

// ── CRC32 (IEEE 802.3, polynomial 0xEDB88320) ───────────────────────

static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (crc & 1 ? 0xEDB88320 : 0);
    }
    return crc;
}
#define CRC32_INIT      0xFFFFFFFF
#define CRC32_FINAL(c)  ((c) ^ 0xFFFFFFFF)

// ── Base64 decode ────────────────────────────────────────────────────

static const int8_t b64d[256] = {
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63,
    52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1,
    -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,
    15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1,
    -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
    41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
};

// Decode base64 string, returns decoded byte count.
static size_t base64_decode(const char *in, size_t in_len, uint8_t *out) {
    size_t o = 0;
    for (size_t i = 0; i + 3 < in_len; i += 4) {
        int8_t a = b64d[(uint8_t)in[i]];
        int8_t b = b64d[(uint8_t)in[i+1]];
        int8_t c = b64d[(uint8_t)in[i+2]];
        int8_t d = b64d[(uint8_t)in[i+3]];
        if (a < 0 || b < 0) break;
        out[o++] = (a << 2) | (b >> 4);
        if (c >= 0) out[o++] = ((b & 0xF) << 4) | (c >> 2);
        if (d >= 0) out[o++] = ((c & 0x3) << 6) | d;
    }
    return o;
}

// ── Helpers ──────────────────────────────────────────────────────────

// Create parent directories recursively (like mkdir -p).
static void mkdir_p(const char *path) {
    char tmp[128];
    strncpy(tmp, path, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    char *last_slash = strrchr(tmp, '/');
    if (!last_slash || last_slash == tmp) return;
    *last_slash = '\0';
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    mkdir(tmp, 0755);
}

// ── Buffered serial reader for recv ──────────────────────────────────
// Reads from stdin in large batches (512 bytes) to drain the USB FIFO
// fast enough to prevent overflow at USB-JTAG speeds (12 Mbps+).
// Reading 1 byte at a time with processing between reads can't keep up;
// the USB RX FIFO overflows and bytes are silently lost, causing CRC
// mismatches on large transfers.
//
// The read buffer persists across recv_read_line() calls so data
// spanning line boundaries is not lost.

// ── Buffered serial reader for recv ──────────────────────────────────
// Debug counters for transfer diagnostics
static int      s_recv_spins = 0;      // times we spun (data recent)
static int      s_recv_yields = 0;     // times we yielded (no data)
static int      s_recv_reads = 0;      // successful read() calls
static int      s_recv_max_gap_us = 0; // longest gap between reads

#define RECV_RX_BUF_SIZE  512
static uint8_t  s_recv_rx_buf[RECV_RX_BUF_SIZE];
static int      s_recv_rx_pos = 0;   // next byte to consume
static int      s_recv_rx_len = 0;   // valid bytes in buffer

// Reset the read buffer (call at start of each recv command)
static void recv_rx_reset(void) {
    s_recv_rx_pos = 0;
    s_recv_rx_len = 0;
}

// Read one line from stdin with timeout.  Returns line length (0 for
// empty line), -1 for timeout.  Strips CR, terminates on LF.
// Yields periodically to prevent task watchdog starvation.
#define RECV_TIMEOUT_US  10000000LL   // 10s no-data timeout
static int recv_read_line(char *buf, int buf_size, int64_t timeout_us) {
    int pos = 0;
    int64_t last_data = esp_timer_get_time();

    while (1) {
        // Refill batch buffer if empty
        if (s_recv_rx_pos >= s_recv_rx_len) {
            int n = read(STDIN_FILENO, s_recv_rx_buf, RECV_RX_BUF_SIZE);
            if (n <= 0) {
                if ((esp_timer_get_time() - last_data) > timeout_us) return -1;
                // Log actual read errors (not just "no data available")
                if (n < 0 && errno != EAGAIN && errno != 0) {
                    printf("[recv] read() err: n=%d errno=%d (%s)\n",
                           n, errno, strerror(errno));
                    fflush(stdout);
                }
                // If we received data recently (within 125ms), spin —
                // the ring buffer refills in microseconds during active
                // USB transfer.  A full 32KB chunk transfers in ~35ms.
                // 125ms covers chunk + JS event loop gaps between batches.
                if ((esp_timer_get_time() - last_data) < 125000) {
                    s_recv_spins++;
                    continue;  // spin — data flowing
                }
                s_recv_yields++;
                vTaskDelay(1);  // yield — sender paused
                continue;
            }
            s_recv_reads++;
            int gap = (int)(esp_timer_get_time() - last_data);
            if (gap > s_recv_max_gap_us) s_recv_max_gap_us = gap;
            s_recv_rx_pos = 0;
            s_recv_rx_len = n;
            last_data = esp_timer_get_time();
        }

        // Process bytes from the batch buffer
        while (s_recv_rx_pos < s_recv_rx_len) {
            uint8_t ch = s_recv_rx_buf[s_recv_rx_pos++];

            if (ch == '\r') continue;
            if (ch == '\n') {
                buf[pos] = '\0';
                return pos;
            }
            if (pos < buf_size - 1) {
                buf[pos++] = (char)ch;
            }
        }
    }
}

// ── Chunk buffer (PSRAM, lazy allocated) ─────────────────────────────

#define RECV_CHUNK_BUF_SIZE  (128 * 1024)  // 128KB — room for ~96KB chunks
static uint8_t *s_recv_chunk_buf = NULL;

// Persistent state for resume across chunks
static char    s_recv_path[128] = {0};
static size_t  s_recv_total = 0;

// ── recv command ─────────────────────────────────────────────────────

static void cmd_recv(const char *path, const char *size_str) {
    if (!path || !size_str) {
        printf("  usage: recv <path> <total_size>\n");
        return;
    }

    // Clear any stale data in the batch read buffer from prior commands
    recv_rx_reset();

    // Lazy-allocate chunk buffer in PSRAM
    if (!s_recv_chunk_buf) {
        s_recv_chunk_buf = (uint8_t *)heap_caps_malloc(
            RECV_CHUNK_BUF_SIZE, MALLOC_CAP_SPIRAM);
        if (!s_recv_chunk_buf) {
            printf("RECV_ERR out of memory (need %d bytes PSRAM)\n",
                   RECV_CHUNK_BUF_SIZE);
            return;
        }
    }

    // Resolve path
    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));

    size_t total = (size_t)strtoul(size_str, NULL, 10);
    if (total == 0) {
        printf("RECV_ERR invalid size\n");
        return;
    }

    strncpy(s_recv_path, resolved, sizeof(s_recv_path) - 1);
    s_recv_path[sizeof(s_recv_path) - 1] = '\0';
    s_recv_total = total;

    // Create parent directories
    if (!sd_io_take(5000)) { printf("RECV_ERR sd card busy\n"); return; }
    mkdir_p(resolved);

    // Determine offset from actual file on disk (survives reboots)
    struct stat st;
    size_t offset = 0;
    if (stat(resolved, &st) == 0 && st.st_size > 0 && (size_t)st.st_size < total) {
        offset = (size_t)st.st_size;  // resume
    }

    // Open file
    FILE *f;
    if (offset > 0) {
        f = fopen(resolved, "ab");  // append for resume
    } else {
        f = fopen(resolved, "wb");  // truncate for new/re-upload
    }
    sd_io_give();

    if (!f) {
        int e = errno;
        printf("RECV_ERR cannot open %s: %s\n", resolved, strerror(e));
        return;
    }

    // Flush all pending TX before signaling ready.  The USB Serial JTAG
    // shares one endpoint for TX and RX — heavy TX output (e.g. 142
    // buffered log lines replayed on Ctrl+C) can starve RX handling in
    // the ISR, causing incoming base64 data to be dropped.  A brief
    // delay after fflush lets the USB TX buffer drain completely.
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Drain any stale RX bytes (echoed chars, newlines) before
    // signaling ready — ensures Phase 1 sees clean base64 data.
    recv_rx_reset();
    {
        char drain_buf[128];
        while (read(STDIN_FILENO, drain_buf, sizeof(drain_buf)) > 0) {}
    }

    // Hold sd_io through the entire serial transfer (Phase 1 + Phase 2).
    // This doesn't protect SD I/O — it prevents other tasks (ADS-B,
    // Meshy) from doing 20-30ms SD flushes that preempt our read loop
    // long enough to overflow the 12KB USB ring buffer.  Their data
    // accumulates safely in RAM buffers and flushes between chunks.
    bool held_sd_io = sd_io_take(5000);

    // Temporarily raise our priority to 5 — above USB data path (4) and
    // all peripheral tasks.  Console is pinned to core 1 (see init below)
    // so this doesn't preempt usb_host_lib or class_driver on core 0 —
    // they keep full-speed ADS-B decoding during the transfer.
    // Restore after Phase 2.
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    UBaseType_t saved_prio = uxTaskPriorityGet(self);
    vTaskPrioritySet(self, 5);

    printf("RECV_READY %lu\n", (unsigned long)offset);
    fflush(stdout);

    // ── Phase 0: Optional CHUNK header ──
    // v1.1 senders send "CHUNK <expected_bytes>\n" before base64 data.
    // v1 senders send base64 immediately.  We detect which by checking
    // if the first line starts with "CHUNK " — base64 lines never do
    // (space is not in the base64 alphabet).
    static char line_buf[128];
    static uint8_t decode_buf[96];  // 76 base64 chars → max 57 bytes
    int32_t expected_chunk_size = -1;  // -1 = not declared (v1 sender)
    size_t chunk_pos = 0;
    int line_count = 0;
    bool error = false;
    bool reached_empty_line = false;
    s_recv_spins = 0;
    s_recv_yields = 0;
    s_recv_reads = 0;
    s_recv_max_gap_us = 0;
    int64_t phase1_start = esp_timer_get_time();

    // Read first line — could be CHUNK header or first base64 line
    int first_len = recv_read_line(line_buf, sizeof(line_buf), RECV_TIMEOUT_US);
    if (first_len < 0) {
        printf("RECV_ERR timeout\n");
        fflush(stdout);
        error = true;
    } else if (first_len == 0) {
        // Empty chunk — no data at all
        reached_empty_line = true;
    } else if (strncmp(line_buf, "CHUNK ", 6) == 0) {
        // v1.1 header: parse expected decoded byte count
        expected_chunk_size = (int32_t)strtoul(line_buf + 6, NULL, 10);
        if (expected_chunk_size <= 0 || expected_chunk_size > (int32_t)RECV_CHUNK_BUF_SIZE) {
            printf("RECV_ERR bad CHUNK size %ld\n", (long)expected_chunk_size);
            fflush(stdout);
            expected_chunk_size = -1;  // ignore, proceed as v1
        }
    } else {
        // v1 sender — first line is base64 data, decode it now
        line_count++;
        size_t decoded = base64_decode(line_buf, first_len, decode_buf);
        if (decoded > 0 && chunk_pos + decoded <= RECV_CHUNK_BUF_SIZE) {
            memcpy(s_recv_chunk_buf + chunk_pos, decode_buf, decoded);
            chunk_pos += decoded;
        }
    }

    // ── Phase 1: Read base64 data into chunk buffer ──
    // sd_io is held — prevents other tasks from doing SD flushes that
    // preempt our read loop and overflow the USB ring buffer.
    if (!error && !reached_empty_line) {
    while (1) {
        int line_len = recv_read_line(line_buf, sizeof(line_buf), RECV_TIMEOUT_US);
        if (line_len < 0) {
            if (!error) {
                printf("RECV_ERR timeout\n");
                fflush(stdout);
            }
            error = true;
            break;
        }
        if (line_len == 0) {
            reached_empty_line = true;
            break;  // empty line = end of base64 data
        }
        line_count++;

        // If we've already hit an error (e.g. buffer overflow), keep
        // reading lines until the empty line to drain the stream, but
        // don't process them.
        if (error) continue;

        // Decode base64 line
        size_t decoded = base64_decode(line_buf, line_len, decode_buf);
        if (decoded == 0) continue;

        // Append to chunk buffer
        if (chunk_pos + decoded > RECV_CHUNK_BUF_SIZE) {
            printf("RECV_ERR chunk too large (>%d bytes)\n", RECV_CHUNK_BUF_SIZE);
            fflush(stdout);
            error = true;
            continue;  // keep draining until empty line
        }
        memcpy(s_recv_chunk_buf + chunk_pos, decode_buf, decoded);
        chunk_pos += decoded;
    }
    }

    // Check for truncation (v1.1: CHUNK header declared expected size)
    bool truncated = false;
    if (expected_chunk_size > 0 && !error && reached_empty_line &&
        (int32_t)chunk_pos != expected_chunk_size) {
        truncated = true;
        printf("RECV_ERR truncated expected=%ld got=%lu\n",
               (long)expected_chunk_size, (unsigned long)chunk_pos);
        fflush(stdout);
    }

    // ── Drain: if we broke out before the empty line (timeout), consume
    // remaining data so the serial stream isn't poisoned.
    int64_t phase1_elapsed = (esp_timer_get_time() - phase1_start) / 1000;
    printf("[recv] phase1: %lu bytes, %d lines, empty=%d, err=%d, %lldms",
           (unsigned long)chunk_pos, line_count, reached_empty_line, error,
           (long long)phase1_elapsed);
    if (expected_chunk_size > 0)
        printf(", expect=%ld", (long)expected_chunk_size);
    printf("\n");
    printf("[recv] reads=%d spins=%d yields=%d max_gap=%dus\n",
           s_recv_reads, s_recv_spins, s_recv_yields, s_recv_max_gap_us);
    fflush(stdout);
    if (!reached_empty_line) {
        while (1) {
            int line_len = recv_read_line(line_buf, sizeof(line_buf), RECV_TIMEOUT_US);
            if (line_len < 0) break;   // timeout — nothing more coming
            if (line_len == 0) {
                reached_empty_line = true;
                break;
            }
        }
    }

    // ── Phase 2: Read and verify CHUNK_CRC32 (mandatory) ──
    // The webapp always sends CHUNK_CRC32 after the empty line.
    // We must read it even on error to keep the serial stream clean.
    // Skip any extra empty lines between the data and CHUNK_CRC32 —
    // terminals and paste buffers may insert additional newlines.
    uint32_t expected_crc = 0;
    bool has_crc = false;
    if (reached_empty_line) {
        // Read lines until we get a non-empty one (the CRC) or timeout.
        // Skip up to 16 extra empty lines (paste formatting artifacts).
        int empty_skipped = 0;
        while (empty_skipped < 16) {
            int crc_line_len = recv_read_line(line_buf, sizeof(line_buf), RECV_TIMEOUT_US);
            if (crc_line_len < 0) {
                printf("[recv] phase2: timeout (skipped %d empties)\n", empty_skipped);
                fflush(stdout);
                break;
            }
            if (crc_line_len == 0) { empty_skipped++; continue; }
            printf("[recv] phase2: '%.50s' (len=%d, skip=%d)\n",
                   line_buf, crc_line_len, empty_skipped);
            fflush(stdout);
            // Got a non-empty line — check if it's CHUNK_CRC32
            if (strncmp(line_buf, "CHUNK_CRC32 ", 12) == 0) {
                expected_crc = (uint32_t)strtoul(line_buf + 12, NULL, 16);
                has_crc = true;
            }
            break;  // either way, we're done looking
        }
    }

    // ── At this point the serial stream is fully drained for this chunk.
    // Safe to return on any error — no leftover data to poison commands.
    // Release sd_io so ADS-B/Meshy can flush while we handle errors or
    // do the Phase 3 write (which takes sd_io again as needed).
    if (held_sd_io) sd_io_give();
    vTaskPrioritySet(self, saved_prio);

    // Close file and bail on Phase 1 errors (timeout, buffer overflow)
    if (error) {
        if (sd_io_take(5000)) {
            fclose(f);
            sd_io_give();
        } else {
            fclose(f);
        }
        return;
    }

    // Bail on truncation (v1.1: CHUNK header size mismatch)
    // The RECV_ERR was already printed above — stream is drained, just close.
    if (truncated) {
        if (sd_io_take(5000)) {
            fclose(f);
            sd_io_give();
        } else {
            fclose(f);
        }
        return;
    }

    // Verify CRC32
    if (!has_crc) {
        printf("RECV_ERR no CHUNK_CRC32 received\n");
        fflush(stdout);
        if (sd_io_take(5000)) {
            fclose(f);
            sd_io_give();
        } else {
            fclose(f);
        }
        return;
    }

    uint32_t actual_crc = CRC32_FINAL(
        crc32_update(CRC32_INIT, s_recv_chunk_buf, chunk_pos));

    if (actual_crc != expected_crc) {
        printf("RECV_ERR crc_mismatch expected=%08lX actual=%08lX\n",
               (unsigned long)expected_crc, (unsigned long)actual_crc);
        fflush(stdout);
        // Don't write — webapp will retry this chunk
        if (sd_io_take(5000)) {
            fclose(f);
            sd_io_give();
        } else {
            fclose(f);
        }
        return;
    }

    // ── Phase 3: CRC verified — write chunk to file ──
    if (!sd_io_take(5000)) {
        printf("RECV_ERR sd card busy during write\n");
        fflush(stdout);
        fclose(f);
        return;
    }

    size_t written = fwrite(s_recv_chunk_buf, 1, chunk_pos, f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    sd_io_give();

    if (written != chunk_pos) {
        printf("RECV_ERR write failed: wrote %lu of %lu bytes\n",
               (unsigned long)written, (unsigned long)chunk_pos);
        fflush(stdout);
        return;
    }

    offset += chunk_pos;

    if (offset >= total) {
        printf("RECV_DONE %lu\n", (unsigned long)offset);
        fflush(stdout);
        ESP_LOGI(CONSOLE_TAG, "File received: %s (%lu bytes, CRC32 verified)",
                 s_recv_path, (unsigned long)offset);
        s_recv_path[0] = '\0';
        s_recv_total = 0;
    } else {
        printf("RECV_CHUNK %lu\n", (unsigned long)offset);
        fflush(stdout);
    }
}

// ── verify_crc32 command ─────────────────────────────────────────────

static void cmd_verify_crc32(const char *path, const char *expected_hex) {
    if (!path || !expected_hex) {
        printf("  usage: verify_crc32 <path> <expected_hex>\n");
        return;
    }

    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));

    uint32_t expected = (uint32_t)strtoul(expected_hex, NULL, 16);

    if (!sd_io_take(5000)) {
        printf("VERIFY_ERR sd card busy\n");
        return;
    }

    FILE *f = fopen(resolved, "rb");

    if (!f) {
        int e = errno;
        sd_io_give();
        printf("VERIFY_ERR cannot open %s: %s\n", resolved, strerror(e));
        return;
    }

    // Read file in 4KB chunks, computing CRC32 incrementally.
    // Hold sd_io for the entire read — this is a verification pass,
    // not a long blocking operation (27MB ≈ 2–5s at SPI speeds).
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    printf("VERIFY_START %s (%ld bytes)\n", resolved, file_size);
    fflush(stdout);
    int64_t verify_start = esp_timer_get_time();

    uint8_t read_buf[4096];
    uint32_t crc = CRC32_INIT;
    size_t total_read = 0;
    int64_t last_progress = esp_timer_get_time();

    while (1) {
        size_t got = fread(read_buf, 1, sizeof(read_buf), f);

        if (got == 0) break;
        crc = crc32_update(crc, read_buf, got);
        total_read += got;

        // Yield every 32 reads (~128KB) to feed task watchdog
        if ((total_read % (32 * sizeof(read_buf))) == 0) {
            vTaskDelay(1);
        }

        // Progress every ~10 seconds
        int64_t now = esp_timer_get_time();
        if ((now - last_progress) >= 10000000LL) {
            int pct = (file_size > 0) ? (int)((int64_t)total_read * 100 / file_size) : 0;
            printf("VERIFY_PROGRESS %d%% (%lu / %ld bytes)\n",
                   pct, (unsigned long)total_read, file_size);
            fflush(stdout);
            last_progress = now;
        }
    }

    fclose(f);
    sd_io_give();

    uint32_t actual = CRC32_FINAL(crc);
    int elapsed_s = (int)((esp_timer_get_time() - verify_start) / 1000000LL);

    if (actual == expected) {
        printf("VERIFY_OK %ld bytes in %ds\n", file_size, elapsed_s);
    } else {
        printf("VERIFY_FAIL actual=%08lX expected=%08lX (%ld bytes in %ds)\n",
               (unsigned long)actual, (unsigned long)expected, file_size, elapsed_s);
    }
    fflush(stdout);
}

static void cmd_stat(const char *path) {
    if (!path) { printf("  usage: stat <path>\n"); return; }
    char resolved[128];
    resolve_path(path, resolved, sizeof(resolved));
    struct stat st;
    if (stat(resolved, &st) == 0) {
        // Read first 16 bytes as hex for version/magic check
        char hex[33] = {0};
        FILE *f = fopen(resolved, "rb");
        if (f) {
            uint8_t hdr[16];
            size_t got = fread(hdr, 1, sizeof(hdr), f);
            fclose(f);
            for (size_t i = 0; i < got; i++) {
                sprintf(hex + i * 2, "%02x", hdr[i]);
            }
        }
        printf("STAT %ld %s %s\n", (long)st.st_size, hex, resolved);
    } else {
        printf("STAT -1 - %s\n", resolved);
    }
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
        if (sd_io_take(5000)) { cmd_ls(arg1); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "cat") == 0) {
        if (sd_io_take(5000)) { cmd_cat(arg1); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "head") == 0) {
        int n = arg2 ? atoi(arg2) : 20;
        if (n <= 0) n = 20;
        if (sd_io_take(5000)) { cmd_head(arg1, n); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "tail") == 0) {
        int n = arg2 ? atoi(arg2) : 20;
        if (n <= 0) n = 20;
        if (sd_io_take(5000)) { cmd_tail(arg1, n); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "rm") == 0 || strcmp(cmd, "del") == 0) {
        if (sd_io_take(5000)) { cmd_rm(arg1); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "mv") == 0 || strcmp(cmd, "rename") == 0) {
        if (sd_io_take(5000)) { cmd_mv(arg1, arg2); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "cp") == 0 || strcmp(cmd, "copy") == 0) {
        if (sd_io_take(5000)) { cmd_cp(arg1, arg2); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "df") == 0) {
        if (sd_io_take(5000)) { cmd_df(); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "cd") == 0) {
        if (arg1) {
            if (sd_io_take(5000)) { cmd_cd(arg1); sd_io_give(); }
            else printf("  SD card busy\n");
        } else {
            cmd_cd(NULL);
        }
    } else if (strcmp(cmd, "pwd") == 0) {
        cmd_pwd();
    } else if (strcmp(cmd, "mkdir") == 0) {
        if (sd_io_take(5000)) { cmd_mkdir(arg1); sd_io_give(); }
        else printf("  SD card busy\n");
    } else if (strcmp(cmd, "rmdir") == 0) {
        if (sd_io_take(5000)) { cmd_rmdir(arg1); sd_io_give(); }
        else printf("  SD card busy\n");

    // ── Service control ──────────────────────────────────────────────
    } else if (strcmp(cmd, "adsb") == 0) {
        cmd_adsb(arg1);
    } else if (strcmp(cmd, "meshy") == 0) {
        cmd_meshy(arg1);
    } else if (strcmp(cmd, "usb") == 0) {
        cmd_usb(arg1);

    } else if (strcmp(cmd, "status") == 0) {
        cmd_status();
    } else if (strcmp(cmd, "version") == 0 || strcmp(cmd, "ver") == 0) {
        cmd_version();
    } else if (strcmp(cmd, "mount") == 0) {
        printf("  Mounting SD card...\n");
        extern bool sd_remount(void);
        extern void sd_set_user_unmounted(bool);
        sd_set_user_unmounted(false);  // re-enable auto-remount regardless of result
        sd_log_bus_reset();            // clear SPI bus failure flag if set
        if (sd_remount()) {
            printf("  SD card mounted. Logging will resume on next ADS-B message.\n");
        } else {
            printf("  Mount failed — auto-remount re-enabled, will keep trying.\n");
        }
    } else if (strcmp(cmd, "unmount") == 0 || strcmp(cmd, "eject") == 0) {
        extern bool sd_is_mounted(void);
        extern void sd_set_user_unmounted(bool);
        if (!sd_is_mounted()) {
            printf("  SD card is not mounted.\n");
        } else {
            // Warn if logging is active
            bool logging = false;
            {
                extern bool sd_log_is_active(void);
                extern bool meshy_sd_is_active(void);
                bool adsb_log = sd_log_is_active();
                bool meshy_log = meshy_sd_is_active();
                if (adsb_log || meshy_log) {
                    printf("  Warning: active log files will be closed:\n");
                    if (adsb_log) printf("    - ADS-B CSV log\n");
                    if (meshy_log) printf("    - Meshtastic CSV log\n");
                    printf("  Continue? [y/N] ");
                    fflush(stdout);
                    // Read y/n with timeout
                    int64_t deadline = esp_timer_get_time() + 15000000LL; // 15s
                    bool confirmed = false;
                    while (esp_timer_get_time() < deadline) {
                        uint8_t c;
                        if (read(STDIN_FILENO, &c, 1) == 1) {
                            if (c == 'y' || c == 'Y') { confirmed = true; printf("y\n"); break; }
                            if (c == 'n' || c == 'N' || c == '\r' || c == '\n') { printf("n\n"); break; }
                        }
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    if (!confirmed) {
                        printf("  Unmount cancelled.\n");
                        logging = true;  // reuse as "cancelled" flag
                    }
                }
            }
            if (!logging) {
                extern void sd_safe_shutdown(void);
                sd_safe_shutdown();
                sd_set_user_unmounted(true);
                printf("  SD card safely unmounted. You can remove it now.\n");
                printf("  Auto-remount disabled. Use 'mount' to re-enable.\n");
            }
        }
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
            vTaskDelay(1);
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

    // ── USB Mass Storage ──
    } else if (strcmp(cmd, "msc") == 0) {
        extern bool sd_msc_is_active_fn(void);
        extern bool sd_msc_enter_fn(void);
        if (sd_msc_is_active_fn()) {
            printf("  Already in MSC mode. Type 'cdc' to exit.\n");
        } else {
            printf("  Entering USB Mass Storage mode...\n");
            printf("  \033[33mWARNING: ADS-B and Meshy logging will stop.\033[0m\n");
            if (sd_msc_enter_fn()) {
                printf("  \033[32mUSB Storage active.\033[0m SD card available to host.\n");
                printf("  Type 'cdc' to exit and resume logging.\n");
            } else {
                printf("  \033[31mFailed to enter MSC mode.\033[0m\n");
            }
        }
    } else if (strcmp(cmd, "cdc") == 0) {
        extern bool sd_msc_is_active_fn(void);
        extern void sd_msc_exit_fn(void);
        if (!sd_msc_is_active_fn()) {
            printf("  Not in MSC mode.\n");
        } else {
            sd_msc_exit_fn();
            printf("  \033[32mNormal operation resumed.\033[0m Fresh logs created.\n");
        }

    // ── WiFi ──
    } else if (strcmp(cmd, "wifi") == 0) {
        extern bool wifi_hosted_is_connected_fn(void);
        extern bool wifi_hosted_is_active(void);
        extern const char *wifi_hosted_get_ip_fn(void);
        if (!arg1 || strcmp(arg1, "status") == 0) {
            printf("  WiFi: %s\n", g_settings.wifi_enabled ? "enabled" : "disabled");
            printf("  SSID: %s\n", g_settings.wifi_ssid[0] ? g_settings.wifi_ssid : "(not set)");
            printf("  Active: %s  Connected: %s\n",
                   wifi_hosted_is_active() ? "yes" : "no",
                   wifi_hosted_is_connected_fn() ? "yes" : "no");
            if (wifi_hosted_is_connected_fn()) {
                printf("  IP: %s\n", wifi_hosted_get_ip_fn());
            }
        } else if (strcmp(arg1, "on") == 0) {
            if (g_settings.wifi_ssid[0] == '\0') {
                printf("  No SSID configured. Use 'wifi ssid <name>' first.\n");
            } else {
                g_settings.wifi_enabled = true;
                g_settings_save_pending = true;
                extern esp_err_t wifi_hosted_start_fn(void);
                if (!wifi_hosted_is_active()) {
                    printf("  Starting WiFi (SSID: %s)...\n", g_settings.wifi_ssid);
                    esp_err_t err = wifi_hosted_start_fn();
                    if (err == ESP_OK) {
                        printf("  WiFi started — connecting in background.\n");
                    } else {
                        printf("  WiFi start failed (0x%x) — will retry on reboot.\n", err);
                    }
                } else {
                    printf("  WiFi already active.\n");
                }
            }
        } else if (strcmp(arg1, "off") == 0) {
            g_settings.wifi_enabled = false;
            g_settings_save_pending = true;
            extern void wifi_hosted_pause(void);
            if (wifi_hosted_is_active()) wifi_hosted_pause();
            printf("  WiFi disabled.\n");
        } else if (strcmp(arg1, "ssid") == 0) {
            if (!arg2) {
                printf("  SSID: %s\n", g_settings.wifi_ssid[0] ? g_settings.wifi_ssid : "(not set)");
            } else {
                strncpy(g_settings.wifi_ssid, arg2, sizeof(g_settings.wifi_ssid) - 1);
                g_settings.wifi_ssid[sizeof(g_settings.wifi_ssid) - 1] = '\0';
                g_settings_save_pending = true;
                printf("  SSID set: %s\n", g_settings.wifi_ssid);
            }
        } else if (strcmp(arg1, "pass") == 0) {
            // Restore arg split so password can contain spaces
            if (arg_split) *arg_split = ' ';
            const char *pass_start = arg2 ? arg2 : NULL;
            if (!pass_start) {
                printf("  Password: %s\n", g_settings.wifi_pass[0] ? "(set)" : "(not set)");
            } else {
                strncpy(g_settings.wifi_pass, pass_start, sizeof(g_settings.wifi_pass) - 1);
                g_settings.wifi_pass[sizeof(g_settings.wifi_pass) - 1] = '\0';
                g_settings_save_pending = true;
                printf("  Password set (%d chars)\n", (int)strlen(g_settings.wifi_pass));
            }
        } else {
            printf("  usage: wifi [on|off|status|ssid <name>|pass <password>]\n");
        }

    // ── NTP ──
    } else if (strcmp(cmd, "ntp") == 0) {
        extern void time_manager_set_ntp_config_fn(const char *server, int32_t poll_s);
        if (!arg1) {
            printf("  NTP server: %s\n", g_settings.ntp_server);
            printf("  Poll interval: %d seconds\n", g_settings.ntp_poll_s);
        } else {
            // ntp <server> [poll_s]
            strncpy(g_settings.ntp_server, arg1, sizeof(g_settings.ntp_server) - 1);
            g_settings.ntp_server[sizeof(g_settings.ntp_server) - 1] = '\0';
            if (arg2) {
                int poll = atoi(arg2);
                if (poll >= 300 && poll <= 3600) {
                    g_settings.ntp_poll_s = poll;
                } else {
                    printf("  Poll interval must be 300-3600 seconds.\n");
                }
            }
            g_settings_save_pending = true;
            time_manager_set_ntp_config_fn(g_settings.ntp_server, g_settings.ntp_poll_s);
            printf("  NTP: %s (poll %ds)\n", g_settings.ntp_server, g_settings.ntp_poll_s);
        }

    // ── Time source status ──
    } else if (strcmp(cmd, "time") == 0) {
        extern void time_manager_print_status_fn(void);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        struct tm tm;
        gmtime_r(&tv.tv_sec, &tm);
        printf("  System clock: %04d-%02d-%02d %02d:%02d:%02d.%03ld UTC\n",
               tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
               tm.tm_hour, tm.tm_min, tm.tm_sec, tv.tv_usec / 1000);
        time_manager_print_status_fn();

    // ── MQTT feeder control ──
    } else if (strcmp(cmd, "mqtt") == 0) {
        if (!arg1 || strcmp(arg1, "status") == 0) {
            printf("  MQTT: %s\n", g_settings.mqtt_enabled ? "enabled" : "disabled");
            printf("  Server: %s:%u\n",
                   g_settings.mqtt_server[0] ? g_settings.mqtt_server : "(not set)",
                   g_settings.mqtt_port);
            printf("  User: %s\n", g_settings.mqtt_user[0] ? g_settings.mqtt_user : "(not set)");
            printf("  Password: %s\n", g_settings.mqtt_pass[0] ? "(set)" : "(not set)");
            printf("  Device ID: %s\n", g_settings.mqtt_device_id[0] ? g_settings.mqtt_device_id : "(auto from MAC)");
            printf("  Connected: %s\n", mqtt_feeder_is_connected() ? "yes" : "no");
        } else if (strcmp(arg1, "on") == 0) {
            g_settings.mqtt_enabled = true;
            g_settings_save_pending = true;
            mqtt_feeder_stop();   // safe if not running
            mqtt_feeder_init();   // connect now
            printf("  MQTT enabled.\n");
        } else if (strcmp(arg1, "off") == 0) {
            g_settings.mqtt_enabled = false;
            g_settings_save_pending = true;
            mqtt_feeder_stop();
            printf("  MQTT disabled and stopped.\n");
        } else if (strcmp(arg1, "server") == 0) {
            if (!arg2) {
                printf("  Server: %s\n", g_settings.mqtt_server[0] ? g_settings.mqtt_server : "(not set)");
            } else {
                strncpy(g_settings.mqtt_server, arg2, sizeof(g_settings.mqtt_server) - 1);
                g_settings.mqtt_server[sizeof(g_settings.mqtt_server) - 1] = '\0';
                g_settings_save_pending = true;
                if (g_settings.mqtt_enabled) { mqtt_feeder_stop(); mqtt_feeder_init(); }
                printf("  Server set: %s\n", g_settings.mqtt_server);
            }
        } else if (strcmp(arg1, "port") == 0) {
            if (!arg2) {
                printf("  Port: %u\n", g_settings.mqtt_port);
            } else {
                int p = atoi(arg2);
                if (p > 0 && p <= 65535) {
                    g_settings.mqtt_port = (uint16_t)p;
                    g_settings_save_pending = true;
                    if (g_settings.mqtt_enabled) { mqtt_feeder_stop(); mqtt_feeder_init(); }
                    printf("  Port set: %u\n", g_settings.mqtt_port);
                } else {
                    printf("  Invalid port (1–65535)\n");
                }
            }
        } else if (strcmp(arg1, "user") == 0) {
            if (!arg2) {
                printf("  User: %s\n", g_settings.mqtt_user[0] ? g_settings.mqtt_user : "(not set)");
            } else {
                strncpy(g_settings.mqtt_user, arg2, sizeof(g_settings.mqtt_user) - 1);
                g_settings.mqtt_user[sizeof(g_settings.mqtt_user) - 1] = '\0';
                g_settings_save_pending = true;
                if (g_settings.mqtt_enabled) { mqtt_feeder_stop(); mqtt_feeder_init(); }
                printf("  User set: %s\n", g_settings.mqtt_user);
            }
        } else if (strcmp(arg1, "pass") == 0) {
            if (arg_split) *arg_split = ' ';  // restore for passwords with spaces
            const char *pass_start = arg2 ? arg2 : NULL;
            if (!pass_start) {
                printf("  Password: %s\n", g_settings.mqtt_pass[0] ? "(set)" : "(not set)");
            } else {
                strncpy(g_settings.mqtt_pass, pass_start, sizeof(g_settings.mqtt_pass) - 1);
                g_settings.mqtt_pass[sizeof(g_settings.mqtt_pass) - 1] = '\0';
                g_settings_save_pending = true;
                if (g_settings.mqtt_enabled) { mqtt_feeder_stop(); mqtt_feeder_init(); }
                printf("  Password set (%d chars)\n", (int)strlen(g_settings.mqtt_pass));
            }
        } else if (strcmp(arg1, "id") == 0) {
            if (!arg2) {
                printf("  Device ID: %s\n", g_settings.mqtt_device_id[0] ? g_settings.mqtt_device_id : "(auto from MAC)");
            } else {
                if (strcmp(arg2, "auto") == 0) {
                    g_settings.mqtt_device_id[0] = '\0';
                    printf("  Device ID set to auto (MAC-derived)\n");
                } else {
                    strncpy(g_settings.mqtt_device_id, arg2, sizeof(g_settings.mqtt_device_id) - 1);
                    g_settings.mqtt_device_id[sizeof(g_settings.mqtt_device_id) - 1] = '\0';
                    printf("  Device ID set: %s\n", g_settings.mqtt_device_id);
                }
                g_settings_save_pending = true;
                if (g_settings.mqtt_enabled) { mqtt_feeder_stop(); mqtt_feeder_init(); }
            }
        } else {
            printf("  usage: mqtt [on|off|status|server <h>|port <n>|user <u>|pass <p>|id <id>]\n");
        }

    } else if (strcmp(cmd, "screenshot") == 0) {
        if (arg1 && strcmp(arg1, "send") == 0) {
            extern bool screenshot_send_fn(void);
            if (!screenshot_send_fn()) {
                printf("  No screenshot to send (capture one first)\n");
            }
        } else {
            extern bool screenshot_save_fn(void);
            printf("  Capturing screenshot...\n");
            if (!screenshot_save_fn()) {
                printf("  Screenshot failed (SD card mounted?)\n");
            }
        }

    // ── File receive (used by webapp to upload files to SD card) ──
    // cmd_recv manages sd_io internally — holds it only during file I/O,
    // not while blocking on stdin for base64 data.
    } else if (strcmp(cmd, "recv") == 0) {
        cmd_recv(arg1, arg2);

    } else if (strcmp(cmd, "verify_crc32") == 0) {
        cmd_verify_crc32(arg1, arg2);

    } else if (strcmp(cmd, "stat") == 0) {
        if (sd_io_take(5000)) { cmd_stat(arg1); sd_io_give(); }
        else printf("  SD card busy\n");

    } else if (strcmp(cmd, "reboot") == 0) {
        printf("  Shutting down SD card...\n");
        extern void sd_safe_shutdown(void);
        sd_safe_shutdown();
        printf("  Rebooting...\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();

    } else if (strcmp(cmd, "gain") == 0) {
        adsb_stats_t st = adsb_get_stats();
        if (!arg1 || arg1[0] == '\0') {
            // Show current gain
            printf("  Tuner gain: %.1f dB (%s%s)\n",
                   st.gain_tenths / 10.0,
                   st.gain_auto ? "adaptive" : "manual",
                   st.gain_auto ? (st.gain_phase == 1 ? ", converging" : ", steady") : "");
            printf("  CRC error rate: %.1f%% (%s)\n",
                   st.crc_error_rate * 100.0,
                   st.gain_auto ? (st.gain_phase == 1 ? "raw" : "EMA") : "raw");
            printf("  Target band: %d–%d%%\n", g_settings.gain_err_low, g_settings.gain_err_high);
            printf("  Position rate: %.1f/s\n", st.pos_rate);
            if (st.farthest_dist_nm > 0) {
                printf("  Max range: %.1f nm (%06lX)\n", st.farthest_dist_nm, (unsigned long)st.farthest_icao);
            }
            printf("  R820T gain range: 0.0 – 49.6 dB (29 steps)\n");
        } else if (strcmp(arg1, "auto") == 0) {
            adsb_set_gain(0, 0);
            printf("  Gain set to adaptive mode (starts at 49.6 dB, adjusts every 60s)\n");
        } else if (strcmp(arg1, "band") == 0) {
            if (!arg2) {
                const char *preset = "";
                if (g_settings.gain_err_low==35&&g_settings.gain_err_high==45) preset = " (conservative)";
                else if (g_settings.gain_err_low==40&&g_settings.gain_err_high==50) preset = " (balanced)";
                else if (g_settings.gain_err_low==45&&g_settings.gain_err_high==55) preset = " (aggressive)";
                printf("  Target band: %d–%d%%%s\n", g_settings.gain_err_low, g_settings.gain_err_high, preset);
                printf("  presets: conservative 35 45, balanced 40 50, aggressive 45 55\n");
            } else {
                int low = 0, high = 0;
                if (sscanf(arg2, "%d %d", &low, &high) == 2) {
                    if (low < 25 || low > 55 || high < 35 || high > 60 || high < low + 10 || high > low + 20) {
                        printf("  error: low 25–55%%, high 35–60%%, spread 10–20%%\n");
                        printf("  presets: conservative 35 45, balanced 40 50, aggressive 45 55\n");
                    } else {
                        g_settings.gain_err_low = (uint8_t)low;
                        g_settings.gain_err_high = (uint8_t)high;
                        g_settings_save_pending = true;
                        const char *preset = "";
                        if (low==35&&high==45) preset = " (conservative)";
                        else if (low==40&&high==50) preset = " (balanced)";
                        else if (low==45&&high==55) preset = " (aggressive)";
                        printf("  Target band set: %d–%d%%%s\n", low, high, preset);
                    }
                } else {
                    printf("  usage: gain band <low> <high> (e.g., gain band 45 55)\n");
                }
            }
        } else if (strcmp(arg1, "restart") == 0) {
            if (!st.gain_auto) {
                printf("  error: gain restart only works in adaptive mode (use 'gain auto' first)\n");
            } else {
                adsb_gain_restart();
                printf("  Restarting from current gain — re-entering phase 1\n");
            }
        } else {
            // Parse dB value
            float db = atof(arg1);
            if (db < 0.0f || db > 49.6f) {
                printf("  error: gain must be 0.0 – 49.6 dB (or 'auto', 'band', 'restart')\n");
            } else {
                uint16_t tenths = (uint16_t)(db * 10 + 0.5f);
                adsb_set_gain(1, tenths);
                printf("  Set manual gain: %.1f dB\n", db);
            }
        }

    } else if (strcmp(cmd, "gps") == 0) {
        extern void time_manager_set_integrity_mode_fn(int mode);
        if (!arg1 || arg1[0] == '\0') {
            // Show current mode
            const char *mode_str;
            switch (g_settings.gps_integrity_mode) {
                case 1:  mode_str = "strict (drift guard + NTP cross-check, no override)"; break;
                default: mode_str = "basic (garbled NMEA guard, 5-consecutive override)"; break;
            }
            printf("  GPS integrity mode: %s\n", mode_str);
            printf("  Modes: basic, strict\n");
        } else if (strcmp(arg1, "basic") == 0) {
            g_settings.gps_integrity_mode = 0;
            time_manager_set_integrity_mode_fn(0);
            g_settings_save_pending = true;
            printf("  GPS integrity mode set to basic\n");
        } else if (strcmp(arg1, "strict") == 0) {
            g_settings.gps_integrity_mode = 1;
            time_manager_set_integrity_mode_fn(1);
            g_settings_save_pending = true;
            printf("  GPS integrity mode set to strict\n");
        } else {
            printf("  error: unknown mode '%s' (use basic or strict)\n", arg1);
        }

    } else {
        printf("  unknown command: '%s' (type 'help' for commands)\n", cmd);
    }

    // NOTE: prompt is printed by the task loop (serial_console_task)
    // after dispatch_command() returns, gated on s_mode == MODE_CMD.
    // Do NOT add print_prompt() here — it causes a double-prompt bug
    // that breaks the webapp's serial command flow (sendCmd resolves
    // on the first prompt, leaving a stale second prompt that races
    // with subsequent commands).
}

// ---------------------------------------------------------------------------
// Console task
// ---------------------------------------------------------------------------
static void serial_console_task(void *arg) {
    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(CONSOLE_TAG, "Serial console ready (Ctrl+C for command mode)");
    {char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
    printf("%sCONSOLE: Serial console ready\n", _ts);}

    uint32_t heartbeat_next = esp_log_timestamp() + (g_settings.heartbeat_period_s * 1000);

    while (1) {
        uint8_t ch;
        int n = read(STDIN_FILENO, &ch, 1);

        if (n <= 0) {
            // Heartbeat in log mode
            if (s_mode == MODE_LOG && g_settings.heartbeat_enabled &&
                esp_log_timestamp() >= heartbeat_next) {
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                printf("%sCONSOLE: Serial console heartbeat\n", _ts);
                printf("%sMEM: free=%u largest_blk=%u min_ever=%u PSRAM=%u DMA=%u DMA_blk=%u DMA_min=%u\n",
                       _ts,
                       (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                       (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                       (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                       (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                       (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
                       (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
                       (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_DMA));

                // Update MQTT feeder status snapshot (no-op if feeder not running)
                {
                    adsb_stats_t st = adsb_get_stats();
                    receiver_pos_t rx = adsb_get_receiver_pos();
                    const esp_app_desc_t *app = esp_app_get_description();
                    mqtt_status_t ms = {0};
                    ms.lat = rx.lat;
                    ms.lon = rx.lon;
                    ms.sats = (uint8_t)rx.sats;
                    ms.hdop = (float)rx.hdop;
                    ms.gain_db = (float)st.gain_tenths / 10.0f;
                    ms.gain_phase = (uint8_t)st.gain_phase;
                    ms.error_rate = st.crc_error_rate;
                    ms.pos_rate = st.pos_rate;
                    ms.max_range_nm = (float)st.farthest_dist_nm;
                    ms.ac_count = (uint16_t)st.active_aircraft;
                    ms.msg_count = st.total_messages;
                    ms.msg_rate = st.msg_rate;
                    ms.mem_free = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
                    ms.mem_psram = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                    ms.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
                    if (app && app->version[0])
                        strncpy(ms.firmware_ver, app->version, sizeof(ms.firmware_ver) - 1);
                    strncpy(ms.dongle_model, st.dongle_model, sizeof(ms.dongle_model) - 1);
                    mqtt_feeder_update_status(&ms);
                }
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
            if (s_mode == MODE_CMD) print_prompt();
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

    ESP_LOGI(CONSOLE_TAG, "Serial console starting");

    // Hook ESP_LOG output — gates all ESP_LOGI/W/E in CMD mode
    s_original_log_vprintf = esp_log_set_vprintf(gated_log_vprintf);

    // Console task does file I/O and stdin reads — no SPI/DMA, safe for PSRAM stack.
    // Pin to core 1 so recv priority boost (5) doesn't preempt
    // usb_host_lib or class_driver on core 0 — ADS-B keeps full speed.
    // Must use PSRAM because internal RAM is exhausted by meshy SPI tasks.
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(serial_console_task, "console", 8192, NULL, 3, NULL, 1, MALLOC_CAP_SPIRAM);
    if (ret != pdPASS) {
        // Fallback to internal RAM (smaller stack), still pinned to core 1
        xTaskCreatePinnedToCore(serial_console_task, "console", 4096, NULL, 3, NULL, 1);
    }
}
