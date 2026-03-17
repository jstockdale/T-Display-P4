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

static const char *CONSOLE_TAG = "CONSOLE";

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

    // CMD mode — format into a temp buffer and push to replay ring
    char tmp[REPLAY_LINE_MAX];
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    replay_push(tmp);
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
        "  \033[32mmount\033[0m             mount SD card\n"
        "  \033[32munmount\033[0m           safely unmount SD card\n"
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

    char *sp = strchr(line, ' ');
    if (sp) {
        *sp = '\0';
        arg1 = sp + 1;
        while (*arg1 == ' ') arg1++;

        sp = strchr(arg1, ' ');
        if (sp) {
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

    while (1) {
        uint8_t ch;
        int n = read(STDIN_FILENO, &ch, 1);

        if (n <= 0) {
            // No data available, yield
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
    xTaskCreate(serial_console_task, "console", 8192, NULL, 2, NULL);
}
