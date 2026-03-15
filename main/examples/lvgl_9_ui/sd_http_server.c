/*
 * Lightweight HTTP file server for SD card access.
 * Serves directory listings and file downloads from /sdcard/.
 */

#include "sd_http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "HTTP_FS";
#define SD_BASE "/sdcard"
#define FILE_BUF_SIZE  4096

// ============================================================
// Helpers
// ============================================================

static const char *size_str(off_t bytes, char *buf, size_t buflen) {
    if (bytes < 1024)
        snprintf(buf, buflen, "%ld B", (long)bytes);
    else if (bytes < 1024 * 1024)
        snprintf(buf, buflen, "%.1f KB", bytes / 1024.0);
    else
        snprintf(buf, buflen, "%.1f MB", bytes / (1024.0 * 1024.0));
    return buf;
}

// URL-decode in place (handles %XX sequences)
static void url_decode(char *str) {
    char *src = str, *dst = str;
    while (*src) {
        if (*src == '%' && src[1] && src[2]) {
            char hex[3] = { src[1], src[2], 0 };
            *dst++ = (char)strtol(hex, NULL, 16);
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

// ============================================================
// GET /  — directory listing
// ============================================================

static esp_err_t index_handler(httpd_req_t *req) {
    DIR *dir = opendir(SD_BASE);
    if (!dir) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "SD card not mounted or /sdcard not accessible");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html");

    // Send header
    const char *header =
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ADS-B Receiver — SD Card</title>"
        "<style>"
        "body{font-family:monospace;background:#111;color:#0f0;padding:20px;}"
        "h1{color:#0f0;border-bottom:1px solid #0f0;padding-bottom:8px;}"
        "table{border-collapse:collapse;width:100%;}"
        "th,td{text-align:left;padding:6px 12px;}"
        "th{border-bottom:1px solid #0f0;}"
        "tr:hover{background:#1a1a1a;}"
        "a{color:#0f0;text-decoration:none;}"
        "a:hover{text-decoration:underline;}"
        ".sz{color:#888;text-align:right;}"
        ".empty{color:#888;padding:20px;}"
        "</style></head><body>"
        "<h1>SD Card Files</h1><table>"
        "<tr><th>Name</th><th class='sz'>Size</th></tr>";
    httpd_resp_sendstr_chunk(req, header);

    // List files
    struct dirent *ent;
    int file_count = 0;
    char line[768];
    char sbuf[32];

    while ((ent = readdir(dir)) != NULL) {
        // Skip hidden files
        if (ent->d_name[0] == '.') continue;

        char fullpath[280];
        snprintf(fullpath, sizeof(fullpath), SD_BASE "/%s", ent->d_name);

        struct stat st;
        if (stat(fullpath, &st) != 0) continue;

        if (S_ISDIR(st.st_mode)) {
            snprintf(line, sizeof(line),
                "<tr><td>&#128193; %s/</td><td class='sz'>—</td></tr>",
                ent->d_name);
        } else {
            snprintf(line, sizeof(line),
                "<tr><td><a href='/dl/%s'>&#128196; %s</a></td>"
                "<td class='sz'>%s</td></tr>",
                ent->d_name, ent->d_name, size_str(st.st_size, sbuf, sizeof(sbuf)));
        }
        httpd_resp_sendstr_chunk(req, line);
        file_count++;
    }
    closedir(dir);

    if (file_count == 0) {
        httpd_resp_sendstr_chunk(req, "<tr><td class='empty' colspan='2'>No files found</td></tr>");
    }

    httpd_resp_sendstr_chunk(req, "</table></body></html>");
    httpd_resp_sendstr_chunk(req, NULL);  // finish chunked response
    return ESP_OK;
}

// ============================================================
// GET /dl/*  — file download
// ============================================================

static esp_err_t download_handler(httpd_req_t *req) {
    // URI is "/dl/<filename>" — skip the "/dl/" prefix
    const char *filename = req->uri + 4;
    if (!filename || filename[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }

    // Decode and sanitize
    char decoded[128];
    strncpy(decoded, filename, sizeof(decoded) - 1);
    decoded[sizeof(decoded) - 1] = '\0';
    url_decode(decoded);

    // Block path traversal
    if (strstr(decoded, "..") != NULL) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Invalid path");
        return ESP_FAIL;
    }

    char filepath[280];
    snprintf(filepath, sizeof(filepath), SD_BASE "/%s", decoded);

    FILE *f = fopen(filepath, "r");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Serving: %s", filepath);

    // Set content type and disposition for download
    httpd_resp_set_type(req, "application/octet-stream");
    char disposition[300];
    snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", decoded);
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);

    // Stream file in chunks
    char *buf = malloc(FILE_BUF_SIZE);
    if (!buf) {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    size_t n;
    while ((n = fread(buf, 1, FILE_BUF_SIZE, f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
            ESP_LOGW(TAG, "Client disconnected during transfer");
            break;
        }
    }

    free(buf);
    fclose(f);

    httpd_resp_send_chunk(req, NULL, 0);  // finish
    return ESP_OK;
}

// ============================================================
// Server startup
// ============================================================

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 8192;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    httpd_uri_t uri_index = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &uri_index);

    httpd_uri_t uri_download = {
        .uri = "/dl/*",
        .method = HTTP_GET,
        .handler = download_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &uri_download);

    return server;
}

// ============================================================
// Background task — waits for WiFi STA IP, then starts server
// ============================================================

static void http_server_task(void *arg) {
    ESP_LOGI(TAG, "Waiting for network...");

    // Poll until any netif has a valid IP (timeout after 30s)
    for (int attempt = 0; attempt < 15; attempt++) {
        esp_netif_t *netif = NULL;
        while ((netif = esp_netif_next_unsafe(netif)) != NULL) {
            esp_netif_ip_info_t ip_info;
            if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
                ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "Network ready — IP: " IPSTR, IP2STR(&ip_info.ip));
                goto ip_found;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Timed out — start server anyway (might work later if IP arrives)
    ESP_LOGW(TAG, "No IP found after 30s, starting server anyway");

ip_found:
    ;
    httpd_handle_t server = start_webserver();
    if (!server) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        vTaskDelete(NULL);
        return;
    }

    // Print the URL
    esp_netif_t *netif = NULL;
    while ((netif = esp_netif_next_unsafe(netif)) != NULL) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
            ip_info.ip.addr != 0) {
            printf("\n=========================================\n");
            printf("  HTTP file server: http://" IPSTR "/\n", IP2STR(&ip_info.ip));
            printf("=========================================\n\n");
            goto done;
        }
    }

    printf("\n=========================================\n");
    printf("  HTTP file server: running on port 80\n");
    printf("  (waiting for IP assignment)\n");
    printf("=========================================\n\n");

done:
    vTaskDelete(NULL);
}

void sd_http_server_start(void) {
    xTaskCreate(http_server_task, "http_fs", 4096, NULL, 2, NULL);
}
