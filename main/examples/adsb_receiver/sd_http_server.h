/*
 * Lightweight HTTP file server for SD card access.
 * Serves /sdcard/ directory listings and file downloads over WiFi.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the SD card HTTP file server.
 * Spawns a background task that waits for a network interface to obtain
 * an IP address, then starts an HTTP server on port 80.
 *
 * Browse to http://<device-ip>/ to list files.
 * Download files via http://<device-ip>/dl/<filename>
 */
void sd_http_server_start(void);

#ifdef __cplusplus
}
#endif
