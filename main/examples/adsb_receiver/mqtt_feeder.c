// mqtt_feeder.c — MQTT publisher for ADS-B aircraft data
//
// See mqtt_feeder.h for API documentation.

#include "mqtt_feeder.h"
#include "device_settings.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TAG "MQTT"

// ISRG Root X1 — root CA for Let's Encrypt certificates.
// Valid until 2035-06-04. PEM format, null-terminated.
// Source: https://letsencrypt.org/certs/isrgrootx1.pem
// Non-static: shared with ota_updater.c for HTTPS downloads.
const char ISRG_ROOT_X1_PEM[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n";

// ─── Configuration ───────────────────────────────────────────────────────────

#define BATCH_INTERVAL_MS   2000    // flush aircraft batch every 2s
#define STATUS_INTERVAL_MS  30000   // status heartbeat every 30s
#define MAX_AIRCRAFT        64      // max aircraft per batch
#define TASK_STACK_SIZE     8192
#define TASK_PRIORITY       3

// ─── State ───────────────────────────────────────────────────────────────────

static esp_mqtt_client_handle_t s_client   = NULL;
static volatile bool            s_connected = false;
static TaskHandle_t             s_task      = NULL;

// Aircraft batch buffer — PSRAM-allocated at init
static mqtt_aircraft_t *s_ac_buf    = NULL;
static volatile int     s_ac_count  = 0;
static SemaphoreHandle_t s_ac_mutex = NULL;

// Receiver status — updated externally, read by feeder task
static mqtt_status_t     s_status       = {0};
static SemaphoreHandle_t s_status_mutex = NULL;

// Device identity
static char s_device_id[33] = {0};

// ─── Reconnect State ────────────────────────────────────────────────────────
// Manual reconnect: immediate → 5s × 2 → 30s thereafter.
// WiFi down → paused (no retries). WiFi up → resume.
#define RETRY_FAST_MS       5000    // first 3 retries: 5s apart
#define RETRY_SLOW_MS       30000   // after 3 failures: 30s apart
#define RETRY_FAST_COUNT    3       // number of fast retries before switching to slow

static int                  s_retry_count = 0;
static esp_timer_handle_t   s_retry_timer = NULL;
static volatile bool        s_paused      = false;

// WiFi connectivity check — defined in main.cpp
extern bool wifi_hosted_is_connected_fn(void);

static void retry_timer_cb(void *arg) {
    if (s_paused || !s_client) return;
    ESP_LOGI(TAG, "Reconnect attempt %d", s_retry_count + 1);
    esp_mqtt_client_reconnect(s_client);
}

static void schedule_retry(void) {
    if (s_paused || !s_client || !s_retry_timer) return;

    // First attempt: immediate. No timer needed.
    if (s_retry_count == 0) {
        s_retry_count++;
        ESP_LOGI(TAG, "Reconnecting immediately (attempt 1)");
        esp_mqtt_client_reconnect(s_client);
        return;
    }

    uint32_t delay_ms = (s_retry_count < RETRY_FAST_COUNT) ? RETRY_FAST_MS : RETRY_SLOW_MS;
    s_retry_count++;

    ESP_LOGI(TAG, "Scheduling reconnect in %lums (attempt %d)",
             (unsigned long)delay_ms, s_retry_count);
    esp_timer_stop(s_retry_timer);  // cancel any pending
    esp_timer_start_once(s_retry_timer, (uint64_t)delay_ms * 1000);
}

static void cancel_retry(void) {
    if (s_retry_timer) {
        esp_timer_stop(s_retry_timer);
    }
}

// ─── PSRAM-backed cJSON allocator ────────────────────────────────────────────
// cJSON uses malloc/free internally for building JSON trees.
// Redirect to PSRAM so JSON serialization doesn't consume internal RAM.

static void *cjson_psram_malloc(size_t size) {
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!p) p = malloc(size);  // fall back to internal if PSRAM exhausted
    return p;
}

static void cjson_psram_free(void *ptr) {
    free(ptr);  // heap_caps_malloc'd memory can be freed with free()
}

static void install_cjson_hooks(void) {
    static bool installed = false;
    if (installed) return;
    cJSON_Hooks hooks = { .malloc_fn = cjson_psram_malloc, .free_fn = cjson_psram_free };
    cJSON_InitHooks(&hooks);
    installed = true;
}

// ─── Forward declarations ────────────────────────────────────────────────────

static void feeder_task(void *arg);
static void event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data);
static void handle_admin_cmd(const char *cmd);
static void pub_batch(void);
static void pub_status(void);
static void pub_meta(void);

// ─── Device ID ───────────────────────────────────────────────────────────────

static void ensure_device_id(void) {
    extern device_settings_t g_settings;
    if (g_settings.mqtt_device_id[0] != '\0') {
        strncpy(s_device_id, g_settings.mqtt_device_id, sizeof(s_device_id) - 1);
        s_device_id[sizeof(s_device_id) - 1] = '\0';
        return;
    }
    // Use the P4's own eFuse base MAC — stable, per-chip, always available.
    // NOT ESP_MAC_WIFI_STA: on ESP-Hosted platforms the WiFi STA MAC belongs
    // to the C6 coprocessor and may differ from the P4's eFuse, or return
    // inconsistent values depending on whether ESP-Hosted has initialized.
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(s_device_id, sizeof(s_device_id),
             "p4-%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ─── Public API ──────────────────────────────────────────────────────────────

const char *mqtt_feeder_device_id(void) {
    ensure_device_id();
    return s_device_id;
}

void mqtt_feeder_init(void) {
    extern device_settings_t g_settings;

    if (!g_settings.mqtt_enabled) {
        ESP_LOGI(TAG, "MQTT feeder disabled in settings");
        return;
    }
    if (g_settings.mqtt_server[0] == '\0') {
        ESP_LOGI(TAG, "MQTT server not configured — skipping");
        return;
    }

    // Install PSRAM-backed cJSON hooks before any JSON work
    install_cjson_hooks();

    ensure_device_id();  // already called by mqtt_feeder_device_id(), but idempotent

    // Allocate aircraft batch buffer in PSRAM
    s_ac_buf = (mqtt_aircraft_t *)heap_caps_malloc(
        sizeof(mqtt_aircraft_t) * MAX_AIRCRAFT, MALLOC_CAP_SPIRAM);
    if (!s_ac_buf) {
        ESP_LOGE(TAG, "Failed to allocate aircraft buffer in PSRAM — MQTT disabled");
        return;
    }
    memset(s_ac_buf, 0, sizeof(mqtt_aircraft_t) * MAX_AIRCRAFT);
    s_ac_count = 0;

    // Create mutexes if not already created (survive stop/reinit cycles)
    if (!s_ac_mutex) s_ac_mutex = xSemaphoreCreateMutex();
    if (!s_status_mutex) s_status_mutex = xSemaphoreCreateMutex();
    if (!s_ac_mutex || !s_status_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes — MQTT disabled");
        heap_caps_free(s_ac_buf);
        s_ac_buf = NULL;
        return;
    }

    // Build broker URI
    char uri[192];
    snprintf(uri, sizeof(uri), "mqtts://%s:%u",
             g_settings.mqtt_server, g_settings.mqtt_port);

    esp_mqtt_client_config_t cfg = {0};
    cfg.broker.address.uri = uri;
    cfg.broker.verification.certificate = ISRG_ROOT_X1_PEM;
    cfg.credentials.client_id = s_device_id;
    cfg.credentials.username = g_settings.mqtt_user[0] ? g_settings.mqtt_user : NULL;
    cfg.credentials.authentication.password = g_settings.mqtt_pass[0] ? g_settings.mqtt_pass : NULL;
    cfg.session.keepalive = 60;
    cfg.network.disable_auto_reconnect = true;  // we handle reconnect manually
    cfg.buffer.size = 4096;      // PSRAM via malloc (CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=0)
    cfg.buffer.out_size = 4096;

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to init MQTT client — MQTT disabled");
        heap_caps_free(s_ac_buf);
        s_ac_buf = NULL;
        return;
    }

    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, event_handler, NULL);

    // Pre-allocate retry timer to avoid internal RAM allocation during disconnect
    if (!s_retry_timer) {
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = retry_timer_cb;
        timer_args.name = "mqtt_retry";
        esp_timer_create(&timer_args, &s_retry_timer);
    }

    esp_err_t err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client (0x%x) — MQTT disabled", err);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        heap_caps_free(s_ac_buf);
        s_ac_buf = NULL;
        return;
    }

    // Create feeder task with PSRAM stack
    BaseType_t ret = xTaskCreateWithCaps(
        feeder_task, "mqtt_feeder",
        TASK_STACK_SIZE, NULL,
        TASK_PRIORITY, &s_task,
        MALLOC_CAP_SPIRAM);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create feeder task — MQTT disabled");
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        heap_caps_free(s_ac_buf);
        s_ac_buf = NULL;
        return;
    }

    ESP_LOGI(TAG, "Feeder started → %s", uri);
}

void mqtt_feeder_update_aircraft(const mqtt_aircraft_t *ac) {
    // Fast path: bail immediately if not connected or not initialized
    if (!s_connected || !s_ac_buf || !s_ac_mutex) return;

    if (xSemaphoreTake(s_ac_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (s_ac_count < MAX_AIRCRAFT) {
            s_ac_buf[s_ac_count++] = *ac;
        }
        // else: buffer full — drop this update (will be in next batch)
        xSemaphoreGive(s_ac_mutex);
    }
    // else: mutex contention — drop silently, never block the decode loop
}

void mqtt_feeder_update_status(const mqtt_status_t *st) {
    if (!s_status_mutex) return;

    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        s_status = *st;
        xSemaphoreGive(s_status_mutex);
    }
}

void mqtt_feeder_stop(void) {
    if (s_task) {
        vTaskDelete(s_task);
        s_task = NULL;
    }
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }
    if (s_ac_buf) {
        heap_caps_free(s_ac_buf);
        s_ac_buf = NULL;
    }
    s_connected = false;
    s_ac_count = 0;
    ESP_LOGI(TAG, "Feeder stopped");
}

bool mqtt_feeder_is_connected(void) {
    return s_connected;
}

void mqtt_feeder_pause(void) {
    if (!s_client) return;
    s_paused = true;
    cancel_retry();
    esp_mqtt_client_stop(s_client);
    s_connected = false;
    ESP_LOGI(TAG, "Paused (WiFi down)");
}

void mqtt_feeder_resume(void) {
    if (!s_client) return;
    s_paused = false;
    s_retry_count = 0;
    ESP_LOGI(TAG, "Resuming (WiFi up)");
    esp_mqtt_client_start(s_client);
}

// ─── Event Handler ───────────────────────────────────────────────────────────

static void event_handler(void *arg, esp_event_base_t base,
                           int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            s_connected = true;
            s_retry_count = 0;
            cancel_retry();
            ESP_LOGI(TAG, "Connected to broker");
            pub_meta();
            // Subscribe to admin commands
            {
                char cmd_topic[64];
                snprintf(cmd_topic, sizeof(cmd_topic), "adsb/%s/cmd", s_device_id);
                esp_mqtt_client_subscribe(s_client, cmd_topic, 1);
                ESP_LOGI(TAG, "Subscribed to %s", cmd_topic);
            }
            // Check for pending rollback ack
            {
                extern bool ota_get_pending_rollback(char *, size_t, char *, size_t);
                extern void ota_clear_pending_rollback(void);
                char run_ver[32], fail_ver[32];
                if (ota_get_pending_rollback(run_ver, sizeof(run_ver),
                                              fail_ver, sizeof(fail_ver))) {
                    mqtt_feeder_publish_ack("firmware_update", "rollback",
                                            fail_ver, run_ver);
                    ota_clear_pending_rollback();
                }
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_connected = false;
            if (s_paused) {
                ESP_LOGI(TAG, "Disconnected (WiFi down — waiting for reconnect)");
            } else {
                ESP_LOGW(TAG, "Disconnected from broker");
                schedule_retry();
            }
            break;
        case MQTT_EVENT_DATA:
            // Incoming message — route admin commands
            if (event->topic && event->topic_len > 0 && event->data && event->data_len > 0) {
                // Check if topic ends with "/cmd"
                char topic_buf[96];
                int tlen = event->topic_len < (int)sizeof(topic_buf) - 1
                           ? event->topic_len : (int)sizeof(topic_buf) - 1;
                memcpy(topic_buf, event->topic, tlen);
                topic_buf[tlen] = '\0';

                char expected_cmd[64];
                snprintf(expected_cmd, sizeof(expected_cmd), "adsb/%s/cmd", s_device_id);

                if (strcmp(topic_buf, expected_cmd) == 0) {
                    // Parse JSON command
                    char data_buf[256];
                    int dlen = event->data_len < (int)sizeof(data_buf) - 1
                               ? event->data_len : (int)sizeof(data_buf) - 1;
                    memcpy(data_buf, event->data, dlen);
                    data_buf[dlen] = '\0';

                    cJSON *root = cJSON_Parse(data_buf);
                    if (root) {
                        cJSON *jcmd = cJSON_GetObjectItem(root, "cmd");
                        if (jcmd && cJSON_IsString(jcmd)) {
                            handle_admin_cmd(jcmd->valuestring);
                        }
                        cJSON_Delete(root);
                    }
                }
            }
            break;
        case MQTT_EVENT_ERROR:
            if (event->error_handle &&
                event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Transport error: %s",
                         strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            break;
    }
}

// ─── Feeder Task ─────────────────────────────────────────────────────────────

static void feeder_task(void *arg) {
    int64_t last_batch  = esp_timer_get_time();
    int64_t last_status = esp_timer_get_time();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));  // wake every 500ms

        if (!s_connected) continue;

        int64_t now = esp_timer_get_time();

        // Aircraft batch
        if ((now - last_batch) >= (BATCH_INTERVAL_MS * 1000LL)) {
            pub_batch();
            last_batch = now;
        }

        // Status heartbeat
        if ((now - last_status) >= (STATUS_INTERVAL_MS * 1000LL)) {
            pub_status();
            last_status = now;
        }
    }
}

// ─── Publishers ──────────────────────────────────────────────────────────────

static void pub_batch(void) {
    if (!s_ac_buf) return;

    // Snapshot and clear the buffer under lock
    // Use a PSRAM-allocated copy to avoid holding the mutex during JSON serialization
    mqtt_aircraft_t *batch = (mqtt_aircraft_t *)heap_caps_malloc(
        sizeof(mqtt_aircraft_t) * MAX_AIRCRAFT, MALLOC_CAP_SPIRAM);
    if (!batch) return;  // PSRAM exhausted — skip this batch

    int count = 0;
    if (xSemaphoreTake(s_ac_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = s_ac_count;
        if (count > 0) {
            memcpy(batch, s_ac_buf, count * sizeof(mqtt_aircraft_t));
            s_ac_count = 0;
        }
        xSemaphoreGive(s_ac_mutex);
    }

    if (count == 0) {
        heap_caps_free(batch);
        return;
    }

    // Build JSON (cJSON allocations go to PSRAM via hooks)
    cJSON *root = cJSON_CreateObject();
    if (!root) { heap_caps_free(batch); return; }

    cJSON *arr = cJSON_AddArrayToObject(root, "aircraft");

    // Add receiver position if available
    if (s_status.lat != 0.0 || s_status.lon != 0.0) {
        cJSON_AddNumberToObject(root, "rx_lat", s_status.lat);
        cJSON_AddNumberToObject(root, "rx_lon", s_status.lon);
    }

    for (int i = 0; i < count; i++) {
        const mqtt_aircraft_t *ac = &batch[i];
        cJSON *obj = cJSON_CreateObject();
        if (!obj) continue;
        cJSON_AddStringToObject(obj, "icao", ac->icao);
        if (ac->callsign[0])        cJSON_AddStringToObject(obj, "callsign", ac->callsign);
        if (ac->has_pos) {
            cJSON_AddNumberToObject(obj, "lat", ac->lat);
            cJSON_AddNumberToObject(obj, "lon", ac->lon);
        }
        if (ac->alt_ft != 0)        cJSON_AddNumberToObject(obj, "alt", ac->alt_ft);
        if (ac->speed_kt != 0)      cJSON_AddNumberToObject(obj, "speed", ac->speed_kt);
        if (ac->heading_deg >= 0)    cJSON_AddNumberToObject(obj, "hdg", ac->heading_deg);
        if (ac->vert_rate_fpm != 0)  cJSON_AddNumberToObject(obj, "vr", ac->vert_rate_fpm);
        if (ac->dist_nm > 0)        cJSON_AddNumberToObject(obj, "dist", ac->dist_nm);
        if (ac->bearing_deg >= 0)    cJSON_AddNumberToObject(obj, "brg", ac->bearing_deg);
        cJSON_AddItemToArray(arr, obj);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    heap_caps_free(batch);

    if (json) {
        char topic[64];
        snprintf(topic, sizeof(topic), "adsb/%s/aircraft", s_device_id);
        esp_mqtt_client_publish(s_client, topic, json, 0, 0, 0);  // QoS 0
        free(json);  // allocated by cJSON via PSRAM hooks
    }
}

static void pub_status(void) {
    mqtt_status_t st;
    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        st = s_status;
        xSemaphoreGive(s_status_mutex);
    } else {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    if (st.lat != 0.0 || st.lon != 0.0) {
        cJSON_AddNumberToObject(root, "lat", st.lat);
        cJSON_AddNumberToObject(root, "lon", st.lon);
    }
    cJSON_AddNumberToObject(root, "sats", st.sats);
    cJSON_AddNumberToObject(root, "hdop", st.hdop);
    cJSON_AddNumberToObject(root, "gain_db", st.gain_db);
    cJSON_AddStringToObject(root, "gain_phase",
        st.gain_phase == 1 ? "converging" : (st.gain_phase == 2 ? "steady" : "manual"));
    cJSON_AddNumberToObject(root, "error_rate", st.error_rate);
    cJSON_AddNumberToObject(root, "pos_rate", st.pos_rate);
    cJSON_AddNumberToObject(root, "max_range_nm", st.max_range_nm);
    cJSON_AddNumberToObject(root, "ac_count", st.ac_count);
    cJSON_AddNumberToObject(root, "msg_count", st.msg_count);
    cJSON_AddNumberToObject(root, "msg_rate", st.msg_rate);
    cJSON_AddNumberToObject(root, "mem_free", st.mem_free);
    cJSON_AddNumberToObject(root, "mem_psram", st.mem_psram);
    cJSON_AddNumberToObject(root, "uptime_s", st.uptime_s);
    if (st.firmware_ver[0])
        cJSON_AddStringToObject(root, "firmware_ver", st.firmware_ver);
    if (st.dongle_model[0])
        cJSON_AddStringToObject(root, "dongle_model", st.dongle_model);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json) {
        char topic[64];
        snprintf(topic, sizeof(topic), "adsb/%s/status", s_device_id);
        esp_mqtt_client_publish(s_client, topic, json, 0, 1, 0);  // QoS 1
        free(json);
    }
}

static void pub_meta(void) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddStringToObject(root, "device_id", s_device_id);
    cJSON_AddStringToObject(root, "client_type", "firmware");
    cJSON_AddStringToObject(root, "platform", "esp32-p4");

    if (s_status.firmware_ver[0])
        cJSON_AddStringToObject(root, "firmware_ver", s_status.firmware_ver);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json) {
        char topic[64];
        snprintf(topic, sizeof(topic), "adsb/%s/meta", s_device_id);
        esp_mqtt_client_publish(s_client, topic, json, 0, 1, 1);  // QoS 1, retained
        free(json);
    }

    ESP_LOGI(TAG, "Published device meta (retained)");
}

// ─── Admin Commands ──────────────────────────────────────────────────────────

static void handle_admin_cmd(const char *cmd) {
    extern esp_err_t ota_download_start(bool force);
    extern esp_err_t ota_flash_from_sd(bool force);

    ESP_LOGI(TAG, "Admin command: %s", cmd);

    if (strcmp(cmd, "firmware_download") == 0) {
        esp_err_t err = ota_download_start(false);
        if (err == ESP_ERR_INVALID_STATE) {
            mqtt_feeder_publish_ack(cmd, "error", "already downloading", NULL);
        } else if (err != ESP_OK) {
            mqtt_feeder_publish_ack(cmd, "error", "download start failed", NULL);
        }
        // Success ack is published by the download task when it completes

    } else if (strcmp(cmd, "firmware_download_force") == 0) {
        esp_err_t err = ota_download_start(true);
        if (err == ESP_ERR_INVALID_STATE) {
            mqtt_feeder_publish_ack(cmd, "error", "already downloading", NULL);
        } else if (err != ESP_OK) {
            mqtt_feeder_publish_ack(cmd, "error", "download start failed", NULL);
        }

    } else if (strcmp(cmd, "firmware_update") == 0) {
        mqtt_feeder_publish_ack(cmd, "starting", NULL, NULL);
        // ota_flash_from_sd reboots on success — ack won't be sent on failure
        esp_err_t err = ota_flash_from_sd(false);
        if (err == ESP_ERR_NOT_FOUND) {
            mqtt_feeder_publish_ack(cmd, "error", "no firmware on SD card", NULL);
        } else if (err == ESP_ERR_INVALID_STATE) {
            mqtt_feeder_publish_ack(cmd, "error", "already running this version", NULL);
        } else if (err == ESP_ERR_INVALID_CRC) {
            mqtt_feeder_publish_ack(cmd, "error", "SHA256 verification failed", NULL);
        } else if (err != ESP_OK) {
            mqtt_feeder_publish_ack(cmd, "error", "flash failed", NULL);
        }

    } else if (strcmp(cmd, "firmware_update_force") == 0) {
        mqtt_feeder_publish_ack(cmd, "starting", NULL, NULL);
        esp_err_t err = ota_flash_from_sd(true);
        if (err != ESP_OK) {
            char reason[48];
            snprintf(reason, sizeof(reason), "flash failed: %s", esp_err_to_name(err));
            mqtt_feeder_publish_ack(cmd, "error", reason, NULL);
        }

    } else {
        ESP_LOGW(TAG, "Unknown admin command: %s", cmd);
        mqtt_feeder_publish_ack(cmd, "error", "unknown command", NULL);
    }
}

void mqtt_feeder_publish_ack(const char *cmd, const char *status,
                              const char *reason, const char *version) {
    if (!s_connected || !s_client) return;

    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddStringToObject(root, "cmd", cmd);
    cJSON_AddStringToObject(root, "status", status);
    if (reason) cJSON_AddStringToObject(root, "reason", reason);

    // Include version — use provided or fall back to running version
    extern const char *ota_running_version(void);
    const char *ver = version ? version : ota_running_version();
    if (ver) cJSON_AddStringToObject(root, "version", ver);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json) {
        char topic[64];
        snprintf(topic, sizeof(topic), "adsb/%s/cmd/ack", s_device_id);
        esp_mqtt_client_publish(s_client, topic, json, 0, 1, 0);  // QoS 1
        free(json);
        ESP_LOGI(TAG, "Ack: %s %s%s%s", cmd, status,
                 reason ? " — " : "", reason ? reason : "");
    }
}
