/**
 * ESP32-CAM Waste Classification — Manual Trigger
 *
 * Classification fires ONLY when:
 *   1. User presses "CLASSIFY" button on web UI, or
 *   2. User types '1' + Enter in the serial monitor.
 *
 * Result is printed as LABEL:<lowercase> for the bridge script
 * to forward to ESP32-S3 toolkit.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "esp_camera.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "img_converters.h"
#include "mbedtls/base64.h"

static const char *TAG = "classify";

/* ══════════════════════════════════════════════════════════
 *  USER CONFIGURATION
 * ══════════════════════════════════════════════════════════ */

#define WIFI_STA_SSID       "Vamsi\xe2\x80\x99s iPhone (2)"
#define WIFI_STA_PASS       "i7A1-T9DB-Dvcb-Nd86"
#define USE_PHONE_HOTSPOT   1

#define OPENROUTER_API_KEY  "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"
#define OPENROUTER_MODEL    "openai/gpt-5.4"
#define OPENROUTER_URL      "https://openrouter.ai/api/v1/chat/completions"

/* ══════════════════════════════════════════════════════════ */

/* ── State ─────────────────────────────────────────────── */
static bool     wifi_connected = false;
static SemaphoreHandle_t classify_sem  = NULL;
static SemaphoreHandle_t jpg_mutex     = NULL;
static uint8_t          *classify_jpg  = NULL;
static size_t            classify_jlen = 0;

/* Last classification result (shown on web UI) */
static char last_label[32] = "\xe2\x80\x94";  /* em-dash */
static int  classify_busy  = 0;

/* ── WiFi ──────────────────────────────────────────────── */

static void wifi_ev(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA: started");
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *dis = (wifi_event_sta_disconnected_t *)data;
            ESP_LOGW(TAG, "STA: disconnected (reason=%d), reconnecting...", dis->reason);
            wifi_connected = false;
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_wifi_connect();
            break;
        }
        default: break
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA: got IP " IPSTR, IP2STR(&evt->ip_info.ip));
        ESP_LOGI(TAG, "Web UI:  http://" IPSTR "/", IP2STR(&evt->ip_info.ip));
        wifi_connected = true;
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ev, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_ev, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta_cfg = { 0 };
    strncpy((char *)sta_cfg.sta.ssid, WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid));
#ifdef USE_PHONE_HOTSPOT
    strncpy((char *)sta_cfg.sta.password, WIFI_STA_PASS, sizeof(sta_cfg.sta.password));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA connecting to '%s'...", WIFI_STA_SSID);
    esp_wifi_connect();
}

/* ── Camera (AI-Thinker ESP32-CAM) ─────────────────────── */

static camera_config_t cam_cfg = {
    .pin_pwdn  = 32, .pin_reset = -1,
    .pin_xclk  = 0,
    .pin_sccb_sda = 26, .pin_sccb_scl = 27,
    .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36,
    .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 = 5,
    .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
};

/* ── OpenRouter classification ─────────────────────────── */

typedef struct { char *buf; size_t len, cap; } resp_buf_t;

static esp_err_t http_ev(esp_http_client_event_t *evt)
{
    resp_buf_t *rb = (resp_buf_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && rb) {
        if (rb->len + evt->data_len < rb->cap) {
            memcpy(rb->buf + rb->len, evt->data, evt->data_len);
            rb->len += evt->data_len;
            rb->buf[rb->len] = '\0';
        }
    }
    return ESP_OK;
}

static const char *extract_classification(const char *json)
{
    const char *p = strstr(json, "\"content\"");
    if (!p) return "trash";
    p = strchr(p + 9, '\"');
    if (!p) return "trash";
    p++;
    if (strstr(p, "Cardboard") || strstr(p, "cardboard")) return "cardboard";
    if (strstr(p, "Plastic")   || strstr(p, "plastic"))   return "plastic";
    if (strstr(p, "Metal")     || strstr(p, "metal"))      return "metal";
    return "trash";
}

static void do_classify(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { ESP_LOGE(TAG, "Camera capture failed"); return; }

    uint8_t *jpg = malloc(fb->len);
    if (!jpg) { esp_camera_fb_return(fb); return; }
    memcpy(jpg, fb->buf, fb->len);
    size_t jlen = fb->len;
    esp_camera_fb_return(fb);

    if (xSemaphoreTake(jpg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (classify_jpg) free(classify_jpg);
        classify_jpg  = jpg;
        classify_jlen = jlen;
        xSemaphoreGive(jpg_mutex);
        xSemaphoreGive(classify_sem);
        ESP_LOGI(TAG, "Snapshot queued (%u bytes)", (unsigned)jlen);
    } else {
        free(jpg);
    }
}

static void classify_task(void *arg)
{
    ESP_LOGI(TAG, "Classification task ready");

    while (true) {
        xSemaphoreTake(classify_sem, portMAX_DELAY);
        classify_busy = 1;

        xSemaphoreTake(jpg_mutex, portMAX_DELAY);
        uint8_t *jpg  = classify_jpg;
        size_t   jlen = classify_jlen;
        classify_jpg  = NULL;
        classify_jlen = 0;
        xSemaphoreGive(jpg_mutex);
        if (!jpg || jlen == 0) { classify_busy = 0; continue; }

        int64_t t0 = esp_timer_get_time();
        ESP_LOGI(TAG, "Classifying (%u bytes)...", (unsigned)jlen);

        /* Base64 encode */
        size_t b64_len = 0;
        mbedtls_base64_encode(NULL, 0, &b64_len, jpg, jlen);
        char *b64 = heap_caps_malloc(b64_len + 1, MALLOC_CAP_SPIRAM);
        if (!b64) { free(jpg); classify_busy = 0; continue; }
        mbedtls_base64_encode((unsigned char *)b64, b64_len + 1, &b64_len, jpg, jlen);
        b64[b64_len] = '\0';
        free(jpg);

        /* Build JSON */
        const char *prompt =
            "Classify the main object: Cardboard, Plastic, Metal, or Trash. "
            "Reply with ONLY one word.";
        size_t jsz = strlen(prompt) + b64_len + 512;
        char *body = heap_caps_malloc(jsz, MALLOC_CAP_SPIRAM);
        if (!body) { free(b64); classify_busy = 0; continue; }
        snprintf(body, jsz,
            "{\"model\":\"%s\",\"messages\":[{\"role\":\"user\",\"content\":["
            "{\"type\":\"text\",\"text\":\"%s\"},"
            "{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,%s\"}}"
            "]}],\"max_tokens\":10}",
            OPENROUTER_MODEL, prompt, b64);
        free(b64);

        /* HTTP POST */
        resp_buf_t rb = {
            .buf = heap_caps_calloc(4096, 1, MALLOC_CAP_SPIRAM),
            .len = 0, .cap = 4096
        };
        if (!rb.buf) { free(body); classify_busy = 0; continue; }

        esp_http_client_config_t hcfg = {
            .url = OPENROUTER_URL, .method = HTTP_METHOD_POST,
            .timeout_ms = 30000, .event_handler = http_ev,
            .user_data = &rb, .crt_bundle_attach = esp_crt_bundle_attach,
        };
        esp_http_client_handle_t c = esp_http_client_init(&hcfg);
        if (!c) { free(body); free(rb.buf); classify_busy = 0; continue; }

        char auth[128];
        snprintf(auth, sizeof(auth), "Bearer %s", OPENROUTER_API_KEY);
        esp_http_client_set_header(c, "Content-Type", "application/json");
        esp_http_client_set_header(c, "Authorization", auth);
        esp_http_client_set_post_field(c, body, strlen(body));

        esp_err_t err = esp_http_client_perform(c);
        int status = esp_http_client_get_status_code(c);
        int ms = (int)((esp_timer_get_time() - t0) / 1000);

        if (err == ESP_OK && status == 200) {
            const char *label = extract_classification(rb.buf);
            strncpy(last_label, label, sizeof(last_label) - 1);
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "=== CLASSIFICATION: %s === (%d ms)", label, ms);
            ESP_LOGI(TAG, "");

            /* Machine-readable line for bridge script */
            printf("LABEL:%s\n", label);
            fflush(stdout);
        } else {
            ESP_LOGE(TAG, "API error: err=%d status=%d (%d ms)", err, status, ms);
            if (rb.len > 0) ESP_LOGE(TAG, "%.200s", rb.buf);
            strncpy(last_label, "error", sizeof(last_label) - 1);
        }

        esp_http_client_cleanup(c);
        free(body);
        free(rb.buf);
        classify_busy = 0;
    }
}

/* ── Serial input task ─────────────────────────────────── */

static void serial_input_task(void *arg)
{
    ESP_LOGI(TAG, "Type '1' + Enter in monitor to classify");
    char buf[16];
    while (true) {
        if (fgets(buf, sizeof(buf), stdin) != NULL) {
            char *p = buf;
            while (*p == ' ' || *p == '\t') p++;
            if (*p == '1') {
                if (!wifi_connected) {
                    ESP_LOGW(TAG, "WiFi not connected yet");
                } else if (classify_busy) {
                    ESP_LOGW(TAG, "Classification in progress...");
                } else {
                    ESP_LOGI(TAG, "Manual trigger from serial");
                    do_classify();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ── HTTP handlers ─────────────────────────────────────── */

#define BOUNDARY "fb0987654321"
static const char *STREAM_CT  = "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char *STREAM_SEP = "\r\n--" BOUNDARY "\r\n";
static const char *STREAM_HDR = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    char hdr[80];
    httpd_resp_set_type(req, STREAM_CT);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        int hlen = snprintf(hdr, sizeof(hdr), STREAM_HDR, (unsigned)fb->len);
        esp_err_t r = httpd_resp_send_chunk(req, STREAM_SEP, strlen(STREAM_SEP));
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, hdr, hlen);
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (r != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    return ESP_OK;
}

static esp_err_t classify_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (!wifi_connected) {
        httpd_resp_sendstr(req, "{\"ok\":false,\"msg\":\"WiFi not connected\"}");
        return ESP_OK;
    }
    if (classify_busy) {
        httpd_resp_sendstr(req, "{\"ok\":false,\"msg\":\"Busy\"}");
        return ESP_OK;
    }

    do_classify();
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t result_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[128];
    snprintf(buf, sizeof(buf), "{\"label\":\"%s\",\"busy\":%s}",
             last_label, classify_busy ? "true" : "false");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    const char *html =
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Waste Classifier</title>"
        "<style>"
        "*{box-sizing:border-box}"
        "body{margin:0;background:#0a0a0a;color:#fff;font-family:system-ui,sans-serif;"
        "display:flex;flex-direction:column;align-items:center;padding:12px}"
        "h2{margin:8px 0 4px;font-size:1.3em}"
        "img{width:100%;max-width:400px;border-radius:8px;border:2px solid #222}"
        "#btn{margin:14px 0;padding:16px 48px;font-size:1.4em;font-weight:700;"
        "border:none;border-radius:12px;cursor:pointer;color:#fff;"
        "background:linear-gradient(135deg,#00c853,#009624);"
        "box-shadow:0 4px 20px rgba(0,200,83,.4);transition:all .15s}"
        "#btn:hover{transform:scale(1.05);box-shadow:0 6px 28px rgba(0,200,83,.6)}"
        "#btn:active{transform:scale(.97)}"
        "#btn.busy{background:#555;pointer-events:none}"
        "#result{font-size:2em;font-weight:700;margin:10px 0;min-height:1.2em;"
        "text-transform:uppercase;letter-spacing:2px}"
        ".cardboard{color:#d4a24e}.plastic{color:#42a5f5}.metal{color:#bdbdbd}.trash{color:#ef5350}"
        ".sub{color:#666;font-size:.8em;margin-top:4px}"
        "</style></head><body>"
        "<h2>&#9851; Waste Classifier</h2>"
        "<img src='/stream'>"
        "<button id='btn' onclick='classify()'>CLASSIFY</button>"
        "<div id='result'></div>"
        "<p class='sub'>Or type <b>1</b> in serial monitor</p>"
        "<script>"
        "const btn=document.getElementById('btn'),res=document.getElementById('result');"
        "let polling=false;"
        "function classify(){"
        "  btn.textContent='Classifying...';btn.classList.add('busy');"
        "  fetch('/classify',{method:'POST'}).then(r=>r.json()).then(d=>{"
        "    if(!d.ok){btn.textContent='CLASSIFY';btn.classList.remove('busy');return;}"
        "    polling=true;poll();"
        "  }).catch(()=>{btn.textContent='CLASSIFY';btn.classList.remove('busy');});"
        "}"
        "function poll(){"
        "  if(!polling)return;"
        "  fetch('/result').then(r=>r.json()).then(d=>{"
        "    if(!d.busy){"
        "      res.textContent=d.label;res.className=d.label;"
        "      btn.textContent='CLASSIFY';btn.classList.remove('busy');"
        "      polling=false;"
        "    } else setTimeout(poll,500);"
        "  }).catch(()=>setTimeout(poll,1000));"
        "}"
        "</script></body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static void start_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 8;
    cfg.stack_size = 8192;

    httpd_handle_t srv = NULL;
    if (httpd_start(&srv, &cfg) != ESP_OK) return;

    httpd_uri_t u1 = {"/",         HTTP_GET,  index_handler,    NULL};
    httpd_uri_t u2 = {"/stream",   HTTP_GET,  stream_handler,   NULL};
    httpd_uri_t u3 = {"/classify", HTTP_POST, classify_handler,  NULL};
    httpd_uri_t u4 = {"/result",   HTTP_GET,  result_handler,   NULL};
    httpd_register_uri_handler(srv, &u1);
    httpd_register_uri_handler(srv, &u2);
    httpd_register_uri_handler(srv, &u3);
    httpd_register_uri_handler(srv, &u4);
    ESP_LOGI(TAG, "HTTP server started");
}

/* ── Main ──────────────────────────────────────────────── */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    classify_sem = xSemaphoreCreateBinary();
    jpg_mutex    = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(classify_task, "classify", 16384, NULL, 5, NULL, 1);
    xTaskCreate(serial_input_task, "serial_in", 4096, NULL, 3, NULL);

    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_ERROR_CHECK(esp_camera_init(&cam_cfg));
    ESP_LOGI(TAG, "Camera OK (JPEG QVGA)");

    start_server();

    ESP_LOGI(TAG, "=== Waste Classifier Ready ===");
    ESP_LOGI(TAG, "Press CLASSIFY on web UI or type '1' in monitor");
}
