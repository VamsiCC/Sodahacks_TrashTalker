/**
 * ESP32-CAM Waste Classification
 *
 * 1. Learns background for ~2 seconds after boot.
 * 2. Detects when a new object enters the scene and stabilises.
 * 3. Captures a JPEG snapshot.
 * 4. Sends it to an OpenRouter viwsion model via HTTPS.
 * 5. Logs the classification: Cardboard / Plastic / Metal / Trash.
 *
 * WiFi: STA-only mode (eduroam WPA2-Enterprise).
 *        Stream available at the IP shown in serial monitor.
 *
 * ── CONFIGURE THESE BEFORE BUILDING ──────────────────────
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
#include "esp_eap_client.h"

static const char *TAG = "classify";

/* ══════════════════════════════════════════════════════════
 *  USER CONFIGURATION — edit these!
 * ══════════════════════════════════════════════════════════ */

/* ── Phone hotspot (active) ─────────────────────────────
 * Set your iPhone hotspot name and password here.        */
#define WIFI_STA_SSID       "Vamsi\xe2\x80\x99s iPhone (2)"
#define WIFI_STA_PASS       "i7A1-T9DB-Dvcb-Nd86"
#define USE_PHONE_HOTSPOT   1

/* ── eduroam (uncomment below & comment hotspot above) ──
 * #define WIFI_STA_SSID       "eduroam"
 * #define EAP_IDENTITY        "vchilamkur@berkeley.edu"
 * #define EAP_PASSWORD        "your_calnet_password"
 */

/* OpenRouter API */
#define OPENROUTER_API_KEY  "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"
#define OPENROUTER_MODEL    "google/gemini-2.5-flash-lite"

/* ══════════════════════════════════════════════════════════ */

/* ── Detection tuning ──────────────────────────────────── */

#define IMG_W           320
#define IMG_H           240
#define CELL_SIZE       10
#define GRID_W          (IMG_W / CELL_SIZE)
#define GRID_H          (IMG_H / CELL_SIZE)

#define BG_LEARN_FRAMES 30
#define BG_ALPHA        16
#define FG_THRESHOLD    30
#define CELL_FG_PCT     35

#define MAX_DETECTIONS  8
#define MIN_OBJ_W       25
#define MIN_OBJ_H       35
#define MAX_OBJ_W       200
#define MAX_OBJ_H       220
#define PERSIST_FRAMES  3
#define DECAY_FRAMES    2

/* How many consecutive frames an object must be stable before
   we trigger an AI classification request */
#define STABLE_FRAMES_TO_CLASSIFY  15

/* Minimum seconds between classification attempts */
#define CLASSIFY_COOLDOWN_SEC      5

/* ── State ─────────────────────────────────────────────── */

typedef struct {
    int x, y, w, h;
    int age;
    bool active;
    bool classified;
} det_t;

static uint8_t *bg_gray   = NULL;
static uint8_t *cur_gray  = NULL;
static uint8_t *fg_mask   = NULL;
static uint8_t *morph_tmp = NULL;

static uint8_t  cell_grid[GRID_H][GRID_W];
static int8_t   cell_lbl[GRID_H][GRID_W];

static det_t    dets[MAX_DETECTIONS];
static int      bg_count  = 0;
static bool     bg_ready  = false;

/* Classification task communication */
static SemaphoreHandle_t  classify_sem  = NULL;
static uint8_t           *classify_jpg  = NULL;
static size_t             classify_jlen = 0;
static SemaphoreHandle_t  jpg_mutex     = NULL;

static bool     wifi_sta_connected = false;
static int64_t  last_classify_time = 0;

/* ── WiFi (AP+STA) ────────────────────────────────────── */

static void wifi_ev(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            /* Don't auto-connect here; we connect manually after scan */
            ESP_LOGI(TAG, "STA: started");
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *dis = (wifi_event_sta_disconnected_t *)data;
            ESP_LOGW(TAG, "STA: disconnected (reason=%d), reconnecting...", dis->reason);
            wifi_sta_connected = false;
            vTaskDelay(pdMS_TO_TICKS(2000));  /* 2 s backoff */
            esp_wifi_connect();
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA: got IP " IPSTR, IP2STR(&evt->ip_info.ip));
        ESP_LOGI(TAG, "Stream URL:  http://" IPSTR "/", IP2STR(&evt->ip_info.ip));
        wifi_sta_connected = true;
    }
}

static void wifi_scan_networks(void)
{
    ESP_LOGI(TAG, "=== Scanning for nearby WiFi networks ===");
    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);  /* blocking */
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi scan failed: %s", esp_err_to_name(err));
        return;
    }
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    ESP_LOGI(TAG, "Found %d networks:", ap_count);
    if (ap_count == 0) return;
    if (ap_count > 20) ap_count = 20;  /* limit */
    wifi_ap_record_t *ap_list = malloc(ap_count * sizeof(wifi_ap_record_t));
    if (!ap_list) return;
    esp_wifi_scan_get_ap_records(&ap_count, ap_list);
    bool found_target = false;
    for (int i = 0; i < ap_count; i++) {
        const char *ssid = (const char *)ap_list[i].ssid;
        ESP_LOGI(TAG, "  [%2d] RSSI:%4d  Ch:%2d  \"%s\"",
                 i+1, ap_list[i].rssi, ap_list[i].primary, ssid);
        if (strcmp(ssid, WIFI_STA_SSID) == 0) found_target = true;
    }
    if (found_target) {
        ESP_LOGI(TAG, ">>> Target SSID '%s' FOUND! <<<", WIFI_STA_SSID);
    } else {
        ESP_LOGW(TAG, ">>> Target SSID '%s' NOT FOUND in scan! <<<", WIFI_STA_SSID);
        ESP_LOGW(TAG, "Make sure hotspot is ON and discoverable (keep Settings>Personal Hotspot open on phone)");
    }
    free(ap_list);
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

    /* Start WiFi in STA mode first so we can scan */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* Temporary empty config just to start WiFi for scanning */
    wifi_config_t empty_cfg = { 0 };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &empty_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Scan and show all visible networks */
    wifi_scan_networks();

    /* Now set the real STA config */
    wifi_config_t sta_cfg = { 0 };
    strncpy((char *)sta_cfg.sta.ssid, WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid));

#ifdef USE_PHONE_HOTSPOT
    /* WPA-Personal for phone hotspot */
    strncpy((char *)sta_cfg.sta.password, WIFI_STA_PASS, sizeof(sta_cfg.sta.password));
#endif

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

#ifndef USE_PHONE_HOTSPOT
    /* WPA2-Enterprise (EAP-PEAP / MSCHAPv2) for eduroam */
    ESP_ERROR_CHECK(esp_eap_client_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)));
    ESP_ERROR_CHECK(esp_eap_client_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)));
    ESP_ERROR_CHECK(esp_eap_client_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD)));
    ESP_ERROR_CHECK(esp_eap_client_set_disable_time_check(true));
    ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());
#endif

    /* WiFi already started for scan — just connect with new config */
    ESP_LOGI(TAG, "STA connecting to '%s'...", WIFI_STA_SSID);
    esp_wifi_connect();
}

/* ── Camera (AI-Thinker ESP32-CAM, OV2640) ─────────────── */

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

    .pixel_format = PIXFORMAT_RGB565,
    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
};

/* ── Image processing ──────────────────────────────────── */

static void to_gray(const uint8_t *src, uint8_t *dst, int n)
{
    for (int i = 0; i < n; i++) {
        uint8_t lo = src[i * 2];
        uint8_t hi = src[i * 2 + 1];
        uint8_t r5 = (hi >> 3) & 0x1F;
        uint8_t g6 = ((hi & 0x07) << 3) | (lo >> 5);
        uint8_t b5 = lo & 0x1F;
        dst[i] = (uint8_t)(((r5 << 3) * 77 + (g6 << 2) * 150 + (b5 << 3) * 29) >> 8);
    }
}

static void update_bg(const uint8_t *gray, int n)
{
    if (!bg_ready) {
        if (bg_count == 0) {
            memcpy(bg_gray, gray, n);
        } else {
            for (int i = 0; i < n; i++)
                bg_gray[i] = (uint8_t)(((int)bg_gray[i] * bg_count + gray[i]) / (bg_count + 1));
        }
        if (++bg_count >= BG_LEARN_FRAMES) {
            bg_ready = true;
            ESP_LOGI(TAG, "✅ Background learned (%d frames). Place an object to classify!", bg_count);
        } else if (bg_count % 10 == 0) {
            ESP_LOGI(TAG, "📷 Learning background... %d/%d frames", bg_count, BG_LEARN_FRAMES);
        }
    } else {
        for (int i = 0; i < n; i++) {
            int d = (int)gray[i] - (int)bg_gray[i];
            if (abs(d) < FG_THRESHOLD)
                bg_gray[i] += d / BG_ALPHA;
        }
    }
}

static void fg_detect(const uint8_t *gray, const uint8_t *bg, uint8_t *mask, int n)
{
    for (int i = 0; i < n; i++)
        mask[i] = (abs((int)gray[i] - (int)bg[i]) > FG_THRESHOLD) ? 255 : 0;
}

static void morph_open(uint8_t *mask, uint8_t *tmp, int w, int h)
{
    memset(tmp, 0, w * h);
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++)
            if (mask[y*w+x] && mask[(y-1)*w+x] && mask[(y+1)*w+x] &&
                mask[y*w+x-1] && mask[y*w+x+1])
                tmp[y*w+x] = 255;

    memset(mask, 0, w * h);
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++)
            if (tmp[y*w+x]) {
                mask[y*w+x] = mask[(y-1)*w+x] = mask[(y+1)*w+x] =
                mask[y*w+x-1] = mask[y*w+x+1] = 255;
            }
}

static void build_grid(const uint8_t *mask, int w)
{
    int thr = (CELL_SIZE * CELL_SIZE * CELL_FG_PCT) / 100;
    memset(cell_grid, 0, sizeof(cell_grid));
    for (int cy = 0; cy < GRID_H; cy++)
        for (int cx = 0; cx < GRID_W; cx++) {
            int cnt = 0, by = cy * CELL_SIZE, bx = cx * CELL_SIZE;
            for (int dy = 0; dy < CELL_SIZE; dy++)
                for (int dx = 0; dx < CELL_SIZE; dx++)
                    if (mask[(by + dy) * w + bx + dx]) cnt++;
            cell_grid[cy][cx] = (cnt >= thr) ? 1 : 0;
        }
}

static void flood(int sy, int sx, int label)
{
    static int16_t stk[GRID_W * GRID_H * 2];
    int top = 0;
    stk[top++] = sy; stk[top++] = sx;
    cell_lbl[sy][sx] = label;
    const int dy[] = {-1, 1, 0, 0};
    const int dx[] = {0, 0, -1, 1};
    while (top > 0) {
        int x = stk[--top], y = stk[--top];
        for (int d = 0; d < 4; d++) {
            int ny = y + dy[d], nx = x + dx[d];
            if (ny >= 0 && ny < GRID_H && nx >= 0 && nx < GRID_W &&
                cell_grid[ny][nx] && cell_lbl[ny][nx] == 0) {
                cell_lbl[ny][nx] = label;
                stk[top++] = ny; stk[top++] = nx;
            }
        }
    }
}

static int find_boxes(det_t *out, int max)
{
    memset(cell_lbl, 0, sizeof(cell_lbl));
    int label = 0;
    for (int y = 0; y < GRID_H; y++)
        for (int x = 0; x < GRID_W; x++)
            if (cell_grid[y][x] && cell_lbl[y][x] == 0) {
                if (++label > 126) goto done;
                flood(y, x, label);
            }
done:;
    int cnt = 0;
    for (int l = 1; l <= label && cnt < max; l++) {
        int x0 = GRID_W, y0 = GRID_H, x1 = 0, y1 = 0;
        bool found = false;
        for (int y = 0; y < GRID_H; y++)
            for (int x = 0; x < GRID_W; x++)
                if (cell_lbl[y][x] == l) {
                    if (x < x0) x0 = x;
                    if (x > x1) x1 = x;
                    if (y < y0) y0 = y;
                    if (y > y1) y1 = y;
                    found = true;
                }
        if (!found) continue;
        int pw = (x1 - x0 + 1) * CELL_SIZE;
        int ph = (y1 - y0 + 1) * CELL_SIZE;
        if (pw >= MIN_OBJ_W && pw <= MAX_OBJ_W &&
            ph >= MIN_OBJ_H && ph <= MAX_OBJ_H) {
            out[cnt].x = x0 * CELL_SIZE;
            out[cnt].y = y0 * CELL_SIZE;
            out[cnt].w = pw;
            out[cnt].h = ph;
            out[cnt].active = true;
            out[cnt].age = 0;
            out[cnt].classified = false;
            cnt++;
        }
    }
    return cnt;
}

static void track(det_t *ndet, int ncnt)
{
    bool used[MAX_DETECTIONS] = {false};
    for (int i = 0; i < MAX_DETECTIONS; i++) {
        if (!dets[i].active) continue;
        bool matched = false;
        for (int j = 0; j < ncnt; j++) {
            if (used[j]) continue;
            int dcx = abs((dets[i].x + dets[i].w/2) - (ndet[j].x + ndet[j].w/2));
            int dcy = abs((dets[i].y + dets[i].h/2) - (ndet[j].y + ndet[j].h/2));
            if (dcx < (dets[i].w + ndet[j].w)/3 &&
                dcy < (dets[i].h + ndet[j].h)/3) {
                dets[i].x = ndet[j].x; dets[i].y = ndet[j].y;
                dets[i].w = ndet[j].w; dets[i].h = ndet[j].h;
                dets[i].age++;
                used[j] = true; matched = true; break;
            }
        }
        if (!matched) {
            dets[i].age--;
            if (dets[i].age < -DECAY_FRAMES) {
                dets[i].active = false;
                dets[i].classified = false;
            }
        }
    }
    for (int j = 0; j < ncnt; j++) {
        if (used[j]) continue;
        for (int i = 0; i < MAX_DETECTIONS; i++)
            if (!dets[i].active) {
                dets[i] = ndet[j];
                dets[i].age = 1;
                dets[i].active = true;
                dets[i].classified = false;
                break;
            }
    }
}

/* ── Trigger snapshot for classification ───────────────── */

static void maybe_trigger_classify(uint8_t *rgb565_buf, size_t buf_len,
                                   int width, int height)
{
    if (!wifi_sta_connected) {
        static int wifi_warn = 0;
        if (++wifi_warn >= 100) { wifi_warn = 0; ESP_LOGW(TAG, "⏳ Waiting for WiFi to connect before classifying..."); }
        return;
    }

    int64_t now_us = esp_timer_get_time();
    if ((now_us - last_classify_time) < (CLASSIFY_COOLDOWN_SEC * 1000000LL))
        return;

    bool need_classify = false;
    for (int i = 0; i < MAX_DETECTIONS; i++) {
        if (dets[i].active &&
            dets[i].age >= STABLE_FRAMES_TO_CLASSIFY &&
            !dets[i].classified) {
            need_classify = true;
            dets[i].classified = true;
            ESP_LOGI(TAG, "🎯 Object stable at (%d,%d %dx%d) for %d frames — triggering classification!",
                     dets[i].x, dets[i].y, dets[i].w, dets[i].h, dets[i].age);
        }
    }
    if (!need_classify) return;

    uint8_t *jpg = NULL;
    size_t jlen = 0;
    bool ok = fmt2jpg(rgb565_buf, buf_len, width, height,
                      PIXFORMAT_RGB565, 80, &jpg, &jlen);
    if (!ok || !jpg) {
        ESP_LOGW(TAG, "JPEG encode failed for classify");
        return;
    }

    if (xSemaphoreTake(jpg_mutex, 0) == pdTRUE) {
        if (classify_jpg) free(classify_jpg);
        classify_jpg  = jpg;
        classify_jlen = jlen;
        xSemaphoreGive(jpg_mutex);

        last_classify_time = now_us;
        xSemaphoreGive(classify_sem);
        ESP_LOGI(TAG, "Snapshot queued (%u bytes JPEG)", (unsigned)jlen);
    } else {
        free(jpg);
    }
}

/* ── Process each frame ────────────────────────────────── */

static void process_frame(camera_fb_t *fb)
{
    int npix = IMG_W * IMG_H;
    to_gray(fb->buf, cur_gray, npix);
    update_bg(cur_gray, npix);
    if (!bg_ready) return;

    fg_detect(cur_gray, bg_gray, fg_mask, npix);
    morph_open(fg_mask, morph_tmp, IMG_W, IMG_H);
    build_grid(fg_mask, IMG_W);

    det_t ndet[MAX_DETECTIONS];
    int nc = find_boxes(ndet, MAX_DETECTIONS);
    track(ndet, nc);

    /* Log active detections periodically */
    static int log_counter = 0;
    if (++log_counter >= 30) {  /* every ~30 frames */
        log_counter = 0;
        int active = 0;
        for (int i = 0; i < MAX_DETECTIONS; i++)
            if (dets[i].active) active++;
        if (active > 0) {
            for (int i = 0; i < MAX_DETECTIONS; i++) {
                if (dets[i].active) {
                    ESP_LOGI(TAG, "👁 Object #%d at (%d,%d %dx%d) age=%d%s%s",
                             i, dets[i].x, dets[i].y, dets[i].w, dets[i].h,
                             dets[i].age,
                             dets[i].age >= STABLE_FRAMES_TO_CLASSIFY ? " STABLE" : "",
                             dets[i].classified ? " (already classified)" : "");
                }
            }
        }
    }

    /* No bounding boxes — just trigger classification when stable */
    maybe_trigger_classify(fb->buf, fb->len, fb->width, fb->height);
}

/* ── OpenRouter API classification task ────────────────── */

#define OPENROUTER_URL "https://openrouter.ai/api/v1/chat/completions"

typedef struct {
    char  *buf;
    size_t len;
    size_t cap;
} resp_buf_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
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
    const char *key = "\"content\"";
    const char *p = strstr(json, key);
    if (!p) return "Trash";
    p = strchr(p + strlen(key), '\"');
    if (!p) return "Trash";
    p++;

    if (strstr(p, "Cardboard") || strstr(p, "cardboard") || strstr(p, "CARDBOARD"))
        return "Cardboard";
    if (strstr(p, "Plastic") || strstr(p, "plastic") || strstr(p, "PLASTIC"))
        return "Plastic";
    if (strstr(p, "Metal") || strstr(p, "metal") || strstr(p, "METAL"))
        return "Metal";

    return "Trash";
}

static void classify_task(void *arg)
{
    ESP_LOGI(TAG, "Classification task started");

    while (true) {
        xSemaphoreTake(classify_sem, portMAX_DELAY);

        xSemaphoreTake(jpg_mutex, portMAX_DELAY);
        uint8_t *jpg  = classify_jpg;
        size_t   jlen = classify_jlen;
        classify_jpg  = NULL;
        classify_jlen = 0;
        xSemaphoreGive(jpg_mutex);

        if (!jpg || jlen == 0) continue;

        ESP_LOGI(TAG, "🔄 Processing snapshot (%u bytes JPEG)...", (unsigned)jlen);
        ESP_LOGI(TAG, "   Step 1/4: Base64 encoding...");

        /* Base64-encode the JPEG */
        size_t b64_len = 0;
        mbedtls_base64_encode(NULL, 0, &b64_len, jpg, jlen);

        char *b64 = heap_caps_malloc(b64_len + 1, MALLOC_CAP_SPIRAM);
        if (!b64) {
            ESP_LOGE(TAG, "Base64 alloc failed (%u bytes)", (unsigned)b64_len);
            free(jpg);
            continue;
        }
        mbedtls_base64_encode((unsigned char *)b64, b64_len + 1, &b64_len, jpg, jlen);
        b64[b64_len] = '\0';
        free(jpg);

        ESP_LOGI(TAG, "   Step 2/4: Building API request (%u bytes base64)...", (unsigned)b64_len);

        /* Build JSON payload */
        const char *prompt =
            "You are a waste sorting classifier. "
            "Look at this image and classify the main object into exactly ONE of these categories: "
            "Cardboard, Plastic, Metal, Trash. "
            "Plastic includes water bottles, plastic containers, etc. "
            "Metal includes soda cans, energy drink cans, aluminum, etc. "
            "Cardboard includes cardboard boxes, paper packaging, etc. "
            "Trash is the default for anything else. "
            "Respond with ONLY the single category word, nothing else.";

        size_t json_sz = strlen(prompt) + b64_len + 512;
        char *json_body = heap_caps_malloc(json_sz, MALLOC_CAP_SPIRAM);
        if (!json_body) {
            ESP_LOGE(TAG, "JSON body alloc failed");
            free(b64);
            continue;
        }

        snprintf(json_body, json_sz,
            "{"
                "\"model\":\"%s\","
                "\"messages\":[{"
                    "\"role\":\"user\","
                    "\"content\":["
                        "{\"type\":\"text\",\"text\":\"%s\"},"
                        "{\"type\":\"image_url\",\"image_url\":{"
                            "\"url\":\"data:image/jpeg;base64,%s\""
                        "}}"
                    "]"
                "}],"
                "\"max_tokens\":20"
            "}",
            OPENROUTER_MODEL, prompt, b64);

        free(b64);

        resp_buf_t rb = {
            .buf = heap_caps_calloc(4096, 1, MALLOC_CAP_SPIRAM),
            .len = 0,
            .cap = 4096,
        };
        if (!rb.buf) {
            ESP_LOGE(TAG, "Response buffer alloc failed");
            free(json_body);
            continue;
        }

        esp_http_client_config_t http_cfg = {
            .url            = OPENROUTER_URL,
            .method         = HTTP_METHOD_POST,
            .timeout_ms     = 30000,
            .event_handler  = http_event_handler,
            .user_data      = &rb,
            .crt_bundle_attach = esp_crt_bundle_attach,
        };

        esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
        if (!client) {
            ESP_LOGE(TAG, "HTTP client init failed");
            free(json_body); free(rb.buf);
            continue;
        }

        char auth_hdr[128];
        snprintf(auth_hdr, sizeof(auth_hdr), "Bearer %s", OPENROUTER_API_KEY);

        esp_http_client_set_header(client, "Content-Type",  "application/json");
        esp_http_client_set_header(client, "Authorization", auth_hdr);
        esp_http_client_set_post_field(client, json_body, strlen(json_body));

        ESP_LOGI(TAG, "   Step 3/4: Sending to OpenRouter (%s)...", OPENROUTER_MODEL);
        int64_t t0 = esp_timer_get_time();

        esp_err_t err = esp_http_client_perform(client);
        int status = esp_http_client_get_status_code(client);

        int elapsed_ms = (int)((esp_timer_get_time() - t0) / 1000);
        ESP_LOGI(TAG, "   Step 4/4: Got response in %d ms (status %d)", elapsed_ms, status);

        if (err == ESP_OK && status == 200) {
            const char *label = extract_classification(rb.buf);
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
            ESP_LOGI(TAG, "║  🏷  CLASSIFICATION: %-15s ║", label);
            ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
            ESP_LOGI(TAG, "");
        } else {
            ESP_LOGE(TAG, "❌ API request failed: err=%d status=%d", err, status);
            if (rb.len > 0) {
                ESP_LOGE(TAG, "Response: %.200s", rb.buf);
            }
        }

        esp_http_client_cleanup(client);
        free(json_body);
        free(rb.buf);
    }
}

/* ── HTTP stream / capture / index ─────────────────────── */

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

        process_frame(fb);

        uint8_t *jpg = NULL; size_t jlen = 0;
        bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                          PIXFORMAT_RGB565, 80, &jpg, &jlen);
        esp_camera_fb_return(fb);
        if (!ok) continue;

        int hlen = snprintf(hdr, sizeof(hdr), STREAM_HDR, (unsigned)jlen);
        esp_err_t r = httpd_resp_send_chunk(req, STREAM_SEP, strlen(STREAM_SEP));
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, hdr, hlen);
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)jpg, jlen);
        free(jpg);
        if (r != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    return ESP_OK;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }

    process_frame(fb);
    uint8_t *jpg = NULL; size_t jlen = 0;
    bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                      PIXFORMAT_RGB565, 90, &jpg, &jlen);
    esp_camera_fb_return(fb);
    if (!ok) { httpd_resp_send_500(req); return ESP_FAIL; }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req, (const char *)jpg, jlen);
    free(jpg);
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    const char *html =
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ESP32-CAM Classify</title>"
        "<style>"
        "body{margin:0;background:#111;display:flex;flex-direction:column;"
        "align-items:center;justify-content:center;min-height:100vh;color:#fff;"
        "font-family:sans-serif}"
        "img{max-width:100%;border:2px solid #333;border-radius:8px}"
        "h2{margin-bottom:8px}"
        ".sub{color:#888;font-size:.85em;margin-top:6px}"
        "a{color:#4fc3f7;margin-top:8px}"
        "</style></head><body>"
        "<h2>&#9851; ESP32-CAM Waste Classifier</h2>"
        "<img src='/stream'>"
        "<p class='sub'>Place an object in view &mdash; classification appears in serial monitor</p>"
        "<p class='sub'>Categories: Cardboard | Plastic | Metal | Trash</p>"
        "<p class='sub'>Let camera see empty scene for ~2 sec at startup</p>"
        "<a href='/capture'>&#128248; Snapshot</a>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static void start_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 4;
    cfg.stack_size = 8192;

    httpd_handle_t srv = NULL;
    if (httpd_start(&srv, &cfg) != ESP_OK) return;

    httpd_uri_t u1 = {"/",        HTTP_GET, index_handler,   NULL};
    httpd_uri_t u2 = {"/capture", HTTP_GET, capture_handler, NULL};
    httpd_uri_t u3 = {"/stream",  HTTP_GET, stream_handler,  NULL};
    httpd_register_uri_handler(srv, &u1);
    httpd_register_uri_handler(srv, &u2);
    httpd_register_uri_handler(srv, &u3);
    ESP_LOGI(TAG, "HTTP server on port %d", cfg.server_port);
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

    int n = IMG_W * IMG_H;
    bg_gray   = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM);
    cur_gray  = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM);
    fg_mask   = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM);
    morph_tmp = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM);
    if (!bg_gray || !cur_gray || !fg_mask || !morph_tmp) {
        ESP_LOGE(TAG, "PSRAM alloc failed");
        return;
    }
    memset(dets, 0, sizeof(dets));

    classify_sem = xSemaphoreCreateBinary();
    jpg_mutex    = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(classify_task, "classify", 16384, NULL, 5, NULL, 1);

    /* Init WiFi BEFORE camera to avoid NVS flash write vs camera DMA race */
    wifi_init_sta();

    /* Brief delay so WiFi config NVS writes complete before camera DMA starts */
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_ERROR_CHECK(esp_camera_init(&cam_cfg));
    ESP_LOGI(TAG, "Camera OK (RGB565 QVGA)");

    start_server();

    ESP_LOGI(TAG, "=== ESP32-CAM Waste Classifier ===");
    ESP_LOGI(TAG, "Stream available once WiFi connects (check IP above)");
    ESP_LOGI(TAG, "Model:   %s", OPENROUTER_MODEL);
    ESP_LOGI(TAG, "Let camera see empty scene for ~2s to learn background");
}
