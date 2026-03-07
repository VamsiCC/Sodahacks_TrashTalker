/**
 * ESP32-CAM Object Detection
 *
 * Detects can/bottle/box-sized objects using background subtraction,
 * draws green bounding boxes, and streams via MJPEG over HTTP.
 *
 * Connect to WiFi AP "ESP32-CAM" then open http://192.168.4.1/
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "img_converters.h"

static const char *TAG = "detect";

/* ── Detection tuning ──────────────────────────────────── */

#define IMG_W           320
#define IMG_H           240
#define CELL_SIZE       10
#define GRID_W          (IMG_W / CELL_SIZE)   /* 32 */
#define GRID_H          (IMG_H / CELL_SIZE)   /* 24 */

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

#define BOX_COLOR       0x07E0      /* bright green RGB565 */
#define BOX_THICK       2

/* ── State ─────────────────────────────────────────────── */

typedef struct {
    int x, y, w, h;
    int age;
    bool active;
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

/* ── WiFi AP ───────────────────────────────────────────── */

#define AP_SSID     "ESP32-CAM"
#define AP_PASS     ""
#define AP_MAX_CONN 2

static void wifi_ev(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED)
        ESP_LOGI(TAG, "Client connected");
    else if (id == WIFI_EVENT_AP_STADISCONNECTED)
        ESP_LOGI(TAG, "Client disconnected");
}

static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ev, NULL, NULL));

    wifi_config_t wcfg = {
        .ap = {
            .ssid           = AP_SSID,
            .ssid_len       = sizeof(AP_SSID) - 1,
            .channel        = 1,
            .password       = AP_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode       = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP '%s' started -> http://192.168.4.1/", AP_SSID);
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
    .frame_size   = FRAMESIZE_QVGA,    /* 320x240 */
    .jpeg_quality = 12,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
};

/* ── Image processing ──────────────────────────────────── */

static void to_gray(const uint8_t *src, uint8_t *dst, int n)
{
    for (int i = 0; i < n; i++) {
        uint8_t hi = src[i * 2];
        uint8_t lo = src[i * 2 + 1];
        uint8_t r5 = hi >> 3;
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
            ESP_LOGI(TAG, "Background ready (%d frames)", bg_count);
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
                    if (x < x0) { x0 = x; }
                    if (x > x1) { x1 = x; }
                    if (y < y0) { y0 = y; }
                    if (y > y1) { y1 = y; }
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
            if (dets[i].age < -DECAY_FRAMES) dets[i].active = false;
        }
    }
    for (int j = 0; j < ncnt; j++) {
        if (used[j]) continue;
        for (int i = 0; i < MAX_DETECTIONS; i++)
            if (!dets[i].active) {
                dets[i] = ndet[j];
                dets[i].age = 1;
                dets[i].active = true;
                break;
            }
    }
}

static void draw_rect(uint16_t *buf, int rx, int ry, int rw, int rh,
                       uint16_t color, int thick)
{
    for (int t = 0; t < thick; t++) {
        int y0 = ry - t, y1 = ry + rh - 1 + t;
        int x0 = rx - t, x1 = rx + rw - 1 + t;
        for (int px = x0; px <= x1; px++) {
            if (y0 >= 0 && y0 < IMG_H && px >= 0 && px < IMG_W)
                buf[y0 * IMG_W + px] = color;
            if (y1 >= 0 && y1 < IMG_H && px >= 0 && px < IMG_W)
                buf[y1 * IMG_W + px] = color;
        }
        for (int py = y0; py <= y1; py++) {
            if (py >= 0 && py < IMG_H && x0 >= 0 && x0 < IMG_W)
                buf[py * IMG_W + x0] = color;
            if (py >= 0 && py < IMG_H && x1 >= 0 && x1 < IMG_W)
                buf[py * IMG_W + x1] = color;
        }
    }
}

static void process_frame(uint8_t *rgb565)
{
    int npix = IMG_W * IMG_H;
    to_gray(rgb565, cur_gray, npix);
    update_bg(cur_gray, npix);
    if (!bg_ready) return;

    fg_detect(cur_gray, bg_gray, fg_mask, npix);
    morph_open(fg_mask, morph_tmp, IMG_W, IMG_H);
    build_grid(fg_mask, IMG_W);

    det_t ndet[MAX_DETECTIONS];
    int nc = find_boxes(ndet, MAX_DETECTIONS);
    track(ndet, nc);

    uint16_t *px = (uint16_t *)rgb565;
    for (int i = 0; i < MAX_DETECTIONS; i++)
        if (dets[i].active && dets[i].age >= PERSIST_FRAMES)
            draw_rect(px, dets[i].x, dets[i].y,
                      dets[i].w, dets[i].h, BOX_COLOR, BOX_THICK);
}

/* ── HTTP ──────────────────────────────────────────────── */

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

        process_frame(fb->buf);

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
    }
    return ESP_OK;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }

    process_frame(fb->buf);
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
        "<title>ESP32-CAM Detect</title>"
        "<style>"
        "body{margin:0;background:#111;display:flex;flex-direction:column;"
        "align-items:center;justify-content:center;min-height:100vh;color:#fff;"
        "font-family:sans-serif}"
        "img{max-width:100%;border:2px solid #333;border-radius:8px}"
        "h2{margin-bottom:8px}"
        ".sub{color:#888;font-size:.85em;margin-top:6px}"
        "a{color:#4fc3f7;margin-top:8px}"
        "</style></head><body>"
        "<h2>&#128247; ESP32-CAM Object Detection</h2>"
        "<img src='/stream'>"
        "<p class='sub'>Green boxes = can/bottle/box-sized objects detected</p>"
        "<p class='sub'>Tip: let camera see empty scene for ~2 sec first</p>"
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

    httpd_uri_t u1 = {"/",       HTTP_GET, index_handler,   NULL};
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

    ESP_ERROR_CHECK(esp_camera_init(&cam_cfg));
    ESP_LOGI(TAG, "Camera OK (RGB565 QVGA)");

    wifi_init_ap();
    start_server();

    ESP_LOGI(TAG, "Ready! Connect to '%s', open http://192.168.4.1/", AP_SSID);
    ESP_LOGI(TAG, "Let camera see empty scene for ~2s to learn background.");
}