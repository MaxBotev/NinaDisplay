/*
 * NINA Guiding Graph Display  — v4
 * Waveshare ESP32-S3-Touch-AMOLED-2.41  (600×450, SH8601 QSPI)
 *
 * Folder must contain ONLY these files (delete ui_Screen1.c / ui_Screen1.h):
 *   nina_guide_v4.ino
 *   ui.c / ui.h
 *   ui_uiGraphPanel.c / ui_uiGraphPanel.h
 *   ui_helpers.c / ui_helpers.h
 *   ui_events.h
 *   ui_comp_hook.c
 *   esp_lcd_sh8601.c / .h
 *   esp_lcd_touch_ft5x06.c / .h
 *   esp_lcd_touch.c / .h
 *
 * lv_conf.h must have:
 *   LV_COLOR_16_SWAP   0
 *   LV_FONT_MONTSERRAT_14  1
 *   LV_FONT_MONTSERRAT_16  1
 *   LV_FONT_MONTSERRAT_30  1
 *   LV_FONT_MONTSERRAT_36  1
 *
 * Board: ESP32S3 Dev Module, 16MB Flash, OPI PSRAM, 921600 baud
 *
 * ── USER CONFIG ─────────────────────────────────────────────────────────── */
// WiFi credentials are stored in NVS flash — configured via captive portal
#define NINA_PORT  1888
static char g_nina_host[16] = "";   // discovered/loaded at runtime
#define POLL_INTERVAL_MS  1000
#define NINA_MAX_HIST  256   // max image-history entries we mirror into the graph
/* ─────────────────────────────────────────────────────────────────────────── */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lcd_sh8601.h"
#include "esp_lcd_touch_ft5x06.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "ui.h"
#include "nina_screens.h"
// Mirror full NINA image history into the Screen3 graph (defined in nina_screens.cpp)
void screens_set_hfr_history(const float *hfr, const int *stars, int n);
// Mirror full session HFR + total-error into the Screen5 accumulated charts
void screens_set_sky_history(const float *hfr, const float *rms_arcsec, int n);
#include "wifi_manager.h"
#include "adc_bsp.h"

static const char *TAG = "nina_guide";
static volatile int   g_batt_pct = -1;
static volatile float g_batt_v   = 0;
// ── Pins (verbatim from 09_LVGL_Test.ino) ────────────────────────────────────
#define LCD_HOST          SPI2_HOST
#define TOUCH_HOST        I2C_NUM_0
#define LCD_BIT_PER_PIXEL 16
#define PIN_LCD_CS        GPIO_NUM_9
#define PIN_LCD_PCLK      GPIO_NUM_10
#define PIN_LCD_DATA0     GPIO_NUM_11
#define PIN_LCD_DATA1     GPIO_NUM_12
#define PIN_LCD_DATA2     GPIO_NUM_13
#define PIN_LCD_DATA3     GPIO_NUM_14
#define PIN_LCD_RST       GPIO_NUM_21
#define PIN_TOUCH_SCL     GPIO_NUM_48
#define PIN_TOUCH_SDA     GPIO_NUM_47
#define PIN_TOUCH_RST     ((gpio_num_t)(-1))
#define PIN_TOUCH_INT     ((gpio_num_t)(-1))
#define LCD_H_RES         600
#define LCD_V_RES         450

// ── LVGL / task ───────────────────────────────────────────────────────────────
#define LVGL_BUF_HEIGHT        (LCD_V_RES / 10)
#define LVGL_TICK_MS           2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK        (6 * 1024)
#define LVGL_TASK_PRIO         2
#define NINA_TASK_STACK        (8 * 1024)
#define NINA_TASK_PRIO         1

// ── SH8601 init (verbatim from 09_LVGL_Test.ino) ────────────────────────────
static const sh8601_lcd_init_cmd_t g_lcd_init_cmds[] = {
  {0xFE, (uint8_t[]){0x20}, 1, 0},
  {0x26, (uint8_t[]){0x0A}, 1, 0},
  {0x24, (uint8_t[]){0x80}, 1, 0},
  {0xFE, (uint8_t[]){0x00}, 1, 0},
  {0x3A, (uint8_t[]){0x55}, 1, 0},
  {0xC2, (uint8_t[]){0x00}, 1, 10},
  {0x35, (uint8_t[]){0x00}, 0, 0},
  {0x51, (uint8_t[]){0x00}, 1, 10},
  {0x11, (uint8_t[]){0x00}, 0, 80},
  {0x2A, (uint8_t[]){0x00,0x10,0x01,0xD1}, 4, 0},
  {0x2B, (uint8_t[]){0x00,0x00,0x02,0x57}, 4, 0},
  {0x29, (uint8_t[]){0x00}, 0, 10},
  {0x36, (uint8_t[]){0x30}, 1, 0},
  {0x53, (uint8_t[]){0x28}, 1, 0},   // WCTRLD1: brightness control ON + dimming ON
  {0x51, (uint8_t[]){0xFF}, 1, 0},
};
// ── Handles / mutexes ────────────────────────────────────────────────────────
static SemaphoreHandle_t    g_lvgl_mux = NULL;
static SemaphoreHandle_t    g_data_mux = NULL;
static esp_lcd_touch_handle_t g_tp     = NULL;

// ── Canvas — attached to ui_Panel2 (612×271 from SLS) ────────────────────────
// Panel2 padding zeroed in code before canvas creation, so canvas = full panel size
// We use exactly the panel's inner dimensions.
#define GW  612   // Panel2 full width (padding zeroed before canvas creation)
#define GH  271   // Panel2 full height (padding zeroed before canvas creation)

static lv_obj_t   *g_canvas     = NULL;
static lv_color_t *g_canvas_buf = NULL;

// ── Guide ring buffer ─────────────────────────────────────────────────────────
#define MAX_PTS  600  // 10 min at 1 step/sec, 20 min at PHD2 2s cadence

// Probe a single IP — returns true if NINA Advanced API answers
static bool nina_probe(const String &ip) {
  HTTPClient http;
  String url = "http://" + ip + ":" + String(NINA_PORT) + "/v2/api/version";
  http.begin(url);
  http.setConnectTimeout(300);   // fast fail for dead hosts
  http.setTimeout(400);
  int code = http.GET();
  http.end();
  return (code == 200);
}
// Patient probe — for verifying a known/saved host. Retries before giving up.
static bool nina_probe_patient(const String &ip) {
  for (int attempt = 0; attempt < 3; attempt++) {
    HTTPClient http;
    String url = "http://" + ip + ":" + String(NINA_PORT) + "/v2/api/version";
    http.begin(url);
    http.setConnectTimeout(2000);   // generous — known host may be slow
    http.setTimeout(2000);
    int code = http.GET();
    http.end();
    if (code == 200) return true;
    Serial.printf("[NINA] saved IP attempt %d/3 failed (code=%d), retrying\n",
      attempt+1, code);
    vTaskDelay(pdMS_TO_TICKS(1000));   // wait between retries
  }
  return false;
}

// Fast probe — for sweeping the subnet, dead hosts must fail quickly
static bool nina_probe_fast(const String &ip) {
  HTTPClient http;
  String url = "http://" + ip + ":" + String(NINA_PORT) + "/v2/api/version";
  http.begin(url);
  http.setConnectTimeout(300);
  http.setTimeout(400);
  int code = http.GET();
  http.end();
  return (code == 200);
}
// Scan the local /24 subnet for a NINA host. Returns "" if none found.
static String nina_scan_subnet() {
  IPAddress local = WiFi.localIP();
  if (local[0] == 0) return "";
  String prefix = String(local[0]) + "." + String(local[1]) + "." + String(local[2]) + ".";
  Serial.printf("[NINA] scanning %s0/24 ...\n", prefix.c_str());

  for (int host = 1; host <= 254; host++) {
    if (host == local[3]) continue;   // skip self
    String ip = prefix + String(host);
    if (nina_probe_fast(ip)) {        // fast probe — dead hosts fail quickly
      Serial.printf("[NINA] FOUND at %s\n", ip.c_str());
      return ip;
    }
    if (host % 20 == 0) {
      Serial.printf("[NINA] ...scanned through .%d\n", host);
      vTaskDelay(pdMS_TO_TICKS(1));   // yield so WiFi/LVGL breathe
    }
  }
  Serial.println("[NINA] not found on subnet");
  return "";
}
// Resolve NINA host: saved IP → probe → full scan → save
static void nina_resolve_host() {
  String saved = wm_load_nina_ip();
   if (saved.length() > 0) {
    Serial.printf("[NINA] verifying saved IP %s (patient)\n", saved.c_str());
    if (nina_probe_patient(saved)) {
      strncpy(g_nina_host, saved.c_str(), sizeof(g_nina_host)-1);
      Serial.printf("[NINA] saved IP works: %s\n", g_nina_host);
      return;
    }
    Serial.println("[NINA] saved IP truly dead after 3 tries — rescanning");
  }
  String found = nina_scan_subnet();
  if (found.length() > 0) {
    strncpy(g_nina_host, found.c_str(), sizeof(g_nina_host)-1);
    wm_save_nina_ip(found);
  }
}

static esp_lcd_panel_io_handle_t g_io_handle = NULL;

struct GuidePoint {
  float ra; float dec;
  float ra_dist; float dec_dist;
  bool  valid; bool dither;
};
static GuidePoint g_hist[MAX_PTS];
static int        g_head  = 0;
static int        g_count = 0;

struct GuiderStats {
  float rmsRA=0, rmsDec=0, rmsTotal=0;
  float rawRA=0, rawDec=0, rawTotal=0;
  bool  guiding=false, connected=false;
  char  state[32] = "---";
};
static GuiderStats g_stats;
static bool        g_need_update = false;

// ── LostLock flash ────────────────────────────────────────────────────────────
static bool     g_flash_active = false;
static uint32_t g_flash_end_ms = 0;
static char     g_prev_state[32] = "";
static lv_timer_t *g_flash_timer = NULL;

static void flash_timer_cb(lv_timer_t *t) {
  if (!g_flash_active || millis() >= g_flash_end_ms) {
    g_flash_active = false;
    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0x161B22), LV_PART_MAIN);
    if (g_flash_timer) { lv_timer_del(g_flash_timer); g_flash_timer = NULL; }
    return;
  }
  static bool red = false;
  red = !red;
  lv_obj_set_style_bg_color(ui_Panel1,
    red ? lv_color_hex(0x8B0000) : lv_color_hex(0x161B22), LV_PART_MAIN);
}

// ── Imaging data ──────────────────────────────────────────────────────────────
struct ImagingData {
  float hfr         = 0;
  int   star_count  = 0;
  int   subs_done   = 0;
  int   subs_total  = 0;
  int   subs_rej    = 0;
  float exp_elapsed = 0;
  float exp_total   = 0;
  char  filter[32]  = "--";
  char  target[64]  = "---";   // ← add this
  float cam_temp    = 0;
  float cooler_pct  = 0;
  float adu_mean    = 0;
};
static ImagingData g_imaging;

// ── Focuser data ──────────────────────────────────────────────────────────────
struct FocuserData {
  int   position     = 0;
  float temperature  = -99;
  bool  tc_enabled   = false;
  float tc_coeff     = 0;
  float last_hfr     = 0;
  int   last_steps   = 0;
  float last_temp    = -99;
};
static FocuserData g_focuser;

// ── Sky data ──────────────────────────────────────────────────────────────────
struct SkyData {
  char  target[64]   = "---";
  char  ra_str[32]   = "--h --m --s";
  char  dec_str[32]  = "--d --' --";
  float altitude     = 0;
  float azimuth      = 0;
  char  pier[16]     = "---";
  int   flip_min     = -1;
  int   subs_done    = 0;
  int   subs_total   = 0;
  int   remain_min   = -1;
};
static SkyData g_sky;

// X-axis zoom: how many points to show. 
// Controlled by Slider1 (1-100). 100 = full history, 1 = most zoomed in.
// Number of guide steps to show. 120 ≈ 5min at PHD2 2-3s cadence.
// Slider on Screen2 adjusts this. One point = one pixel, graph grows right→left.
static int g_x_zoom_pts = 120;

// ── Graph colours (NINA-style dark theme) ────────────────────────────────────
#define COL_BG        lv_color_hex(0x0D1117)
#define COL_GRID      lv_color_hex(0x21262D)
#define COL_ZERO      lv_color_hex(0x3A3F47)
#define COL_RA_BAR    lv_color_hex(0x1A3A6B)
#define COL_DEC_BAR   lv_color_hex(0x6B1A1A)
#define COL_RA_LINE   lv_color_hex(0x4472C4)
#define COL_DEC_LINE  lv_color_hex(0xE05050)
#define COL_DITHER    lv_color_hex(0x50C878)
#define COL_TICK      lv_color_hex(0x6A7380)

// ── Dynamic Y scale ───────────────────────────────────────────────────────────
static float g_scale_as = 2.0f;

static float nice_scale(float v) {
  // Round up to nearest 0.5" step for tight fit
  // e.g. max=0.62 -> scale=0.5 no, 1.0 yes... use 0.5 steps
  float stepped = ceilf(v * 2.0f) / 2.0f;  // round to nearest 0.5
  if (stepped < 0.5f) stepped = 0.5f;
  return stepped;
}

// Left margin for Y-axis labels
#define YAXIS_W  30
#define GRAPH_X  YAXIS_W
#define GRAPH_W  (GW - YAXIS_W)

static int errToY(float as, int h) {
  float sc = g_scale_as;
  float cl = (as < -sc) ? -sc : (as > sc) ? sc : as;
  return (int)((1.0f - (cl/sc + 1.0f)*0.5f) * (float)(h-4)) + 2;
}

// ── LVGL callbacks (verbatim from 09_LVGL_Test.ino) ─────────────────────────
static bool cb_flush_ready(esp_lcd_panel_io_handle_t io,
                            esp_lcd_panel_io_event_data_t *edata, void *ctx) {
  lv_disp_flush_ready((lv_disp_drv_t*)ctx);
  return false;
}

static void cb_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *map) {
  esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
  // Byte-swap every pixel — this panel requires it regardless of LV_COLOR_16_SWAP
  int px = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
  uint16_t *buf = (uint16_t*)map;
  for (int i = 0; i < px; i++) {
    uint16_t p = buf[i];
    buf[i] = (p >> 8) | (p << 8);
  }
  esp_lcd_panel_draw_bitmap(panel,
    area->x1, area->y1 + 16,
    area->x2 + 1, area->y2 + 17, map);
}

static void cb_rounder(lv_disp_drv_t *drv, lv_area_t *area) {
  area->x1 = (area->x1>>1)<<1; area->y1 = (area->y1>>1)<<1;
  area->x2 = ((area->x2>>1)<<1)+1; area->y2 = ((area->y2>>1)<<1)+1;
}

static void cb_disp_update(lv_disp_drv_t *drv) {
  esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
  switch (drv->rotated) {
    case LV_DISP_ROT_NONE:   esp_lcd_panel_swap_xy(panel,false); esp_lcd_panel_mirror(panel,true,false);  break;
    case LV_DISP_ROT_90:     esp_lcd_panel_swap_xy(panel,true);  esp_lcd_panel_mirror(panel,true,true);   break;
    case LV_DISP_ROT_180:    esp_lcd_panel_swap_xy(panel,false); esp_lcd_panel_mirror(panel,false,true);  break;
    case LV_DISP_ROT_270:    esp_lcd_panel_swap_xy(panel,true);  esp_lcd_panel_mirror(panel,false,false); break;
  }
}

static void cb_touch(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;
  uint16_t tx, ty; uint8_t cnt = 0;
  esp_lcd_touch_read_data(tp);
  bool pressed = esp_lcd_touch_get_coordinates(tp, &tx, &ty, NULL, &cnt, 1);
  if (pressed && cnt > 0) {
    data->point.x = tx; data->point.y = ty;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void cb_tick(void *arg) { lv_tick_inc(LVGL_TICK_MS); }

// ── LVGL task ────────────────────────────────────────────────────────────────
static void lvgl_task(void *arg) {
  uint32_t d = LVGL_TASK_MAX_DELAY_MS;
  while (1) {
    if (xSemaphoreTake(g_lvgl_mux, portMAX_DELAY) == pdTRUE) {
      d = lv_timer_handler();
      xSemaphoreGive(g_lvgl_mux);
    }
    if (d > LVGL_TASK_MAX_DELAY_MS) d = LVGL_TASK_MAX_DELAY_MS;
    if (d < LVGL_TASK_MIN_DELAY_MS) d = LVGL_TASK_MIN_DELAY_MS;
    vTaskDelay(pdMS_TO_TICKS(d));
  }
}
static bool lvgl_lock(int ms) {
  return xSemaphoreTake(g_lvgl_mux, ms<0?portMAX_DELAY:pdMS_TO_TICKS(ms))==pdTRUE;
}
static void lvgl_unlock() { xSemaphoreGive(g_lvgl_mux); }

// ── Graph draw ────────────────────────────────────────────────────────────────
static void graph_draw() {
  if (!g_canvas || !g_canvas_buf) return;
  int w = GW, h = GH;

  // Dynamic scale from visible data
  if (g_count > 0) {
    int dc = (g_count < g_x_zoom_pts) ? g_count : g_x_zoom_pts;
    if (dc < 1) dc = 1;
    float mx = 0.5f;
    for (int i = 0; i < dc; i++) {
      int idx = (g_head - dc + i + MAX_PTS) % MAX_PTS;
      if (!g_hist[idx].valid) continue;
      float ar = fabsf(g_hist[idx].ra), ad = fabsf(g_hist[idx].dec);
      if (ar > mx) mx = ar;
      if (ad > mx) mx = ad;
    }
    g_scale_as = nice_scale(mx * 1.05f);
    if (g_scale_as < 1.0f) g_scale_as = 1.0f;
  }

  lv_canvas_fill_bg(g_canvas, COL_BG, LV_OPA_COVER);

  // Y-axis labels + ticks
  lv_draw_label_dsc_t lbl;
  lv_draw_label_dsc_init(&lbl);
  lbl.color = COL_TICK;
  lbl.font  = &lv_font_montserrat_16;
  lbl.align = LV_TEXT_ALIGN_RIGHT;

  lv_draw_line_dsc_t tick_d;
  lv_draw_line_dsc_init(&tick_d);
  tick_d.color = COL_TICK; tick_d.width = 1; tick_d.opa = LV_OPA_COVER;

  // Draw ticks, labels and grid at every 0.5" step
  // Use float steps so we handle half-arcsec scales cleanly
  float step = (g_scale_as <= 1.0f) ? 0.5f : 1.0f;  // 0.5" steps when zoomed in
  lv_draw_line_dsc_t grid_d;
  lv_draw_line_dsc_init(&grid_d);
  grid_d.opa = LV_OPA_COVER;

  for (float as = -g_scale_as; as <= g_scale_as + 0.01f; as += step) {
    int y = errToY(as, h);

    // Tick mark
    lv_point_t tp[2] = {{YAXIS_W-5, y},{YAXIS_W, y}};
    lv_canvas_draw_line(g_canvas, tp, 2, &tick_d);

    // Label — show integers and halves (e.g. "1", "1.5")
    char num[8];
    if (fabsf(as - roundf(as)) < 0.01f) {
      snprintf(num, sizeof(num), "%d", (int)roundf(as));
    } else {
      snprintf(num, sizeof(num), "%.1f", as);
    }
    lv_canvas_draw_text(g_canvas, 0, (lv_coord_t)(y-9), YAXIS_W-4, &lbl, num);

    // Grid line
    grid_d.color = (fabsf(as) < 0.01f) ? COL_ZERO : COL_GRID;
    grid_d.width = (fabsf(as) < 0.01f) ? 2 : 1;
    lv_point_t gp[2] = {{GRAPH_X, y},{w-1, y}};
    lv_canvas_draw_line(g_canvas, gp, 2, &grid_d);
  }

  if (g_count == 0) return;

  // g_x_zoom_pts = total points that represent the full window width (e.g. 120 = 5min).
  // Each point occupies GRAPH_W/g_x_zoom_pts pixels so the window always fills the panel.
  // While filling up, data grows from right; empty space stays on the left.
  // Once full, oldest points scroll off the left edge.
  int drawCount = (g_count < g_x_zoom_pts) ? g_count : g_x_zoom_pts;
  // pixels per point — constant regardless of how many points we have so far
  float pxPer = (float)GRAPH_W / (float)g_x_zoom_pts;
  // newest point sits at the right edge; older points extend left from there
  // point i=drawCount-1 is newest → right edge
  // point i=0 is oldest visible → left of the data region
  // x of point i = GRAPH_X + GRAPH_W - (drawCount - i) * pxPer
  int zeroY = errToY(0.0f, h);

  // Correction bars
  lv_draw_rect_dsc_t bar_d;
  lv_draw_rect_dsc_init(&bar_d);
  bar_d.border_width = 0; bar_d.radius = 0;
  int barW = (int)pxPer; if (barW < 1) barW = 1;

  for (int i = 0; i < drawCount; i++) {
    int idx = (g_head - drawCount + i + MAX_PTS) % MAX_PTS;
    if (!g_hist[idx].valid) continue;
    int x = GRAPH_X + (int)(GRAPH_W - (drawCount - i) * pxPer);
    if (g_hist[idx].ra_dist != 0.0f) {
      int by = errToY(g_hist[idx].ra_dist, h);
      int top = (by<zeroY)?by:zeroY, bot = (by<zeroY)?zeroY:by;
      bar_d.bg_color = COL_RA_BAR; bar_d.bg_opa = LV_OPA_70;
      lv_canvas_draw_rect(g_canvas, x, top, barW, bot-top+1, &bar_d);
    }
    if (g_hist[idx].dec_dist != 0.0f) {
      int by = errToY(g_hist[idx].dec_dist, h);
      int top = (by<zeroY)?by:zeroY, bot = (by<zeroY)?zeroY:by;
      bar_d.bg_color = COL_DEC_BAR; bar_d.bg_opa = LV_OPA_70;
      lv_canvas_draw_rect(g_canvas, x, top, barW, bot-top+1, &bar_d);
    }
  }

  // Error line traces
  lv_draw_line_dsc_t ra_d, dec_d;
  lv_draw_line_dsc_init(&ra_d); lv_draw_line_dsc_init(&dec_d);
  ra_d.color  = COL_RA_LINE;  ra_d.width  = 2; ra_d.opa  = LV_OPA_COVER;
  dec_d.color = COL_DEC_LINE; dec_d.width = 2; dec_d.opa = LV_OPA_COVER;

  int prevRAy=-1, prevDecY=-1, prevX=-1;
  for (int i = 0; i < drawCount; i++) {
    int idx = (g_head - drawCount + i + MAX_PTS) % MAX_PTS;
    if (!g_hist[idx].valid) { prevRAy=prevDecY=-1; prevX=-1; continue; }
    int x    = GRAPH_X + (int)(GRAPH_W - (drawCount - i) * pxPer);
    int ray  = errToY(g_hist[idx].ra,  h);
    int decy = errToY(g_hist[idx].dec, h);
    if (g_hist[idx].dither) {
      lv_draw_line_dsc_t dit; lv_draw_line_dsc_init(&dit);
      dit.color=COL_DITHER; dit.width=1; dit.opa=LV_OPA_80;
      lv_point_t dp[2]={{(lv_coord_t)x,0},{(lv_coord_t)x,(lv_coord_t)(h-1)}};
      lv_canvas_draw_line(g_canvas, dp, 2, &dit);
    }
    if (prevRAy>=0 && prevX>=0) {
      lv_point_t p[2]={{(lv_coord_t)prevX,(lv_coord_t)prevRAy},{(lv_coord_t)x,(lv_coord_t)ray}};
      lv_canvas_draw_line(g_canvas, p, 2, &ra_d);
    }
    if (prevDecY>=0 && prevX>=0) {
      lv_point_t p[2]={{(lv_coord_t)prevX,(lv_coord_t)prevDecY},{(lv_coord_t)x,(lv_coord_t)decy}};
      lv_canvas_draw_line(g_canvas, p, 2, &dec_d);
    }
    prevRAy=ray; prevDecY=decy; prevX=x;
  }
}

// ── UI refresh timer (200ms) ──────────────────────────────────────────────────
static void ui_refresh_cb(lv_timer_t *t) {
  if (!g_need_update) return;
  g_need_update = false;

  lv_obj_t *active = lv_scr_act();
  bool on_guiding  = (active == ui_uiGraphPanel);  // Screen1
  bool on_imaging  = (active == ui_Screen3);
  bool on_focuser  = (active == ui_Screen4);
  bool on_sky      = (active == ui_Screen5);
  screens_update_battery_bars(g_batt_pct, g_batt_v);
  // ── Guiding labels + graph — only when on Screen1 ────────────────────────
  if (on_guiding) {
    GuiderStats snap;
    if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
      snap = g_stats;
      xSemaphoreGive(g_data_mux);
    } else return;

    char buf[64];
    snprintf(buf, sizeof(buf), "State: %s", snap.state);
    lv_label_set_text(ui_Status, buf);
    lv_obj_set_style_text_color(ui_Status,
      strcmp(snap.state, "Guiding") == 0 ? lv_color_hex(0x3FB950) : lv_color_hex(0xF85149),
      LV_PART_MAIN);

    if (strcmp(snap.state, "LostLock") == 0 && strcmp(g_prev_state, "LostLock") != 0) {
      g_flash_active = true;
      g_flash_end_ms = millis() + 4000;
      if (!g_flash_timer) {
        g_flash_timer = lv_timer_create(flash_timer_cb, 150, NULL);
      }
    }
    strncpy(g_prev_state, snap.state, sizeof(g_prev_state)-1);

    snprintf(buf, sizeof(buf), "RA: %.2f (%.2f\")", snap.rawRA, snap.rmsRA);
    lv_label_set_text(ui_RALabel, buf);

    snprintf(buf, sizeof(buf), "Dec: %.2f (%.2f\")", snap.rawDec, snap.rmsDec);
    lv_label_set_text(ui_DECLabel, buf);

    snprintf(buf, sizeof(buf), "Total: %.2f (%.2f\")", snap.rawTotal, snap.rmsTotal);
    lv_label_set_text(ui_TOTLabel, buf);

    if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
      graph_draw();
      xSemaphoreGive(g_data_mux);
    }
  }

  // ── Imaging screen — only when on Screen3 ────────────────────────────────
  if (on_imaging) {
    ImagingData img_snap;
    if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(50)) == pdTRUE) {
      img_snap = g_imaging;
      xSemaphoreGive(g_data_mux);
    }
    Serial.printf("[IMG] update: HFR=%.2f stars=%d subs=%d/%d\n",
      img_snap.hfr, img_snap.star_count, img_snap.subs_done, img_snap.subs_total);
    screens_update_imaging(
      img_snap.hfr, img_snap.star_count, img_snap.subs_total,
      img_snap.subs_rej, img_snap.exp_elapsed, img_snap.exp_total,
      img_snap.filter, img_snap.cam_temp, img_snap.cooler_pct, img_snap.adu_mean);
  }

  // ── Focuser screen — only when on Screen4 ────────────────────────────────
  if (on_focuser) {
    FocuserData foc_snap;
    ImagingData img_snap;
    if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(50)) == pdTRUE) {
      foc_snap = g_focuser;
      img_snap = g_imaging;
      xSemaphoreGive(g_data_mux);
    }
    screens_update_focuser(
      foc_snap.position, foc_snap.temperature, foc_snap.tc_enabled,
      foc_snap.tc_coeff, foc_snap.last_hfr, foc_snap.last_steps,
      foc_snap.last_temp, img_snap.cam_temp);
  }

  // ── Sky screen — only when on Screen5 ────────────────────────────────────
  if (on_sky) {
  SkyData sky_snap; GuiderStats guide_snap; ImagingData img_snap;
  if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(50)) == pdTRUE) {
    sky_snap = g_sky; guide_snap = g_stats; img_snap = g_imaging;
    xSemaphoreGive(g_data_mux);
  }
  screens_update_sky(sky_snap.target, img_snap.filter, img_snap.hfr,
                     sky_snap.subs_done, guide_snap.rmsTotal, sky_snap.flip_min);
}
}
// ── NINA polling ──────────────────────────────────────────────────────────────
static bool http_get(const String &url, String &out) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] skipped — WiFi not connected");
    return false;
  }
  HTTPClient http;
  http.begin(url);
  http.setTimeout(2500);
  unsigned long t0 = millis();
  int code = http.GET();
  unsigned long dt = millis() - t0;
  if (code != 200) {
    Serial.printf("[HTTP] GET %s -> %d (%lums)\n", url.c_str(), code, dt);
    http.end();
    return false;
  }
  out = http.getString();
  http.end();
  Serial.printf("[HTTP] GET %s -> 200 (%lums, %d bytes)\n", url.c_str(), dt, out.length());
  return true;
}

static void poll_nina() {
  String base = "http://" + String(g_nina_host) + ":" + String(NINA_PORT) + "/v2/api";
  String body;

  // ninaAPI v2 returns everything we need from /equipment/guider/info :
  //   Response.RMSError.RA.Arcseconds / .Pixel   (nested!)
  //   Response.RMSError.Dec.Arcseconds / .Pixel
  //   Response.RMSError.Total.Arcseconds / .Pixel
  //   Response.LastGuideStep.RADistanceRaw / .DECDistanceRaw   (the live point, in pixels)
  //   Response.PixelScale  (arcsec per pixel — to convert the guide step to arcsec)
  // There is NO /chart endpoint, so we append LastGuideStep each poll.

  if (http_get(base + "/equipment/guider/info", body)) {
    // Use DynamicJsonBuffer (v5) or DynamicJsonDocument (v6/v7)
    // We use v5 API which is compatible with all versions
    DynamicJsonBuffer jbuf(6144);
    JsonObject &root = jbuf.parseObject(body);
    if (!root.success()) {
    } else {
      JsonObject &r = root["Response"];

      GuiderStats s;
      s.connected = r["Connected"].as<bool>();
      const char *st = r["State"];
      strncpy(s.state, st ? st : "---", sizeof(s.state)-1);
      s.guiding = (strcmp(s.state,"Guiding")==0);

      // RMS nested structure: RMSError.RA.Arcseconds etc.
      s.rmsRA    = r["RMSError"]["RA"]["Arcseconds"]    | 0.0f;
      s.rmsDec   = r["RMSError"]["Dec"]["Arcseconds"]   | 0.0f;
      s.rmsTotal = r["RMSError"]["Total"]["Arcseconds"] | 0.0f;
      s.rawRA    = r["RMSError"]["RA"]["Pixel"]         | 0.0f;
      s.rawDec   = r["RMSError"]["Dec"]["Pixel"]        | 0.0f;
      s.rawTotal = r["RMSError"]["Total"]["Pixel"]      | 0.0f;


      if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(100))) {
        g_stats = s; xSemaphoreGive(g_data_mux);
      }

      // Guide step: pixels * pixelScale = arcsec
      float pixscale = r["PixelScale"] | 1.0f;
      JsonObject &gs = r["LastGuideStep"];
      if (gs.success()) {
        float ra_px  = gs["RADistanceRaw"]  | 0.0f;
        float dec_px = gs["DECDistanceRaw"] | 0.0f;

        // Only append if this is a new guide step (value changed from last poll)
        static float last_ra_px = -9999.0f, last_dec_px = -9999.0f;
        bool new_step = (fabsf(ra_px - last_ra_px) > 0.0001f ||
                         fabsf(dec_px - last_dec_px) > 0.0001f);
        if (new_step) {
          last_ra_px  = ra_px;
          last_dec_px = dec_px;
          GuidePoint pt;
          pt.ra       = ra_px  * pixscale;
          pt.dec      = dec_px * pixscale;
          pt.ra_dist  = pt.ra;
          pt.dec_dist = pt.dec;
          pt.valid    = true;
          pt.dither   = false;
            if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(100))) {
            g_hist[g_head] = pt;
            g_head = (g_head+1) % MAX_PTS;
            if (g_count < MAX_PTS) g_count++;
            xSemaphoreGive(g_data_mux);
          }
        }
      }
    }
  } else {
    if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(100))) {
      g_stats.connected = false;
      strncpy(g_stats.state, "Offline", sizeof(g_stats.state));
      xSemaphoreGive(g_data_mux);
    }
  }

  g_need_update = true;
}

// ── Demo mode ─────────────────────────────────────────────────────────────────
static void inject_demo() {
  static float t=0, pe=0; static int dctr=0;
  t+=0.04f; pe+=0.018f; dctr++;

  float ra  =  0.55f*sinf(pe) + 0.20f*sinf(t*1.1f) + ((int)(esp_random()%31)-15)*0.018f;
  float dec =  0.30f*sinf(t*0.7f+0.8f) + 0.12f*cosf(t*1.8f) + ((int)(esp_random()%25)-12)*0.014f;
  float ra_d  = -ra  * 0.65f + ((int)(esp_random()%11)-5)*0.008f;
  float dec_d = -dec * 0.65f + ((int)(esp_random()%11)-5)*0.006f;
  bool  dith  = (dctr%55==0);

  if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(50))) {
    g_hist[g_head]={ra,dec,ra_d,dec_d,true,dith};
    g_head=(g_head+1)%MAX_PTS;
    if (g_count<MAX_PTS) g_count++;
    g_stats.connected=false; g_stats.guiding=true;
    strncpy(g_stats.state,"Demo",sizeof(g_stats.state));
    int n=(g_count<50)?g_count:50;
    float sRA=0,sDec=0;
    for (int i=0;i<n;i++){
      int idx=(g_head-1-i+MAX_PTS)%MAX_PTS;
      sRA+=g_hist[idx].ra*g_hist[idx].ra; sDec+=g_hist[idx].dec*g_hist[idx].dec;
    }
    g_stats.rmsRA=sqrtf(sRA/n); g_stats.rmsDec=sqrtf(sDec/n);
    g_stats.rmsTotal=sqrtf((sRA+sDec)/(2*n));
    g_stats.rawRA=fabsf(ra); g_stats.rawDec=fabsf(dec);
    g_stats.rawTotal=sqrtf(ra*ra+dec*dec);
    xSemaphoreGive(g_data_mux);
  }
  g_need_update=true;
}

static void poll_imaging() {
  String base = "http://" + String(g_nina_host) + ":" + String(NINA_PORT) + "/v2/api";
  String body;
  ImagingData d;

  // ── Camera info (temp, cooler, exposure progress) ─────────────────────────
  if (http_get(base + "/equipment/camera/info", body)) {
    DynamicJsonBuffer jbuf(4096);
    JsonObject &root = jbuf.parseObject(body);
    if (root.success()) {
      JsonObject &r = root["Response"];
      d.cam_temp    = r["Temperature"]    | 0.0f;
      d.cooler_pct  = r["CoolerPower"]    | 0.0f;
      d.exp_elapsed = r["ExposureElapsed"]| 0.0f;
      d.exp_total   = r["ExposureTime"]   | 0.0f;
    }
  }

  // ── Full image history — drives both the live values AND the graph ────────
  // We parse the entire history each cycle so the Screen3 graph mirrors NINA
  // exactly (no missed subs, no drift). The latest entry feeds the live labels.
  if (http_get(base + "/image-history?all=true", body)) {
    // Count subs cheaply first (buffer-proof) by counting "HFR": occurrences
    int sub_count = 0, scan = 0;
    while ((scan = body.indexOf("\"HFR\":", scan)) >= 0) { sub_count++; scan += 6; }
    d.subs_done = sub_count;

    DynamicJsonBuffer jbuf(16384);   // large: full history grows with the session
    JsonObject &root = jbuf.parseObject(body);
    if (root.success()) {
      JsonArray &arr = root["Response"];
      if (arr.success() && arr.size() > 0) {
        int n = arr.size();

        // Build local arrays for the graphs: HFR, stars, and per-sub total error
        static float hist_hfr[NINA_MAX_HIST];
        static int   hist_stars[NINA_MAX_HIST];
        static float hist_rms[NINA_MAX_HIST];
        int gi = 0;
        for (int i = 0; i < n && gi < NINA_MAX_HIST; i++) {
          JsonObject &it = arr[i];
          float h = it["HFR"]  | 0.0f;
          int   s = it["Stars"]| 0;
          if (isnan(h)) h = 0;
          hist_hfr[gi]   = h;
          hist_stars[gi] = s;

          // Parse total error (arcsec) from RmsText: "Tot: 0.88 (0.64\")"
          // We want the value in parentheses (arcsec), not the pixel value.
          const char *rms = it["RmsText"] | "";
          float arcsec = 0.0f;
          if (rms && rms[0]) {
            const char *paren = strchr(rms, '(');
            if (paren) arcsec = atof(paren + 1);   // value right after '('
          }
          hist_rms[gi] = arcsec;
          gi++;
        }
        screens_set_hfr_history(hist_hfr, hist_stars, gi);
        screens_set_sky_history(hist_hfr, hist_rms, gi);   // accumulated Screen5 charts

        // Latest entry → live labels
        JsonObject &item = arr[n - 1];
        float hfr = item["HFR"] | 0.0f;
        if (!isnan(hfr)) d.hfr = hfr;
        d.star_count = item["Stars"] | 0;
        float mean = item["Mean"] | 0.0f;
        if (!isnan(mean)) d.adu_mean = mean;

        const char *filt = item["Filter"] | "";
        if (filt && strlen(filt) > 0) strncpy(d.filter, filt, sizeof(d.filter)-1);
        else                          strncpy(d.filter, "--", sizeof(d.filter)-1);

        const char *tname = item["TargetName"] | "";
        if (tname && strlen(tname) > 0) strncpy(d.target, tname, sizeof(d.target)-1);
        else                            strncpy(d.target, "---", sizeof(d.target)-1);

        Serial.printf("[IMG] history: %d subs, latest HFR=%.2f stars=%d filter=%s\n",
          d.subs_done, d.hfr, d.star_count, d.filter);
      } else {
        Serial.println("[IMG] image-history: empty (no subs yet this session)");
      }
    } else {
      Serial.printf("[IMG] image-history parse FAILED (count=%d via scan)\n", d.subs_done);
    }
  }

  Serial.printf("[IMG] camera: temp=%.1f cooler=%.0f%% filter=%s\n",
    d.cam_temp, d.cooler_pct, d.filter);

  if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(100))) {
    g_imaging = d; xSemaphoreGive(g_data_mux);
  }
}

static void poll_focuser() {
  String base = "http://" + String(g_nina_host) + ":" + String(NINA_PORT) + "/v2/api";
  String body;
  FocuserData d;

  if (http_get(base + "/equipment/focuser/info", body)) {
    DynamicJsonBuffer jbuf(2048);
    JsonObject &root = jbuf.parseObject(body);
    if (root.success()) {
      JsonObject &r = root["Response"];
      d.position    = r["Position"]                  | 0;
      d.temperature = r["Temperature"]               | -99.0f;
      d.tc_enabled  = r["TempCompEnabled"]           | false;
      d.tc_coeff    = r["TempCompStepsPerDegree"]    | 0.0f;
      d.last_hfr    = r["LastAutoFocusResult"]["HFR"]| 0.0f;
      d.last_steps  = r["LastAutoFocusResult"]["Steps"]| 0;
      d.last_temp   = r["LastAutoFocusResult"]["Temperature"]| -99.0f;
    }
  }

  if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(100))) {
    g_focuser = d; xSemaphoreGive(g_data_mux);
  }
}

static void poll_sky() {
  String base = "http://" + String(g_nina_host) + ":" + String(NINA_PORT) + "/v2/api";
  String body;
  SkyData d;

  if (http_get(base + "/sequence/state", body)) {
    // Target name
    int k = body.indexOf("\"TargetName\":\"");
    if (k >= 0) {
      k += 14;
      int end = body.indexOf('"', k);
      if (end > k) {
        String name = body.substring(k, end);
        name.trim();
        strncpy(d.target, name.length() ? name.c_str() : "---", sizeof(d.target)-1);
      }
    } else {
      strncpy(d.target, "DON'T LOOK UP", sizeof(d.target)-1);
    }

    // Time to meridian flip — NINA reports HOURS; 24.0 is the "unknown" sentinel
    int m = body.indexOf("\"TimeToMeridianFlip\":");
    if (m >= 0) {
      m += 21;   // past "TimeToMeridianFlip":
      int end = body.indexOf(',', m);
      if (end < 0) end = body.indexOf('}', m);
      if (end > m) {
        float hours = body.substring(m, end).toFloat();
        if (hours >= 23.99f) d.flip_min = -1;             // unknown / not calculated
        else                 d.flip_min = hours * 60.0f;  // hours → minutes
      }
    }
    Serial.printf("[SKY] target=%s flip=%.1fmin\n", d.target, d.flip_min);
  }

  // Sub count from full image-history — string-count "HFR": (buffer-proof)
  if (http_get(base + "/image-history?all=true", body)) {
    int c = 0, scan = 0;
    while ((scan = body.indexOf("\"HFR\":", scan)) >= 0) { c++; scan += 6; }
    d.subs_done = c;
  }

  if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(100))) {
    g_sky = d; xSemaphoreGive(g_data_mux);
  }
}
static void nina_task(void *arg) {
  Serial.println("[NINA] task started");
  int cycle = 0;
  bool was_connected = false;
  bool host_resolved = false;
  
  while (1) {
    wifi_manager_loop();
    if (cycle % 10 == 0) {
  float v; int raw;
  adc_get_value(&v, &raw);
  if (v > 2.5f && v < 4.5f) {       // ignore ADC2/WiFi-clash garbage
    g_batt_v   = v;
    g_batt_pct = lipo_pct(v);
    Serial.printf("[BATT] %.2fV -> %d%%\n", v, g_batt_pct);
  }
}
    bool connected = (WiFi.status()==WL_CONNECTED);

    if (!connected && wm_ap_active) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

    if (connected && !host_resolved) {
      nina_resolve_host();
      host_resolved = (strlen(g_nina_host) > 0);
      if (!host_resolved) {
        Serial.println("[NINA] no host — retrying in 10s");
        vTaskDelay(pdMS_TO_TICKS(10000));
        continue;
      }
    }

    if (connected != was_connected) { was_connected = connected; }

    if (connected && host_resolved) {
      poll_nina();
      vTaskDelay(pdMS_TO_TICKS(50));
      if (cycle % 6 == 0)  { poll_imaging();  vTaskDelay(pdMS_TO_TICKS(50)); }
      if (cycle % 10 == 0) { poll_focuser(); vTaskDelay(pdMS_TO_TICKS(50)); }
      if (cycle % 10 == 0) { poll_sky();     vTaskDelay(pdMS_TO_TICKS(50)); }
    } else if (!connected) {
      inject_demo();
    }
    cycle++;
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// Set panel brightness 0-255 via SH8601 reg 0x51
static void set_brightness(uint8_t level) {
  if (!g_io_handle) return;
  esp_lcd_panel_io_tx_param(g_io_handle, 0x51, (uint8_t[]){level}, 1);
}

extern "C" void brigtnesschange(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int val = lv_slider_get_value(slider);   // 0-100
  // Map 0-100% → 10-255 (avoid fully black at 0)
  uint8_t level = (uint8_t)(10 + (val * 245) / 100);
  set_brightness(level);
}

// ── X-scale slider callback (called by SLS when Slider1 value changes) ──────
// Slider range 1-100, default 50.
// We map it so that:
//   100 = show all available history (up to GRAPH_W points)
//     1 = show only the last ~1% of points (most zoomed in, ~6 points)
// A quadratic curve gives finer control at the zoomed-in end.
extern "C" void xscalechange(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  int val = lv_slider_get_value(slider);  // 1-100

  // Quadratic mapping: pct = (val/100)^2 * 100
  // val=100 → pct=100 (full history)
  // val=50  → pct=25  (last quarter)
  // val=10  → pct=1   (last 1%)
  // Map slider 1-100 linearly to 20-600 points
  // 50 (default) → 120 pts ≈ 5min at 2.5s cadence
  // 100          → 600 pts ≈ 25min
  // 1            → 20 pts  ≈ 1min
  int pts = 20 + (int)((val - 1) * (580.0f / 99.0f));
  if (pts < 20)  pts = 20;
  if (pts > 600) pts = 600;

  if (xSemaphoreTake(g_data_mux, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_x_zoom_pts = pts;
    xSemaphoreGive(g_data_mux);
  }
  g_need_update = true;  // trigger immediate redraw
}

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);   // BAT_ON — hold power so board stays up after PWR release
  Serial.begin(115200);
  delay(500);   // give USB-CDC time to enumerate so first prints aren't lost
  Serial.println();
  Serial.println("========================================");
  Serial.println("  NINA Guiding Display - booting");
  Serial.printf ("  NINA target: %s:%d\n", g_nina_host, NINA_PORT);
  Serial.println("========================================");
  delay(500);
  adc_bsp_init();
  // in setup(), AFTER adc_bsp_init() but BEFORE wifi_manager_begin()
{
  float v; int raw;
  for (int i = 0; i < 5; i++) {        // average a few clean reads
    adc_get_value(&v, &raw);
    if (v > 2.5f && v < 4.5f) { g_batt_v = v; g_batt_pct = lipo_pct(v); }
    delay(20);
  }
  Serial.printf("[BATT] boot reading: %.2fV -> %d%%\n", g_batt_v, g_batt_pct);
}
  static lv_disp_draw_buf_t disp_buf;
  static lv_disp_drv_t      disp_drv;

  // SPI + LCD
  const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(
    PIN_LCD_PCLK, PIN_LCD_DATA0, PIN_LCD_DATA1, PIN_LCD_DATA2, PIN_LCD_DATA3,
    LCD_H_RES*LCD_V_RES*LCD_BIT_PER_PIXEL/8);
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_handle_t io_handle=NULL;
  const esp_lcd_panel_io_spi_config_t io_config =
    SH8601_PANEL_IO_QSPI_CONFIG(PIN_LCD_CS, cb_flush_ready, &disp_drv);
  sh8601_vendor_config_t vendor_config = {
    .init_cmds=g_lcd_init_cmds,
    .init_cmds_size=sizeof(g_lcd_init_cmds)/sizeof(g_lcd_init_cmds[0]),
    .flags={.use_qspi_interface=1}
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
  g_io_handle = io_handle;
  esp_lcd_panel_handle_t panel=NULL;
  const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num=PIN_LCD_RST,
    .rgb_ele_order=LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel=LCD_BIT_PER_PIXEL,
    .vendor_config=&vendor_config
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
  // Test: enable brightness control (0x53), then set low brightness (0x51)
ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

  // ── BRIGHTNESS DIAGNOSTIC — sweep dark→bright, watch the screen ──────────
  

  // Touch
  const i2c_config_t i2c_conf = {
    .mode=I2C_MODE_MASTER, .sda_io_num=PIN_TOUCH_SDA, .scl_io_num=PIN_TOUCH_SCL,
    .sda_pullup_en=GPIO_PULLUP_ENABLE, .scl_pullup_en=GPIO_PULLUP_ENABLE,
    .master={.clk_speed=300000}
  };
  ESP_ERROR_CHECK(i2c_param_config(TOUCH_HOST, &i2c_conf));
  ESP_ERROR_CHECK(i2c_driver_install(TOUCH_HOST, I2C_MODE_MASTER, 0, 0, 0));
  esp_lcd_panel_io_handle_t tp_io=NULL;
  const esp_lcd_panel_io_i2c_config_t tp_io_cfg=ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_cfg, &tp_io));
  const esp_lcd_touch_config_t tp_cfg = {
    .x_max=LCD_V_RES-1, .y_max=LCD_H_RES-1,
    .rst_gpio_num=PIN_TOUCH_RST, .int_gpio_num=PIN_TOUCH_INT,
    .levels={.reset=0,.interrupt=0},
    .flags={.swap_xy=1,.mirror_x=0,.mirror_y=1}
  };
  ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io, &tp_cfg, &g_tp));

  // LVGL
  lv_init();
  lv_color_t *buf1=(lv_color_t*)heap_caps_malloc(LCD_H_RES*LVGL_BUF_HEIGHT*sizeof(lv_color_t),MALLOC_CAP_DMA);
  lv_color_t *buf2=(lv_color_t*)heap_caps_malloc(LCD_H_RES*LVGL_BUF_HEIGHT*sizeof(lv_color_t),MALLOC_CAP_DMA);
  assert(buf1&&buf2);
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES*LVGL_BUF_HEIGHT);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res=LCD_H_RES; disp_drv.ver_res=LCD_V_RES;
  disp_drv.flush_cb=cb_flush; disp_drv.rounder_cb=cb_rounder;
  disp_drv.drv_update_cb=cb_disp_update;
  disp_drv.draw_buf=&disp_buf; disp_drv.user_data=panel;
  lv_disp_t *disp=lv_disp_drv_register(&disp_drv);

  const esp_timer_create_args_t tick_args={.callback=cb_tick,.name="lvgl_tick"};
  esp_timer_handle_t tick_timer;
  ESP_ERROR_CHECK(esp_timer_create(&tick_args,&tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer,LVGL_TICK_MS*1000));

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type=LV_INDEV_TYPE_POINTER; indev_drv.disp=disp;
  indev_drv.read_cb=cb_touch; indev_drv.user_data=g_tp;
  lv_indev_drv_register(&indev_drv);

  g_lvgl_mux=xSemaphoreCreateMutex();
  g_data_mux=xSemaphoreCreateMutex();
  assert(g_lvgl_mux&&g_data_mux);

  if (lvgl_lock(-1)) {
    // Init SLS UI — loads ui_uiGraphPanel screen
    ui_init();
    lv_disp_load_scr(ui_Screen6);
    set_brightness(10 + (50 * 245) / 100);   // 50% → ~132
    lv_obj_set_style_bg_color(ui_batterybar, lv_color_hex(0x21262D), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_batterybar, LV_OPA_COVER, LV_PART_MAIN);
    // Override theme colours on all objects
    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0x161B22), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_uiGraphPanel,   LV_OPA_COVER,           LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_uiGraphPanel, 0,                  LV_PART_MAIN);

    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0x161B22), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_Panel1,   LV_OPA_COVER,           LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_Panel1, 0,                  LV_PART_MAIN);

    // Zero Panel2 padding/border BEFORE canvas creation so GW/GH fill it exactly
    lv_obj_set_style_bg_color(ui_Panel2,     lv_color_hex(0x0D1117), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_Panel2,       LV_OPA_COVER,           LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_Panel2, 0,                       LV_PART_MAIN);
    lv_obj_set_style_pad_all(ui_Panel2,      0,                       LV_PART_MAIN);
    lv_obj_set_style_radius(ui_Panel2,       0,                       LV_PART_MAIN);

    // Attach canvas to Panel2 — fills it completely
    g_canvas_buf=(lv_color_t*)heap_caps_malloc(GW*GH*sizeof(lv_color_t), MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if (!g_canvas_buf) {
      Serial.println("[CANVAS] PSRAM alloc FAILED");
    } else {
      g_canvas=lv_canvas_create(ui_Panel2);
      lv_canvas_set_buffer(g_canvas, g_canvas_buf, GW, GH, LV_IMG_CF_TRUE_COLOR);
      lv_obj_set_pos(g_canvas, 0, 0);
      lv_obj_set_size(g_canvas, GW, GH);
      graph_draw();
      Serial.printf("[CANVAS] OK %dx%d\n", GW, GH);
    }

    screens_init();  // build imaging/focuser/sky panels
    lv_timer_create(ui_refresh_cb, 200, NULL);
    lvgl_unlock();
  }

  xTaskCreate(lvgl_task,"LVGL",LVGL_TASK_STACK,NULL,LVGL_TASK_PRIO,NULL);

  // WiFi — try saved credentials, fall back to captive portal AP
  bool wifi_ok = wifi_manager_begin();
  if (wifi_ok) {
    ESP_LOGI(TAG, "WiFi connected: %s", WiFi.localIP().toString().c_str());
  } else {
    ESP_LOGI(TAG, "AP mode — connect phone to 'NINA-Display' and open 192.168.4.1");
  }

  Serial.println("[SETUP] creating nina_task");
  BaseType_t ok = xTaskCreate(nina_task,"NINA",NINA_TASK_STACK,NULL,NINA_TASK_PRIO,NULL);
  Serial.printf("[SETUP] xTaskCreate returned %d (pdPASS=%d)\n", ok, pdPASS);
  Serial.println("[SETUP] nina_task create returned");
  ESP_LOGI(TAG,"Setup done");
}
extern "C" void resetwifi(lv_event_t *e) {
  if (!lv_obj_has_state(ui_sure1, LV_STATE_CHECKED) ||
      !lv_obj_has_state(ui_sure2, LV_STATE_CHECKED)) {
    Serial.println("[WIFI] reset blocked — check both boxes");
    return;
  }
  Serial.println("[WIFI] Reset — clearing credentials and rebooting");
  wm_clear_credentials();
  delay(500);
  ESP.restart();
}
static int lipo_pct(float v) {
  if (v >= 4.20f) return 100;
  if (v <= 3.30f) return 0;
  static const float vtab[] = {3.30,3.50,3.60,3.70,3.75,3.80,3.85,3.90,3.95,4.00,4.10,4.20};
  static const int   ptab[] = {   0,   5,  10,  20,  30,  45,  55,  65,  75,  85,  95, 100};
  for (int i = 0; i < 11; i++) {
    if (v < vtab[i+1]) {
      float frac = (v - vtab[i]) / (vtab[i+1] - vtab[i]);
      return ptab[i] + (int)(frac * (ptab[i+1] - ptab[i]));
    }
  }
  return 100;
}
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
