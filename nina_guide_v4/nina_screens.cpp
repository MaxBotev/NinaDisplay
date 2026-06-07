/*
 * nina_screens.cpp
 * Screen3 — HFR & Stars graph (NINA-style dual Y-axis)
 * Screen4 — Focuser
 * Screen5 — Sky / Mount
 *
 * SLS objects used:
 *   Screen3: ui_ImagingPanel (593x364), ui_Panel4 (header), ui_hfrlabel, ui_starslabel
 *   Screen4: ui_FocuserPanel
 *   Screen5: ui_SkyPanel
 */

#include <cstdio>
#include <cmath>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "esp_heap_caps.h"
#include "lvgl.h"
#include "ui.h"
#include <Arduino.h>

LV_FONT_DECLARE(ui_font_Font12);
LV_FONT_DECLARE(ui_font_Font13);
LV_FONT_DECLARE(lv_font_montserrat_14);
// ── Colours ───────────────────────────────────────────────────────────────────
#define C(h)       lv_color_hex(h)
#define COL_BG     C(0x0D1117)
#define COL_PANEL  C(0x161B22)
#define COL_BORDER C(0x30363D)
#define COL_WHITE  C(0xE6EDF3)
#define COL_GREY   C(0x8B949E)
#define COL_GOOD   C(0x3FB950)
#define COL_WARN   C(0xD29922)
#define COL_BAD    C(0xF85149)
#define COL_DIM    C(0x484F58)
#define COL_HFR    C(0x3FB950)   // green  — HFR line (matches NINA)
#define COL_STARS  C(0xE5A117)   // amber  — Stars line (matches NINA)

// ── Graph data history ────────────────────────────────────────────────────────
// No cap — all frames stored, graph compresses to fit width (like NINA)
#define IMG_MAX_PTS  256
static float s_hfr[IMG_MAX_PTS]   = {0};
static int   s_stars[IMG_MAX_PTS] = {0};
static int   s_img_count = 0;   // total frames recorded

// ── Canvas ────────────────────────────────────────────────────────────────────
// ImagingPanel: 593x364, with padding zeroed = full size
#define IGW  593
#define IGH  320   // leave 44px at bottom for x-axis labels

// Left margin for HFR Y-axis, right margin for Stars Y-axis
#define YAXIS_L  52
#define YAXIS_R  48
#define GRAPH_X  YAXIS_L
#define GRAPH_W  (IGW - YAXIS_L - YAXIS_R)
#define GRAPH_H  (IGH - 20)   // top 20px for label clearance

static lv_obj_t   *s_canvas    = NULL;
static lv_color_t *s_canvas_buf = NULL;

static bool  s_use_fahrenheit = false;
static float s_ambient_c     = -99.0f;
static float s_camera_c      = -99.0f;

// ── Helpers ───────────────────────────────────────────────────────────────────
// Apply battery state to any bar (and optional label, may be NULL)

static void apply_battery_bar(lv_obj_t *bar, lv_obj_t *label, int pct, float volts) {
  if (!bar) return;
  if (pct < 0) {
    if (label) lv_label_set_text(label, "no data");
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
    return;
  }
  if (pct > 100) pct = 100;
  if (pct < 0)   pct = 0;

  lv_bar_set_mode(bar, LV_BAR_MODE_NORMAL);
  lv_bar_set_range(bar, 0, 100);
  lv_bar_set_value(bar, pct, LV_ANIM_OFF);

  if (label) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%d%%  (%.2fV)", pct, volts);
    lv_label_set_text(label, buf);
  }

  lv_color_t col;
  if (pct <= 10)      col = lv_color_hex(0xF85149);
  else if (pct <= 30) col = lv_color_hex(0xD29922);
  else if (pct <= 60) col = lv_color_hex(0xD0C541);
  else                col = lv_color_hex(0x3FB950);

  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(bar, col, LV_PART_INDICATOR);
}
void screens_update_battery_bars(int pct, float volts) {
  lv_obj_t *active = lv_scr_act();
  if (active == ui_uiGraphPanel)  apply_battery_bar(ui_batbar, NULL, pct, volts);
  else if (active == ui_Screen2) {
    apply_battery_bar(ui_batterybar, ui_batterylabel, pct, volts);
    apply_battery_bar(ui_batbar1, NULL, pct, volts);
  }
  else if (active == ui_Screen3)  apply_battery_bar(ui_batbar2, NULL, pct, volts);
  else if (active == ui_Screen5)  apply_battery_bar(ui_batbar3, NULL, pct, volts);
}
void screens_update_battery(int pct, float volts) {
  char buf[32];
  if (pct < 0) {
    lv_label_set_text(ui_batterylabel, "no data");
    return;
  }
  if (pct > 100) pct = 100;
  if (pct < 0)   pct = 0;

  lv_bar_set_mode(ui_batterybar, LV_BAR_MODE_NORMAL);
  lv_bar_set_range(ui_batterybar, 0, 100);
  lv_bar_set_value(ui_batterybar, pct, LV_ANIM_OFF);
  Serial.printf("[BATT-UI] set %d, reads %d\n", pct, lv_bar_get_value(ui_batterybar));

  snprintf(buf, sizeof(buf), "%d%%  (%.2fV)", pct, volts);
  lv_label_set_text(ui_batterylabel, buf);

  lv_color_t col;
  if (pct <= 10)      col = lv_color_hex(0xF85149);
  else if (pct <= 30) col = lv_color_hex(0xD29922);
  else if (pct <= 60) col = lv_color_hex(0xD0C541);
  else                col = lv_color_hex(0x3FB950);

  lv_obj_set_style_bg_opa(ui_batterybar, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(ui_batterybar, col, LV_PART_INDICATOR);
}
static lv_obj_t *make_val(lv_obj_t *p, const char *t, int x, int y, lv_color_t col) {
  lv_obj_t *l = lv_label_create(p);
  lv_label_set_text(l, t);
  lv_obj_set_style_text_font(l, &ui_font_Font12, 0);
  lv_obj_set_style_text_color(l, col, 0);
  lv_obj_set_pos(l, x, y);
  return l;
}

// ── Y-axis mapping ────────────────────────────────────────────────────────────
static int hfr_to_y(float v, float mn, float mx) {
  if (mx <= mn) mx = mn + 0.1f;
  float norm = (v - mn) / (mx - mn);
  // invert: high value = top of graph
  return (int)((1.0f - norm) * (GRAPH_H - 4)) + 2;
}

static int stars_to_y(int v, int mn, int mx) {
  if (mx <= mn) mx = mn + 1;
  float norm = (float)(v - mn) / (float)(mx - mn);
  return (int)((1.0f - norm) * (GRAPH_H - 4)) + 2;
}

// ── Main graph draw ───────────────────────────────────────────────────────────
static void img_graph_draw() {
  if (!s_canvas || !s_canvas_buf || s_img_count == 0) {
    if (s_canvas) lv_canvas_fill_bg(s_canvas, COL_BG, LV_OPA_COVER);
    return;
  }

  lv_canvas_fill_bg(s_canvas, COL_BG, LV_OPA_COVER);

  int n = s_img_count;   // total points

  // Find HFR min/max for left Y-axis
  float hfr_min = 9999, hfr_max = 0;
  for (int i = 0; i < n; i++) {
    if (s_hfr[i] > 0) {
      if (s_hfr[i] < hfr_min) hfr_min = s_hfr[i];
      if (s_hfr[i] > hfr_max) hfr_max = s_hfr[i];
    }
  }
  if (hfr_min > hfr_max) { hfr_min = 1.0f; hfr_max = 3.0f; }
  // Add 10% headroom each side
  float hfr_pad = (hfr_max - hfr_min) * 0.15f;
  if (hfr_pad < 0.05f) hfr_pad = 0.05f;
  hfr_min -= hfr_pad; hfr_max += hfr_pad;

  // Find Stars min/max for right Y-axis
  int st_min = 99999, st_max = 0;
  for (int i = 0; i < n; i++) {
    if (s_stars[i] > 0) {
      if (s_stars[i] < st_min) st_min = s_stars[i];
      if (s_stars[i] > st_max) st_max = s_stars[i];
    }
  }
  if (st_min > st_max) { st_min = 0; st_max = 50; }
  int st_pad = (int)((st_max - st_min) * 0.15f) + 1;
  st_min -= st_pad; st_max += st_pad;
  if (st_min < 0) st_min = 0;

  // ── Grid lines (4 horizontal) ─────────────────────────────────────────────
  lv_draw_line_dsc_t gd;
  lv_draw_line_dsc_init(&gd);
  gd.color = C(0x21262D); gd.width = 1; gd.opa = LV_OPA_COVER;
  // dashed look: draw short segments with gaps
  for (int row = 0; row <= 4; row++) {
    int y = 2 + row * (GRAPH_H - 4) / 4;
    // draw dashed line
    for (int x = GRAPH_X; x < GRAPH_X + GRAPH_W; x += 12) {
      int x2 = x + 7; if (x2 > GRAPH_X + GRAPH_W) x2 = GRAPH_X + GRAPH_W;
      lv_point_t pts[2] = {{(lv_coord_t)x, (lv_coord_t)y}, {(lv_coord_t)x2, (lv_coord_t)y}};
      lv_canvas_draw_line(s_canvas, pts, 2, &gd);
    }
  }

  // ── Left Y-axis labels (HFR) ──────────────────────────────────────────────
  lv_draw_label_dsc_t ld;
  lv_draw_label_dsc_init(&ld);
  ld.font  = &lv_font_montserrat_14;
  ld.color = COL_HFR;
  ld.align = LV_TEXT_ALIGN_RIGHT;
  for (int row = 0; row <= 4; row++) {
    int y = 2 + row * (GRAPH_H - 4) / 4;
    float val = hfr_max - row * (hfr_max - hfr_min) / 4.0f;
    char buf[12];
    snprintf(buf, sizeof(buf), "%.2f", val);
    lv_canvas_draw_text(s_canvas, 0, y - 8, YAXIS_L - 4, &ld, buf);
  }

  // ── Right Y-axis labels (Stars) ───────────────────────────────────────────
  ld.color = COL_STARS;
  ld.align = LV_TEXT_ALIGN_LEFT;
  for (int row = 0; row <= 4; row++) {
    int y = 2 + row * (GRAPH_H - 4) / 4;
    int val = (int)(st_max - row * (st_max - st_min) / 4.0f);
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", val);
    lv_canvas_draw_text(s_canvas, GRAPH_X + GRAPH_W + 4, y - 8, YAXIS_R - 2, &ld, buf);
  }

  // ── X-axis frame number labels ────────────────────────────────────────────
  ld.color = COL_GREY;
  ld.align = LV_TEXT_ALIGN_CENTER;
  int labelStep = (n <= 10) ? 1 : (n <= 20) ? 2 : (n / 8);
  if (labelStep < 1) labelStep = 1;
  for (int i = 0; i < n; i += labelStep) {
    int x = GRAPH_X + (int)((float)i / (n > 1 ? n - 1 : 1) * (GRAPH_W - 1));
    char buf[8]; snprintf(buf, sizeof(buf), "%d", i + 1);
    lv_canvas_draw_text(s_canvas, x - 10, GRAPH_H + 4, 20, &ld, buf);
    // tick mark
    lv_draw_line_dsc_t td; lv_draw_line_dsc_init(&td);
    td.color = COL_GREY; td.width = 1; td.opa = LV_OPA_COVER;
    lv_point_t tp[2] = {{(lv_coord_t)x,(lv_coord_t)(GRAPH_H)},{(lv_coord_t)x,(lv_coord_t)(GRAPH_H+3)}};
    lv_canvas_draw_line(s_canvas, tp, 2, &td);
  }

  // ── Stars line (dashed amber) ─────────────────────────────────────────────
  lv_draw_line_dsc_t sl;
  lv_draw_line_dsc_init(&sl);
  sl.color = COL_STARS; sl.width = 2; sl.opa = LV_OPA_COVER;

  int prevX = -1, prevY = -1;
  for (int i = 0; i < n; i++) {
    if (s_stars[i] <= 0) { prevX = prevY = -1; continue; }
    int x = GRAPH_X + (int)((float)i / (n > 1 ? n - 1 : 1) * (GRAPH_W - 1));
    int y = stars_to_y(s_stars[i], st_min, st_max);
    if (prevX >= 0) {
      // dashed: alternate draw/skip segments of 6px
      int dx = x - prevX, dy = y - prevY;
      float len = sqrtf(dx*dx + dy*dy);
      int segs = (int)(len / 6); if (segs < 1) segs = 1;
      for (int s = 0; s < segs; s += 2) {
        float t0 = (float)s / segs, t1 = (float)(s+1) / segs;
        lv_point_t pp[2] = {
          {(lv_coord_t)(prevX + t0*dx), (lv_coord_t)(prevY + t0*dy)},
          {(lv_coord_t)(prevX + t1*dx), (lv_coord_t)(prevY + t1*dy)}
        };
        lv_canvas_draw_line(s_canvas, pp, 2, &sl);
      }
    }
    prevX = x; prevY = y;
  }

  // ── HFR line (solid green) with dot markers ───────────────────────────────
  lv_draw_line_dsc_t hl;
  lv_draw_line_dsc_init(&hl);
  hl.color = COL_HFR; hl.width = 2; hl.opa = LV_OPA_COVER;

  lv_draw_rect_dsc_t dot;
  lv_draw_rect_dsc_init(&dot);
  dot.bg_color = COL_HFR; dot.bg_opa = LV_OPA_COVER;
  dot.border_width = 0; dot.radius = LV_RADIUS_CIRCLE;

  prevX = -1; prevY = -1;
  for (int i = 0; i < n; i++) {
    if (s_hfr[i] <= 0) { prevX = prevY = -1; continue; }
    int x = GRAPH_X + (int)((float)i / (n > 1 ? n - 1 : 1) * (GRAPH_W - 1));
    int y = hfr_to_y(s_hfr[i], hfr_min, hfr_max);
    if (prevX >= 0) {
      lv_point_t pp[2] = {{(lv_coord_t)prevX,(lv_coord_t)prevY},{(lv_coord_t)x,(lv_coord_t)y}};
      lv_canvas_draw_line(s_canvas, pp, 2, &hl);
    }
    // dot marker — 5px circle
    lv_canvas_draw_rect(s_canvas, x-3, y-3, 6, 6, &dot);
    prevX = x; prevY = y;
  }
}

// ── Focuser / Sky stubs (kept minimal — add content later) ────────────────────


static lv_obj_t *sky_target_val = NULL;
static lv_obj_t *sky_ra_val     = NULL;
static lv_obj_t *sky_dec_val    = NULL;
static lv_obj_t *sky_alt_val    = NULL;
static lv_obj_t *sky_az_val     = NULL;
static lv_obj_t *sky_pier_val   = NULL;
static lv_obj_t *sky_flip_val   = NULL;
static lv_obj_t *sky_seq_bar    = NULL;
static lv_obj_t *sky_seq_lbl    = NULL;
static lv_obj_t *sky_time_val   = NULL;

static void make_divider(lv_obj_t *p, int y) {
  lv_obj_t *l = lv_obj_create(p);
  lv_obj_set_size(l, 560, 1);
  lv_obj_set_pos(l, 10, y);
  lv_obj_set_style_bg_color(l, C(0x30363D), 0);
  lv_obj_set_style_bg_opa(l, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(l, 0, 0);
  lv_obj_set_style_radius(l, 0, 0);
}

// ── Build Imaging screen ──────────────────────────────────────────────────────
static void build_imaging_screen() {
  // Style Panel4 (header bar)
  lv_obj_set_style_bg_color(ui_Panel4,    COL_PANEL, 0);
  lv_obj_set_style_bg_opa(ui_Panel4,      LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(ui_Panel4,0, 0);
  lv_obj_set_style_pad_all(ui_Panel4,     0, 0);

  // Style ImagingPanel
  lv_obj_set_style_bg_color(ui_ImagingPanel,     COL_BG, 0);
  lv_obj_set_style_bg_opa(ui_ImagingPanel,       LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(ui_ImagingPanel, 0, 0);
  lv_obj_set_style_pad_all(ui_ImagingPanel,      0, 0);
  lv_obj_set_style_radius(ui_ImagingPanel,       0, 0);

  // Style SLS header labels
  lv_obj_set_style_text_color(ui_hfrlabel,   COL_HFR,   0);
  lv_obj_set_style_text_color(ui_starslabel, COL_STARS, 0);

  // Allocate canvas buffer in PSRAM
  s_canvas_buf = (lv_color_t*)heap_caps_malloc(
    IGW * IGH * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_canvas_buf) { Serial.println("[IMG] canvas alloc FAILED"); return; }

  s_canvas = lv_canvas_create(ui_ImagingPanel);
  lv_canvas_set_buffer(s_canvas, s_canvas_buf, IGW, IGH, LV_IMG_CF_TRUE_COLOR);
  lv_obj_set_pos(s_canvas, 0, 0);
  lv_obj_set_size(s_canvas, IGW, IGH);
  lv_canvas_fill_bg(s_canvas, COL_BG, LV_OPA_COVER);

  Serial.printf("[IMG] canvas OK %dx%d\n", IGW, IGH);
}


// ── Build Sky screen ──────────────────────────────────────────────────────────
// ── Screen5 chart series ──────────────────────────────────────────────────────
static lv_chart_series_t *s_guide_series = NULL;
static lv_chart_series_t *s_hfr_series   = NULL;
// ── Build Sky/Summary screen ──────────────────────────────────────────────────


static void build_sky_screen() {
  lv_obj_set_style_bg_color(ui_SkyPanel, COL_BG, 0);
  lv_obj_set_style_bg_opa(ui_SkyPanel, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(ui_SkyPanel, 0, 0);
  lv_obj_clear_flag(ui_SkyPanel, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_style_text_color(ui_targettonight, COL_WHITE, 0);
  lv_obj_set_style_text_color(ui_filtername,    COL_GREY, 0);
  lv_obj_set_style_text_color(ui_totaldone,     COL_WHITE, 0);
  lv_obj_set_style_text_color(ui_hfrstat,       COL_GOOD, 0);
  lv_obj_set_style_text_color(ui_totstat, lv_color_hex(0x4472C4), 0);

  // Guiding chart — Total error only
  lv_obj_set_style_bg_color(ui_guidingchart, COL_BG, LV_PART_MAIN);
  lv_obj_set_style_border_color(ui_guidingchart, C(0x30363D), LV_PART_MAIN);
  lv_obj_set_style_border_width(ui_guidingchart, 1, LV_PART_MAIN);
  lv_chart_set_point_count(ui_guidingchart, 60);
  lv_chart_set_range(ui_guidingchart, LV_CHART_AXIS_PRIMARY_Y, 0, 200);  // 0.01"
  lv_chart_remove_series(ui_guidingchart, lv_chart_get_series_next(ui_guidingchart, NULL));
  s_guide_series = lv_chart_add_series(ui_guidingchart, lv_color_hex(0x4472C4), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_all_value(ui_guidingchart, s_guide_series, 0);

  // HFR chart
  lv_obj_set_style_bg_color(ui_hfronly, COL_BG, LV_PART_MAIN);
  lv_obj_set_style_border_color(ui_hfronly, C(0x30363D), LV_PART_MAIN);
  lv_obj_set_style_border_width(ui_hfronly, 1, LV_PART_MAIN);
  lv_chart_set_point_count(ui_hfronly, 50);
  lv_chart_set_range(ui_hfronly, LV_CHART_AXIS_PRIMARY_Y, 0, 500);  // 0.01"
  lv_chart_remove_series(ui_hfronly, lv_chart_get_series_next(ui_hfronly, NULL));
  s_hfr_series = lv_chart_add_series(ui_hfronly, lv_color_hex(0x3FB950), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_all_value(ui_hfronly, s_hfr_series, 0);
}

// ── Sky/Summary update ────────────────────────────────────────────────────────
void screens_update_sky(
    const char *target, const char *filter,
    float hfr, int subs_done, float rms_total, float flip_min)
{
  char buf[64];

  lv_label_set_text(ui_targettonight, (target && target[0]) ? target : "---");
  lv_label_set_text(ui_filtername, (filter && strlen(filter)>0) ? filter : "No filter");

  // Total images done
  snprintf(buf, sizeof(buf), "%d", subs_done);
  lv_label_set_text(ui_totaldone, buf);

  // Meridian flip countdown
  if (flip_min > 0) {
    int h = (int)flip_min / 60;
    int m = (int)flip_min % 60;
    if (h > 0) snprintf(buf, sizeof(buf), "%dh %dm", h, m);
    else       snprintf(buf, sizeof(buf), "%dm", m);
  } else {
    snprintf(buf, sizeof(buf), "--");
  }
  lv_label_set_text(ui_meridian, buf);

  // HFR stat (charts are populated by screens_set_sky_history with the full session)
  if (hfr > 0) {
    snprintf(buf, sizeof(buf), "HFR: %.2f", hfr);
    lv_obj_set_style_text_color(ui_hfrstat,
      hfr < 2.0f ? COL_GOOD : hfr < 3.0f ? COL_WARN : COL_BAD, 0);
  } else snprintf(buf, sizeof(buf), "HFR: --");
  lv_label_set_text(ui_hfrstat, buf);

  // Total error stat
  if (rms_total > 0) {
    snprintf(buf, sizeof(buf), "Tot: %.2f\"", rms_total);
  } else snprintf(buf, sizeof(buf), "Tot: --");
  lv_label_set_text(ui_totstat, buf);
}

// Populate both Screen5 charts with the ENTIRE session so they show the
// accumulated trend (not a scrolling live window). Called from poll_imaging.
void screens_set_sky_history(const float *hfr, const float *rms_arcsec, int n) {
  if (n <= 0) return;

  // Cap to a sane max so a long night doesn't overflow the chart point arrays
  const int MAX_PTS = 250;
  int skip = 0;
  if (n > MAX_PTS) { skip = n - MAX_PTS; n = MAX_PTS; }   // keep most recent
  hfr       += skip;
  rms_arcsec += skip;

  // Auto-scale Y ranges to the data, with a little headroom
  float hfr_max = 0.1f, rms_max = 0.1f;
  for (int i = 0; i < n; i++) {
    if (hfr[i]        > hfr_max) hfr_max = hfr[i];
    if (rms_arcsec[i] > rms_max) rms_max = rms_arcsec[i];
  }

  // HFR chart — point count = number of subs, values in 0.01 units
  if (s_hfr_series) {
    lv_chart_set_point_count(ui_hfronly, n);
    lv_chart_set_range(ui_hfronly, LV_CHART_AXIS_PRIMARY_Y,
                       0, (lv_coord_t)((hfr_max * 1.15f) * 100.0f));
    for (int i = 0; i < n; i++)
      lv_chart_set_value_by_id(ui_hfronly, s_hfr_series, i,
                               (lv_coord_t)(hfr[i] * 100.0f));
    lv_chart_refresh(ui_hfronly);
  }

  // Guiding (total error) chart — arcsec in 0.01 units
  if (s_guide_series) {
    lv_chart_set_point_count(ui_guidingchart, n);
    lv_chart_set_range(ui_guidingchart, LV_CHART_AXIS_PRIMARY_Y,
                       0, (lv_coord_t)((rms_max * 1.15f) * 100.0f));
    for (int i = 0; i < n; i++)
      lv_chart_set_value_by_id(ui_guidingchart, s_guide_series, i,
                               (lv_coord_t)(rms_arcsec[i] * 100.0f));
    lv_chart_refresh(ui_guidingchart);
  }
}
// ═════════════════════════════════════════════════════════════════════════════
// PUBLIC API
// ═════════════════════════════════════════════════════════════════════════════



// ── Imaging update ────────────────────────────────────────────────────────────
// Replace the entire HFR/stars graph history in one shot. Called by
// poll_imaging() with the full set of subs from NINA, so the on-screen graph
// is always an exact mirror of NINA's image history — no drift, no missed subs.
void screens_set_hfr_history(const float *hfr, const int *stars, int n) {
  if (n > IMG_MAX_PTS) {
    // keep the most recent IMG_MAX_PTS points
    int skip = n - IMG_MAX_PTS;
    hfr   += skip;
    stars += skip;
    n = IMG_MAX_PTS;
  }
  for (int i = 0; i < n; i++) {
    s_hfr[i]   = hfr[i];
    s_stars[i] = stars[i];
  }
  s_img_count = n;
}

void screens_update_imaging(
    float hfr, int star_count, int subs_total, int subs_rejected,
    float exp_elapsed, float exp_total, const char *filter,
    float cam_temp, float cooler_pct, float adu_mean)
{
  // NOTE: graph history is now populated directly by poll_imaging() via
  // screens_set_hfr_history(), so the graph mirrors NINA exactly and never
  // drifts or misses subs. We no longer append here based on HFR changes.

  char buf[48];

  // HFR label — fixed width so it never wraps
  if (hfr > 0)
    snprintf(buf, sizeof(buf), "HFR: %.2f", hfr);
  else
    snprintf(buf, sizeof(buf), "HFR: --");
  lv_label_set_text(ui_hfrlabel, buf);
  lv_obj_set_style_text_color(ui_hfrlabel,
    hfr <= 0 ? COL_GREY : hfr < 2.0f ? COL_GOOD : hfr < 3.0f ? COL_WARN : COL_BAD, 0);

  // Stars label — fixed width, right-anchored
  snprintf(buf, sizeof(buf), "Stars: %d", star_count);
  lv_label_set_text(ui_starslabel, buf);
  //lv_obj_set_width(ui_starslabel, 120);                        // ← pin width
  //lv_label_set_long_mode(ui_starslabel, LV_LABEL_LONG_CLIP);  // ← no wrap

  img_graph_draw();
}

// ── Focuser update ────────────────────────────────────────────────────────────
// ── Temperature display state ─────────────────────────────────────────────────


// Convert and format temperature
static void fmt_temp(char *buf, int bufsz, float c, bool use_f) {
  if (c < -90.0f) {
    snprintf(buf, bufsz, "n/a");
    return;
  }
  if (use_f) {
    snprintf(buf, bufsz, "%.1f F", c * 9.0f / 5.0f + 32.0f);
  } else {
    snprintf(buf, bufsz, "%.1f C", c);
  }
}

// Arc color: green in safe zone, interpolates to red outside
// safe_lo / safe_hi define the green band
static lv_color_t temp_arc_color(float c, float safe_lo, float safe_hi) {
  if (c < -90.0f) return C(0x484F58);  // unknown — grey

  float dist = 0.0f;
  if (c < safe_lo)       dist = (safe_lo - c) / 15.0f;   // below safe
  else if (c > safe_hi)  dist = (c - safe_hi) / 15.0f;   // above safe
  if (dist > 1.0f) dist = 1.0f;

  // Lerp green → yellow → red
  if (dist < 0.5f) {
    // green → yellow
    float t = dist * 2.0f;
    uint8_t r = (uint8_t)(0x3F + t * (0xD2 - 0x3F));
    uint8_t g = (uint8_t)(0xB9 + t * (0x99 - 0xB9));
    uint8_t b = (uint8_t)(0x50 + t * (0x22 - 0x50));
    return lv_color_make(r, g, b);
  } else {
    // yellow → red
    float t = (dist - 0.5f) * 2.0f;
    uint8_t r = (uint8_t)(0xD2 + t * (0xF8 - 0xD2));
    uint8_t g = (uint8_t)(0x99 + t * (0x51 - 0x99));
    uint8_t b = (uint8_t)(0x22 + t * (0x49 - 0x22));
    return lv_color_make(r, g, b);
  }
}

static void refresh_temp_display() {
  char buf[32];

  if (s_ambient_c > -90.0f) {
    lv_arc_set_value(ui_ambientarc, (int)s_ambient_c);
    lv_color_t col = temp_arc_color(s_ambient_c, -10.0f, 20.0f);
    lv_obj_set_style_arc_color(ui_ambientarc, col, LV_PART_INDICATOR);
  }
  fmt_temp(buf, sizeof(buf), s_ambient_c, s_use_fahrenheit);
  lv_label_set_text(ui_ambienttemp, buf);   // ← SLS object directly

  if (s_camera_c > -90.0f) {
    lv_arc_set_value(ui_cameraarc, (int)s_camera_c);
    lv_color_t col = temp_arc_color(s_camera_c, -12.0f, -9.0f);
    lv_obj_set_style_arc_color(ui_cameraarc, col, LV_PART_INDICATOR);
  }
  fmt_temp(buf, sizeof(buf), s_camera_c, s_use_fahrenheit);
  lv_label_set_text(ui_cameratemp, buf);    // ← SLS object directly
}





// ── Sky update ────────────────────────────────────────────────────────────────

// ── Build Focuser screen ──────────────────────────────────────────────────────
// ── C/F toggle button ─────────────────────────────────────────────────────────
extern "C" void cfbuttonpressed(lv_event_t *e) {
  s_use_fahrenheit = !s_use_fahrenheit;
  refresh_temp_display();
}
// ── Build Focuser screen ──────────────────────────────────────────────────────
static void build_focuser_screen() {
  // Style both arcs — dark track, green indicator, no knob
  auto style_arc = [](lv_obj_t *arc) {
    lv_obj_set_style_arc_color(arc, C(0x21262D), LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, COL_GOOD,    LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(arc,   LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc,  0,             LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
  };
  style_arc(ui_ambientarc);
  style_arc(ui_cameraarc);

  // Style C/F button
  lv_obj_set_style_bg_color(ui_cfbutton, C(0x21262D), LV_PART_MAIN);
  lv_obj_set_style_border_color(ui_cfbutton, C(0x30363D), LV_PART_MAIN);
  lv_obj_set_style_border_width(ui_cfbutton, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(ui_cfbutton, 8, LV_PART_MAIN);
  lv_obj_set_style_text_color(ui_Label9, COL_WHITE, 0);

  // Header label colours
  lv_obj_set_style_text_color(ui_Label8, COL_GREY, 0);  // Ambient temp
  lv_obj_set_style_text_color(ui_Label1, COL_GREY, 0);  // Camera temp

  // Value label colours
  lv_obj_set_style_text_color(ui_ambienttemp, COL_WHITE, 0);
  lv_obj_set_style_text_color(ui_cameratemp,  COL_WHITE, 0);
}
void screens_update_focuser(
    int position, float temperature, bool tc_enabled, float tc_coeff,
    float last_af_hfr, int last_af_steps, float last_af_temp,
    float cam_temp)
{
  if (temperature > -50.0f) s_ambient_c = temperature;
  if (cam_temp > -90.0f)    s_camera_c  = cam_temp;
  refresh_temp_display();
}
void screens_init() {
  build_imaging_screen();
  build_focuser_screen();
  build_sky_screen();
}
