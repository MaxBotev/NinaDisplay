#pragma once

// Max V-curve points mirrored from NINA AUTOFOCUS-POINT-ADDED events
#define AF_MAX_PTS 50

#ifdef __cplusplus
extern "C++" {
#endif
void screens_set_sky_history(const float *hfr, const float *rms_arcsec, int n);
// Screen1 — autofocus V-curve + hfrlabel1 status text
void screens_update_autofocus(const int *pos, const float *hfr, int n,
    bool running, int best_pos, float best_hfr);
void screens_set_hfr_history(const float *hfr, const int *stars, int n);
void screens_init();
void screens_update_battery_bars(int pct, float volts);
// subs_done is repurposed as star_count for the HFR/Stars graph
void screens_update_imaging(
    float hfr, int star_count, int subs_total, int subs_rejected,
    float exp_elapsed, float exp_total, const char *filter,
    float cam_temp, float cooler_pct, float adu_mean);

void screens_update_focuser(
    int position, float temperature, bool tc_enabled, float tc_coeff,
    float last_af_hfr, int last_af_steps, float last_af_temp,
    float cam_temp);

void screens_update_sky(const char *target, const char *filter,
    float hfr, int subs_done, float rms_total, float flip_min);
void screens_update_battery(int pct, float volts);

#ifdef __cplusplus
}
#endif
