#include "rpm_filter.h"
#include "radun.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct {
    float b0,b1,b2,a1,a2;
    float x1,x2,y1,y2;
} notch_t;

static notch_t filters[RPM_FILTER_AXES][RPM_FILTER_MOTORS][RPM_FILTER_HARMONICS];
static float erpm_filt[RPM_FILTER_MOTORS];
static float erpm_raw_prev[RPM_FILTER_MOTORS];
static bool motor_active[RPM_FILTER_MOTORS];
static uint32_t last_coeff_us = 0;
static uint32_t last_telem_us[RPM_FILTER_MOTORS];

static void notch_init(notch_t *f) {
    f->b0=1.0f; f->b1=0.0f; f->b2=0.0f;
    f->a1=0.0f; f->a2=0.0f;
    f->x1=f->x2=f->y1=f->y2=0.0f;
}

static void notch_set(notch_t *f, float f0, float q, float fs) {
    if (fs <= 0.0f) return;
    if (f0 < RPM_FILTER_MIN_HZ || f0 > RPM_FILTER_MAX_HZ || f0 >= fs * 0.48f) {
        notch_init(f);
        return;
    }
    float w0 = 2.0f * M_PI * f0 / fs;
    float cw = cosf(w0);
    float sw = sinf(w0);
    float alpha = sw / (2.0f * q);
    float a0 = 1.0f + alpha;
    f->b0 = (1.0f - cw) * 0.5f / a0;
    f->b1 = (1.0f - cw) / a0;
    f->b2 = (1.0f - cw) * 0.5f / a0;
    f->a1 = (-2.0f * cw) / a0;
    f->a2 = (1.0f - alpha) / a0;
}

static float notch_apply(notch_t *f, float x) {
    float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

static float erpm_to_hz(uint16_t erpm) {
    if (erpm == 0) return 0.0f;
    float mech_rpm = (float)erpm / ((float)MOTOR_POLES / 2.0f);
    return mech_rpm / 60.0f;
}

void rpm_filter_init(void) {
    for (int a=0;a<RPM_FILTER_AXES;a++)
        for (int m=0;m<RPM_FILTER_MOTORS;m++)
            for (int h=0;h<RPM_FILTER_HARMONICS;h++)
                notch_init(&filters[a][m][h]);
    for (int m=0;m<RPM_FILTER_MOTORS;m++) {
        erpm_filt[m]=0.0f;
        erpm_raw_prev[m]=0.0f;
        motor_active[m]=false;
        last_telem_us[m]=0;
    }
    last_coeff_us = 0;
}

void rpm_filter_feed_erpm(uint8_t motor, uint16_t erpm) {
    rpm_filter_update_telemetry(motor, erpm);
}

void rpm_filter_update_telemetry(uint8_t motor, uint16_t erpm) {
    if (motor >= RPM_FILTER_MOTORS) return;
    uint32_t now = micros();
    last_telem_us[motor] = now;
    if (erpm == 0) {
        erpm_filt[motor] *= 0.8f;
        if (erpm_filt[motor] < 10.0f) {
            erpm_filt[motor] = 0.0f;
            motor_active[motor] = false;
        } else {
            motor_active[motor] = true;
        }
        erpm_raw_prev[motor] = 0.0f;
        return;
    }
    float dt = 0.0005f;
    float rc = 1.0f / (2.0f * M_PI * (float)RPM_FILTER_LPF_HZ);
    float alpha = dt / (rc + dt);
    erpm_filt[motor] += alpha * ((float)erpm - erpm_filt[motor]);
    erpm_raw_prev[motor] = (float)erpm;
    motor_active[motor] = (erpm_filt[motor] > 50.0f);
}

void rpm_filter_update_coeffs(void) {
    uint32_t now = micros();
    if (last_coeff_us != 0 && (now - last_coeff_us) < (1000000U/RPM_FILTER_UPDATE_HZ)) return;
    last_coeff_us = now;
    float fs = 3200.0f;
    for (int m=0;m<RPM_FILTER_MOTORS;m++) {
        if (!motor_active[m]) {
            for (int h=0;h<RPM_FILTER_HARMONICS;h++)
                for (int a=0;a<RPM_FILTER_AXES;a++)
                    notch_init(&filters[a][m][h]);
            continue;
        }
        float f0 = erpm_to_hz((uint16_t)erpm_filt[m]);
        if (f0 < 1.0f) continue;
        for (int h=1;h<=RPM_FILTER_HARMONICS;h++) {
            float fh = f0 * (float)h;
            for (int a=0;a<RPM_FILTER_AXES;a++) {
                notch_set(&filters[a][m][h-1], fh, (float)RPM_FILTER_Q, fs);
            }
        }
    }
}

float rpm_filter_apply(uint8_t axis, float in) {
    if (axis >= RPM_FILTER_AXES) return in;
    float y = in;
    for (int m=0;m<RPM_FILTER_MOTORS;m++) {
        if (!motor_active[m]) continue;
        for (int h=0;h<RPM_FILTER_HARMONICS;h++) {
            y = notch_apply(&filters[axis][m][h], y);
        }
    }
    return y;
}

bool rpm_filter_active(uint8_t motor) {
    if (motor >= RPM_FILTER_MOTORS) return false;
    if (last_telem_us[motor] == 0) return motor_active[motor];
    if ((micros() - last_telem_us[motor]) > 500000) {
        motor_active[motor] = false;
        return false;
    }
    return motor_active[motor];
}

bool rpm_filter_healthy(void) {
    bool any = false;
    for (int m=0;m<RPM_FILTER_MOTORS;m++) if (motor_active[m]) any = true;
    return any;
}
