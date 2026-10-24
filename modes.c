/* modes.c — режимы, attitude, RPM, iterm (всё в одном файле, чтобы не было дубликатов) */
#include "radun.h"
#include <math.h>

/* ============================================================================
   ATTITUDE (комплементарный фильтр) — правильная версия с использованием accel
   ============================================================================ */
void imu_update_attitude(float dps[3], float acc[3], float att[2], float dt) {
    float accel_roll  = atan2f(acc[1], acc[2]) * 57.2958f;
    float accel_pitch = atan2f(-acc[0], sqrtf(acc[1]*acc[1] + acc[2]*acc[2])) * 57.2958f;
    const float alpha = 0.98f;

    att[0] = alpha * (att[0] + dps[0] * dt) + (1.0f - alpha) * accel_roll;
    att[1] = alpha * (att[1] + dps[1] * dt) + (1.0f - alpha) * accel_pitch;

    if (att[0] >  180.0f) att[0] -= 360.0f;
    if (att[0] < -180.0f) att[0] += 360.0f;
    if (att[1] >  180.0f) att[1] -= 360.0f;
    if (att[1] < -180.0f) att[1] += 360.0f;
}

/* ============================================================================
   MODES — преобразование стиков в setpoint
   ============================================================================ */
void modes_apply(crsf_t *rx, float att[2], float sp[3]) {
    sp[0] = (float)rx->r * 0.4f;
    sp[1] = (float)rx->p * 0.4f;
    sp[2] = (float)rx->y * 0.4f;

    /* Angle mode на AUX1 */
    if (rx->aux & 0x01) {
        sp[0] += 60.0f * (0.0f - att[0]);
        sp[1] += 60.0f * (0.0f - att[1]);
    }
}

/* ============================================================================
   MIXER — обычный X-quad (asymmetric_mixer вызывается из radun_advanced)
   ============================================================================ */
void mixer(float out[3], uint16_t thr, uint16_t m[4]) {
    int16_t t  = thr;
    int16_t r  = (int16_t)(out[0] * 0.5f);
    int16_t pi = (int16_t)(out[1] * 0.5f);
    int16_t y  = (int16_t)(out[2] * 0.5f);

    m[0] = t + r + pi + y;
    m[1] = t - r + pi - y;
    m[2] = t - r - pi + y;
    m[3] = t + r - pi - y;

    for (int i = 0; i < 4; i++) {
        if (m[i] < 1000) m[i] = 1000;
        if (m[i] > 2000) m[i] = 2000;
    }
}

/* ============================================================================
   FAILSAFE
   ============================================================================ */
static uint32_t fs_t0 = 0;

void failsafe(crsf_t *rx, bool *active) {
    if (rx->lq > 50) {
        *active = false;
        fs_t0 = 0;
        return;
    }
    if (fs_t0 == 0) fs_t0 = hal_ms();
    if (hal_ms() - fs_t0 > 500) {
        *active = true;
        rx->thr = 1000;
        rx->r = 0;
        rx->p = 0;
        rx->y = 0;
    }
}

/* ============================================================================
   ITERM init (перенесено из attitude.c)
   ============================================================================ */
void iterm_init(float i[3], float lim) {
    (void)lim;
    i[0] = i[1] = i[2] = 0.0f;
}

/* ============================================================================
   RPM model (перенесено из attitude.c)
   ============================================================================ */
void rpm_init(float *model, float *meas, float *fused, float *conf, float hz[3]) {
    *model = *meas = *fused = *conf = 0.0f;
    hz[0] = hz[1] = hz[2] = 0.0f;
}

void rpm_update(float *model, float *meas, float *fused, float *conf,
                float hz[3], float vbat, float duty) {
    *model = 1960.0f * vbat * duty;
    *fused = *model;
    *conf  = 0.7f;

    float base_hz = *fused / 60.0f;
    hz[0] = base_hz * 2.0f;   /* blade pass fundamental */
    hz[1] = base_hz * 4.0f;
    hz[2] = base_hz * 8.0f;
}
