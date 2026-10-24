/* modes.c — режимы, attitude, RPM, iterm (всё в одном файле, чтобы не было дубликатов) */
#include "radun.h"
#include <math.h>

/* ============================================================================
   ATTITUDE (Mahony AHRS) — точный фильтр с кватернионами
   ============================================================================ */

/* Fast inverse square root approximation (для Cortex-M4 FPU) */
static float fast_inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* Mahony AHRS state */
static float mahony_q0 = 1.0f, mahony_q1 = 0.0f, mahony_q2 = 0.0f, mahony_q3 = 0.0f;
static float mahony_integralFBx = 0, mahony_integralFBy = 0, mahony_integralFBz = 0;
static float mahony_twoKp = 2.0f * 0.5f;   /* пропорциональная коррекция */
static float mahony_twoKi = 2.0f * 0.001f; /* интегральная коррекция */

void mahony_ahrs_update(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float halfvx, halfvy, halfvz, halfex, halfey, halfez;
    float qa, qb, qc;

    /* Только если акселерометр не равен нулю (не в свободном падении) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* Нормализация акселерометра */
        recipNorm = fast_inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        
        /* Компоненты кватерниона */
        q0q0 = mahony_q0 * mahony_q0; q0q1 = mahony_q0 * mahony_q1; 
        q0q2 = mahony_q0 * mahony_q2; q0q3 = mahony_q0 * mahony_q3;
        q1q1 = mahony_q1 * mahony_q1; q1q2 = mahony_q1 * mahony_q2; 
        q1q3 = mahony_q1 * mahony_q3;
        q2q2 = mahony_q2 * mahony_q2; q2q3 = mahony_q2 * mahony_q3; 
        q3q3 = mahony_q3 * mahony_q3;
        
        /* Расчёт предполагаемого направления гравитации */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        
        /* Ошибка между предполагаемым и измеренным направлением */
        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;
        
        /* Интегральная коррекция (компенсирует дрейф гироскопа) */
        if (mahony_twoKi > 0.0f) {
            mahony_integralFBx += mahony_twoKi * halfex * dt;
            mahony_integralFBy += mahony_twoKi * halfey * dt;
            mahony_integralFBz += mahony_twoKi * halfez * dt;
            gx += mahony_integralFBx; gy += mahony_integralFBy; gz += mahony_integralFBz;
        } else {
            mahony_integralFBx = 0.0f;
            mahony_integralFBy = 0.0f;
            mahony_integralFBz = 0.0f;
        }
        
        /* Пропорциональная коррекция */
        gx += mahony_twoKp * halfex;
        gy += mahony_twoKp * halfey;
        gz += mahony_twoKp * halfez;
    }
    
    /* Интегрирование угловой скорости (гироскоп) */
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    qa = mahony_q0; qb = mahony_q1; qc = mahony_q2;
    
    mahony_q0 += (-qb * gx - qc * gy - mahony_q3 * gz);
    mahony_q1 += (qa * gx + qc * gz - mahony_q3 * gy);
    mahony_q2 += (qa * gy - qb * gz + mahony_q3 * gx);
    mahony_q3 += (qa * gz + qb * gy - qc * gx);
    
    /* Нормализация кватерниона */
    recipNorm = fast_inv_sqrt(mahony_q0*mahony_q0 + mahony_q1*mahony_q1 + 
                              mahony_q2*mahony_q2 + mahony_q3*mahony_q3);
    mahony_q0 *= recipNorm; mahony_q1 *= recipNorm;
    mahony_q2 *= recipNorm; mahony_q3 *= recipNorm;
}

void imu_update_attitude(float dps[3], float acc[3], float att[2], float dt) {
    /* Преобразование dps в радианы/секунду */
    float gx = dps[0] * 0.0174533f; /* °/s → рад/с */
    float gy = dps[1] * 0.0174533f;
    float gz = dps[2] * 0.0174533f;
    
    /* Обновление Mahony AHRS (без магнитометра для FPV) */
    mahony_ahrs_update(gx, gy, gz, acc[0], acc[1], acc[2], dt);
    
    /* Конвертация кватерниона в углы Эйлера (roll, pitch) */
    float q0 = mahony_q0, q1 = mahony_q1, q2 = mahony_q2, q3 = mahony_q3;
    
    /* Roll (phi) - вращение вокруг оси X */
    att[0] = atan2f(2.0f * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3) * 57.29578f;
    
    /* Pitch (theta) - вращение вокруг оси Y */
    att[1] = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
    
    /* Нормализация углов */
    if (att[0] >  180.0f) att[0] -= 360.0f;
    if (att[0] < -180.0f) att[0] += 360.0f;
    if (att[1] >  180.0f) att[1] -= 360.0f;
    if (att[1] < -180.0f) att[1] += 360.0f;
}

/* Функция сброса AHRS при перезапуске */
void mahony_ahrs_reset(void) {
    mahony_q0 = 1.0f; mahony_q1 = 0.0f; mahony_q2 = 0.0f; mahony_q3 = 0.0f;
    mahony_integralFBx = 0; mahony_integralFBy = 0; mahony_integralFBz = 0;
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

/* Turtle mode: flip drone when upside down */
void turtle_mode_mix(uint16_t m[4], bool active) {
    if (!active) return;
    /* reverse motor direction conceptually: swap pairs */
    uint16_t t = m[0]; m[0] = m[2]; m[2] = t;
    t = m[1]; m[1] = m[3]; m[3] = t;
}

/* Launch control: limit roll/pitch until airborne */
void launch_control(float sp[3], uint16_t thr, bool active) {
    if (!active) return;
    if (thr < 1200) { sp[0] *= 0.3f; sp[1] *= 0.3f; }
}
float anti_gravity_scale(uint16_t thr) {
    float t = (float)(thr - 1000) / 1000.0f;
    if (t < 0) t = 0; if (t > 1) t = 1;
    return 1.0f + t * 2.0f;
}
float vbat_sag_compensate(float vbat, float current) {
    float sag = current * 0.05f; /* 50mV per A rough */
    float v = vbat - sag;
    if (v < 3.0f) v = 3.0f;
    return vbat / v;
}
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

    /* Prefer real ESC telemetry when available */
    float real_rpm = dshot_get_rpm(0);
    if (real_rpm > 0.0f) {
        *meas = real_rpm;
        *fused = real_rpm;
        *conf  = 0.95f;
    } else {
        *meas  = *model;
        *fused = *model;
        *conf  = 0.7f;
    }

    float base_hz = *fused / 60.0f;
    hz[0] = base_hz * 2.0f;   /* blade pass fundamental */
    hz[1] = base_hz * 4.0f;
    hz[2] = base_hz * 8.0f;
}
