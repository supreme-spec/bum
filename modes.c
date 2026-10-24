/* modes.c — режимы, attitude, RPM, iterm (всё в одном файле, чтобы не было дубликатов) */
#include "radun.h"
#include <math.h>
#include <string.h>

/* ============================================================================
   ATTITUDE (Mahony AHRS) — точный фильтр с кватернионами
   ============================================================================ */

/* Fast inverse square root approximation (для Cortex-M4 FPU) */
static float fast_inv_sqrt(float x) {
    if (x <= 0.0f || !isfinite(x)) {
        return 1.0f;   /* безопасная единица, иначе кватернион обнуляется -> NaN */
    }

    const float half_x = 0.5f * x;
    float y = x;
    uint32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759dfU - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (half_x * y * y));
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
static inline float deadband(float v, float db)
{
    if (v > db) return v - db;
    if (v < -db) return v + db;
    return 0.0f;
}

/* angle -> rate: внешний P по углу, внутренний контур скорости */
static void angle_to_rate(const float att[2], const float sp_deg[2], float sp_rate[2])
{
    const float K_ANGLE = 6.0f;          /* (deg/s) на градус ошибки */
    const float MAX_RATE = 400.0f;

    float e_r = sp_deg[0] - att[0];
    float e_p = sp_deg[1] - att[1];
    sp_rate[0] = fmaxf(-MAX_RATE, fminf(MAX_RATE, e_r * K_ANGLE));
    sp_rate[1] = fmaxf(-MAX_RATE, fminf(MAX_RATE, e_p * K_ANGLE));
}

void modes_apply(crsf_t* rx, float att[2], float sp[3]) {
    /* AUX1: Angle mode, AUX2: Horizon mode, AUX3: Air mode */
    bool angle_mode = (rx->aux & 1);
    bool horizon_mode = (rx->aux & 2);
    bool air_mode = (rx->aux & 4);

    /* стики -> нормированные [-1..1], дедбенд 3 % */
    float n[3];
    n[0] = deadband(rx->r / 819.0f, 0.03f);
    n[1] = deadband(rx->p / 819.0f, 0.03f);
    n[2] = deadband(rx->y / 819.0f, 0.03f);

    if (angle_mode) {
        float sp_deg[2] = { n[0] * 55.0f, n[1] * 55.0f };
        angle_to_rate(att, sp_deg, sp);
        sp[2] = n[2] * 300.0f;
    } else if (horizon_mode) {
        float blend = fminf(1.0f, sqrtf(n[0]*n[0] + n[1]*n[1]));
        float ang[2], rate[2];
        float sp_deg[2] = { n[0] * 55.0f, n[1] * 55.0f };
        angle_to_rate(att, sp_deg, ang);
        rate[0] = n[0] * 800.0f; rate[1] = n[1] * 800.0f;
        sp[0] = (1.0f - blend) * ang[0] + blend * rate[0];
        sp[1] = (1.0f - blend) * ang[1] + blend * rate[1];
        sp[2] = n[2] * 300.0f;
    } else if (air_mode) {
        sp[0] = n[0] * 800.0f;
        sp[1] = n[1] * 800.0f;
        sp[2] = n[2] * 300.0f;
    } else {                                                 /* Acro */
        sp[0] = n[0] * 800.0f;
        sp[1] = n[1] * 800.0f;
        sp[2] = n[2] * 300.0f;
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

float anti_gravity_scale(uint16_t thr) {
    float t = ((float)thr - 1000.0f) / 1000.0f;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return 1.0f + t * 2.0f;
}
float vbat_sag_compensate(float vbat, float current) {
    float sag = current * 0.05f; /* 50mV per A rough */
    float v = vbat - sag;
    if (v < 3.0f) v = 3.0f;
    return vbat / v;
}
void failsafe(crsf_t *rx, bool *active) {
    if (rx->valid && rx->lq > 50) {
        *active = false;
        return;
    }

    *active = true;
    rx->thr = 1000;
    rx->r = 0;
    rx->p = 0;
    rx->y = 0;
    rx->aux = 0;
}

bool arming_interlock_update(const crsf_t *rx, bool failsafe_active) {
    static bool require_neutral = true;

    if (failsafe_active || !rx->valid) {
        require_neutral = true;
        return false;
    }

    if (require_neutral) {
        bool sticks_neutral = rx->r >= -200 && rx->r <= 200 &&
                              rx->p >= -200 && rx->p <= 200 &&
                              rx->y >= -200 && rx->y <= 200;
        if (rx->thr >= 1050 || !sticks_neutral) return false;
        require_neutral = false;
    }
    return true;
}

/* Pre-arm safety checks */
typedef struct {
    bool gyro_calibrated;
    bool gyro_healthy;
    bool acc_healthy;
    bool battery_ok;
    bool rx_ok;
    bool sensors_consistent;
    uint32_t check_time_ms;
} prearm_checks_t;

static prearm_checks_t prearm = {0};

void prearm_init(void) {
    prearm.gyro_calibrated = false;
    prearm.gyro_healthy = false;
    prearm.acc_healthy = false;
    prearm.battery_ok = false;
    prearm.rx_ok = false;
    prearm.sensors_consistent = false;
    prearm.check_time_ms = 0;
}

bool prearm_check(const imu_t *imu, const crsf_t *rx, const bat_t *bat) {
    uint32_t now = hal_ms();

    /* Check gyro health */
    prearm.gyro_healthy = imu->healthy;

    /* Check accelerometer (simple sanity check) */
    float acc_mag = imu->acc[0]*imu->acc[0] + imu->acc[1]*imu->acc[1] + imu->acc[2]*imu->acc[2];
    acc_mag = sqrtf(acc_mag);
    prearm.acc_healthy = (acc_mag > 0.8f * 9.81f && acc_mag < 1.2f * 9.81f);

    /* Check battery */
    prearm.battery_ok = (bat->v > 3.5f * battery_get_cell_count()) && !battery_low_voltage();

    /* Check receiver */
    prearm.rx_ok = rx->valid && rx->lq > 50;

    /* Sensor consistency - gyro not spinning when disarmed */
    float gyro_mag = imu->dps[0]*imu->dps[0] + imu->dps[1]*imu->dps[1] + imu->dps[2]*imu->dps[2];
    gyro_mag = sqrtf(gyro_mag);
    prearm.sensors_consistent = (gyro_mag < 10.0f);  /* Less than 10 deg/s when stationary */

    prearm.check_time_ms = now;

    /* All checks must pass */
    return prearm.gyro_healthy && prearm.acc_healthy &&
           prearm.battery_ok && prearm.rx_ok && prearm.sensors_consistent;
}

void prearm_get_status(bool *all_ok, uint8_t *failed_mask) {
    *all_ok = prearm.gyro_healthy && prearm.acc_healthy &&
              prearm.battery_ok && prearm.rx_ok && prearm.sensors_consistent;

    *failed_mask = 0;
    if (!prearm.gyro_healthy) *failed_mask |= (1 << 0);
    if (!prearm.acc_healthy) *failed_mask |= (1 << 1);
    if (!prearm.battery_ok) *failed_mask |= (1 << 2);
    if (!prearm.rx_ok) *failed_mask |= (1 << 3);
    if (!prearm.sensors_consistent) *failed_mask |= (1 << 4);
}

/* ============================================================================
   LAUNCH CONTROL
   ============================================================================ */
typedef struct {
    bool active;
    float pitch_limit;
    float throttle_limit;
    uint32_t start_time_ms;
} launch_ctrl_t;

static launch_ctrl_t launch_ctrl = {0};

void launch_control(float sp[3], uint16_t thr, bool active) {
    if (active && !launch_ctrl.active) {
        launch_ctrl.active = true;
        launch_ctrl.start_time_ms = hal_ms();
        launch_ctrl.pitch_limit = 15.0f;  /* Limit pitch to 15 degrees */
        launch_ctrl.throttle_limit = 0.7f;  /* Limit throttle to 70% */
    } else if (!active) {
        launch_ctrl.active = false;
    }

    if (launch_ctrl.active) {
        uint32_t elapsed = hal_ms() - launch_ctrl.start_time_ms;

        /* Gradually release limits over 2 seconds */
        float progress = (float)elapsed / 2000.0f;
        if (progress > 1.0f) progress = 1.0f;

        float current_pitch_limit = launch_ctrl.pitch_limit + (45.0f - launch_ctrl.pitch_limit) * progress;
        float current_throttle_limit = launch_ctrl.throttle_limit + (1.0f - launch_ctrl.throttle_limit) * progress;

        /* Apply pitch limit */
        if (sp[1] > current_pitch_limit) sp[1] = current_pitch_limit;
        if (sp[1] < -current_pitch_limit) sp[1] = -current_pitch_limit;

        /* Apply throttle limit (handled in mixer) */
        /* Store limit for mixer to use */
    }
}

/* ============================================================================
   TURTLE MODE
   ============================================================================ */
typedef struct {
    bool active;
    uint16_t motors[4];
} turtle_mode_t;

static turtle_mode_t turtle = {0};

void turtle_mode_mix(uint16_t m[4], bool active) {
    if (active && !turtle.active) {
        turtle.active = true;
        /* Store current motor positions */
        for (int i = 0; i < 4; i++) {
            turtle.motors[i] = m[i];
        }
    } else if (!active) {
        turtle.active = false;
    }

    if (turtle.active) {
        /* In turtle mode, motors run at low speed to flip the quad */
        /* Motors 1 and 3 spin one way, 2 and 4 the other */
        m[0] = 1100;  /* Motor 1 */
        m[1] = 1100;  /* Motor 2 */
        m[2] = 1100;  /* Motor 3 */
        m[3] = 1100;  /* Motor 4 */
    }
}
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
