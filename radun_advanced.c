#include "radun_advanced.h"
#include <math.h>

/* ============================================================================
   PREDICTIVE FEED-FORWARD by Jerk
   ============================================================================ */
pred_ff_t pred_ff = {{0},{0},{0}};

void pred_ff_init(float kf[3]) {
    for (int i = 0; i < 3; i++) {
        pred_ff.last_setpoint[i] = 0;
        pred_ff.jerk[i] = 0;
        pred_ff.kf_pred[i] = kf[i];
    }
}

void pred_ff_update(float sp[3], float ff_out[3], float dt) {
    for (int i = 0; i < 3; i++) {
        float jerk = (sp[i] - pred_ff.last_setpoint[i]) / dt;
        pred_ff.jerk[i] = jerk;
        float predictive_sp = sp[i] + jerk * 0.002f;
        ff_out[i] = pred_ff.kf_pred[i] * predictive_sp * 0.01f;
        pred_ff.last_setpoint[i] = sp[i];
    }
}

/* ============================================================================
   CRASH RECOVERY
   ============================================================================ */
crash_rec_t crash_rec = {false, 0, 0};

void crash_rec_init(void) {
    crash_rec.detected = false;
    crash_rec.timer_ms = 0;
    crash_rec.start_ms = 0;
}

bool crash_rec_check(float gyro[3], uint16_t throttle, bool armed) {
    if (!armed) {
        crash_rec.detected = false;
        return false;
    }

    bool extreme_spin = (fabsf(gyro[0]) > 1200.0f ||
                         fabsf(gyro[1]) > 1200.0f ||
                         fabsf(gyro[2]) > 1200.0f);
    bool low_throttle = (throttle < 1050);

    if (extreme_spin && low_throttle && !crash_rec.detected) {
        crash_rec.detected = true;
        crash_rec.start_ms = hal_ms();
        crash_rec.timer_ms = hal_ms();
    }

    if (crash_rec.detected) {
        if (hal_ms() - crash_rec.timer_ms > 500) {
            crash_rec.detected = false;
        }
    }

    return crash_rec.detected;
}

void crash_rec_reset_pid(float i[3]) {
    i[0] = 0.0f;
    i[1] = 0.0f;
    i[2] = 0.0f;
}

/* ============================================================================
   BLADE-LOSS COMPENSATION
   ============================================================================ */
blade_loss_t blade_loss = {{1,1,1,1}, false, -1};

void blade_loss_init(void) {
    for (int i = 0; i < 4; i++) blade_loss.efficiency[i] = 1.0f;
    blade_loss.active = false;
    blade_loss.damaged_idx = -1;
    blade_loss.timer_ms = 0;
}

void blade_loss_detect(const float currents[4], const float gyro[3]) {
    float avg = (currents[0] + currents[1] + currents[2] + currents[3]) / 4.0f;
    if (avg < 2.0f) return;

    for (int i = 0; i < 4; i++) {
        if (currents[i] < avg * 0.4f) {
            blade_loss.active = true;
            blade_loss.damaged_idx = i;
            blade_loss.timer_ms = hal_ms();
            blade_loss.efficiency[i] = 0.2f;
            return;
        }
    }
}

void asymmetric_mixer(float out[3], uint16_t thr, uint16_t m[4], const float eff[4]) {
    float r = out[0] * 0.5f;
    float pi = out[1] * 0.5f;
    float y = out[2] * 0.5f;

    float raw_m[4];
    raw_m[0] = thr + r + pi + y;
    raw_m[1] = thr - r + pi - y;
    raw_m[2] = thr - r - pi + y;
    raw_m[3] = thr + r - pi - y;

    if (blade_loss.active && blade_loss.damaged_idx >= 0) {
        int opp = (blade_loss.damaged_idx == 0) ? 2 :
                  (blade_loss.damaged_idx == 1) ? 3 :
                  (blade_loss.damaged_idx == 2) ? 0 : 1;
        raw_m[opp] += (2000.0f - raw_m[blade_loss.damaged_idx]) * 0.5f;
    }

    for (int i = 0; i < 4; i++) {
        float comp = 1000.0f + (raw_m[i] - 1000.0f) * eff[i];
        if (comp < 1000.0f) comp = 1000.0f;
        if (comp > 2000.0f) comp = 2000.0f;
        m[i] = (uint16_t)comp;
    }
}

/* ============================================================================
   ML CRASH DETECTION (TinyML)
   ============================================================================ */
typedef struct { float w1[3][5]; float b1[5]; float w2[5]; float b2; } crash_nn_t;
static crash_nn_t crash_model = {
    /* weights1 (3x5) */
    {{0.5f,0.3f,-0.2f,0.1f,0.0f},{-0.4f,0.6f,0.2f,-0.1f,0.3f},{0.1f,-0.3f,0.5f,0.4f,-0.2f}},
    {0.1f,-0.1f,0.0f,0.2f,-0.1f},
    /* weights2 (5) */
    {0.8f,-0.6f,0.4f,0.3f,-0.5f},
    -0.2f
};

static float relu(float x){ return x>0?x:0; }
static float sigmoid(float x){ return 1.0f/(1.0f+expf(-x)); }

bool nn_crash_detect(float gyro[3]) {
    float hidden[5];
    for (int i=0;i<5;i++){
        float s = crash_model.b1[i];
        for (int j=0;j<3;j++) s += gyro[j]*crash_model.w1[j][i];
        hidden[i] = relu(s);
    }
    float out = crash_model.b2;
    for (int i=0;i<5;i++) out += hidden[i]*crash_model.w2[i];
    return sigmoid(out) > 0.8f;
}
current_notch_t cur_notch = {0, 0};

void cur_notch_init(void) {
    cur_notch.estimated_hz = 0;
    cur_notch.filtered_current = 0;
}

float cur_notch_update(float duty, float vbat, int pole_pairs) {
    float pp = (float)pole_pairs / 2.0f;
    float electrical_rpm = (1960.0f * vbat * duty) / pp;
    float hz = electrical_rpm / 60.0f;
    cur_notch.estimated_hz = hz;
    return hz;
}