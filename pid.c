#include "radun.h"

void pid_init(float kp[3], float ki[3], float kd[3], float kf[3],
              float out[3], float i[3], float gp[3], float dl[3]) {
    kp[0]=35; ki[0]=63; kd[0]=35; kf[0]=30;
    kp[1]=37; ki[1]=53; kd[1]=35; kf[1]=30;
    kp[2]=32; ki[2]=57; kd[2]=0;  kf[2]=30;
    out[0]=out[1]=out[2]=0;
    i[0]=i[1]=i[2]=0;
    gp[0]=gp[1]=gp[2]=0;
    dl[0]=dl[1]=dl[2]=0;
}

void pid_update(float kp[3], float ki[3], float kd[3], float kf[3],
                float sp[3], float dps[3], float dt, float thr,
                float out[3], float i[3], float gp[3], float dl[3]) {
    float tpa = 1.0f;
    if (thr > 1500) tpa = 1.0f - 0.5f * ((thr - 1500) / 500.0f);
    for (int a = 0; a < 3; a++) {
        float err = sp[a] - dps[a];
        float p_term = kp[a] * err * 0.01f;
        float i_relax = 1.0f / (1.0f + (sp[a]*sp[a]) / 810000.0f);
        i[a] += ki[a] * err * 0.01f * i_relax * dt;
        if (i[a] > 400.0f) i[a] = 400.0f;
        if (i[a] < -400.0f) i[a] = -400.0f;
        float d_raw = -(dps[a] - gp[a]) / dt;
        dl[a] += 0.15f * (d_raw - dl[a]);
        float d_term = kd[a] * dl[a] * 0.0001f * tpa;
        float ff = kf[a] * sp[a] * 0.01f;
        out[a] = p_term + i[a] + d_term + ff;
        gp[a] = dps[a];
        if (out[a] > 1000.0f) out[a] = 1000.0f;
        if (out[a] < -1000.0f) out[a] = -1000.0f;
    }
}

void modes_apply(crsf_t* rx, float att[2], float sp[3]) {
    sp[0] = (float)rx->r * 0.4f;
    sp[1] = (float)rx->p * 0.4f;
    sp[2] = (float)rx->y * 0.4f;
    if (rx->aux & 0x01) {
        sp[0] += 60.0f * (0.0f - att[0]);
        sp[1] += 60.0f * (0.0f - att[1]);
    }
}

void mixer(float out[3], uint16_t thr, uint16_t m[4]) {
    int16_t t = thr, r = out[0]*0.5f, pi = out[1]*0.5f, y = out[2]*0.5f;
    m[0] = t + r + pi + y;
    m[1] = t - r + pi - y;
    m[2] = t - r - pi + y;
    m[3] = t + r - pi - y;
    for (int i = 0; i < 4; i++) {
        if (m[i] < 1000) m[i] = 1000;
        if (m[i] > 2000) m[i] = 2000;
    }
}

static uint32_t fs_t0;
void failsafe(crsf_t* rx, bool* active) {
    if (rx->lq > 50) { *active = false; fs_t0 = 0; return; }
    if (fs_t0 == 0) fs_t0 = hal_ms();
    if (hal_ms() - fs_t0 > 500) {
        *active = true;
        rx->thr = 1000; rx->r = 0; rx->p = 0; rx->y = 0;
    }
}