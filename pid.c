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

