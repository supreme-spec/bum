#include "radun.h"

static float sp_filtered[3] = {0, 0, 0};
static float d_weight[3] = {1.0f, 1.0f, 1.0f};  /* D-term setpoint weight */

void pid_init(float kp[3], float ki[3], float kd[3], float kf[3],
              float out[3], float i[3], float gp[3], float dl[3]) {
    kp[0]=35; ki[0]=63; kd[0]=35; kf[0]=30;
    kp[1]=37; ki[1]=53; kd[1]=35; kf[1]=30;
    kp[2]=32; ki[2]=57; kd[2]=0;  kf[2]=30;
    out[0]=out[1]=out[2]=0;
    i[0]=i[1]=i[2]=0;
    gp[0]=gp[1]=gp[2]=0;
    dl[0]=dl[1]=dl[2]=0;
    sp_filtered[0]=sp_filtered[1]=sp_filtered[2]=0;
    d_weight[0]=d_weight[1]=d_weight[2]=1.0f;
}

void pid_set_d_weight(float weight[3]) {
    for (int i = 0; i < 3; i++) {
        if (weight[i] >= 0.0f && weight[i] <= 1.0f) {
            d_weight[i] = weight[i];
        }
    }
}

void pid_update(float kp[3], float ki[3], float kd[3], float kf[3],
                float sp[3], float dps[3], float dt, float thr,
                float out[3], float i[3], float gp[3], float dl[3]) {
    /* TPA on all terms, not just D */
    float tpa = 1.0f;
    if (thr > 1500) tpa = 1.0f - 0.5f * ((thr - 1500) / 500.0f);

    for (int a = 0; a < 3; a++) {
        float err = sp[a] - dps[a];

        /* P term */
        float p_term = kp[a] * err * 0.01f * tpa;

        /* I term with relaxation and anti-windup */
        float i_relax = 1.0f / (1.0f + (sp[a]*sp[a]) / 810000.0f);
        i[a] += ki[a] * err * 0.01f * i_relax * dt;

        /* Anti-windup: reduce I term when saturated */
        if (out[a] > 900.0f && err > 0) i[a] *= 0.98f;
        if (out[a] < -900.0f && err < 0) i[a] *= 0.98f;

        if (i[a] > 400.0f) i[a] = 400.0f;
        if (i[a] < -400.0f) i[a] = -400.0f;

        /* D term with setpoint filtering (reduce derivative kick) */
        sp_filtered[a] += 0.1f * (sp[a] - sp_filtered[a]);  /* Low-pass filter setpoint */
        float d_raw = -(dps[a] - sp_filtered[a] * d_weight[a]) / dt;  /* Apply weight to setpoint */
        dl[a] += 0.15f * (d_raw - dl[a]);
        float d_term = kd[a] * dl[a] * 0.0001f * tpa;

        /* Feed-forward */
        float ff = kf[a] * sp[a] * 0.01f * tpa;

        out[a] = p_term + i[a] + d_term + ff;
        gp[a] = dps[a];

        if (out[a] > 1000.0f) out[a] = 1000.0f;
        if (out[a] < -1000.0f) out[a] = -1000.0f;
    }
}

