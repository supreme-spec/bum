#include "radun.h"

#define PID_ERR_NORM    (1.0f / 500.0f)      /* 500 °/s -> 1.0 */
#define PID_D_LPF_HZ    100.0f
#define PID_I_LIMIT     0.5f

static float sp_filtered[3] = {0, 0, 0};
static float d_weight[3] = {1.0f, 1.0f, 1.0f};

void pid_init(float kp[3], float ki[3], float kd[3], float kf[3],
              float out[3], float i[3], float gp[3], float dl[3]) {
    kp[0]=45; ki[0]=0.35f; kd[0]=0.0012f; kf[0]=0.0f;
    kp[1]=45; ki[1]=0.35f; kd[1]=0.0012f; kf[1]=0.0f;
    kp[2]=35; ki[2]=0.20f; kd[2]=0.0f;    kf[2]=0.0f;
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
    /* TPA: режем усиление выше 50 % газа, линейно до 20 % на 100 % */
    float tpa = 1.0f;
    if (thr > 1500.0f) tpa = 1.0f - 0.8f * (thr - 1500.0f) / 500.0f;
    if (tpa < 0.2f) tpa = 0.2f;

    float a_d = dt / (dt + 1.0f / (6.2831853f * PID_D_LPF_HZ));

    for (int a = 0; a < 3; a++) {
        /* err и sp в °/s, нормализованные к 1.0 при 500 °/s */
        float err = (sp[a] - dps[a]) * PID_ERR_NORM;

        /* P */
        float p = kp[a] * err;

        /* I с anti-windup (back-calculation при насыщении) */
        i[a] += ki[a] * err * dt;
        if (i[a] >  PID_I_LIMIT) i[a] =  PID_I_LIMIT;
        if (i[a] < -PID_I_LIMIT) i[a] = -PID_I_LIMIT;

        /* D по измерению (без derivative kick), PT1 с срезом в Гц */
        float d_meas = -(dps[a] - gp[a]) * PID_ERR_NORM / dt;
        dl[a] += a_d * (d_meas - dl[a]);

        /* feed-forward */
        float ff = kf[a] * (sp[a] - sp_filtered[a]) * PID_ERR_NORM / dt * dt;

        float sum = p + i[a] + kd[a] * dl[a] + ff;
        sum *= tpa;

        /* anti-windup back-calculation */
        if (sum > 1.0f)  { i[a] -= (sum - 1.0f) * 0.5f; sum = 1.0f; }
        if (sum < -1.0f) { i[a] -= (sum + 1.0f) * 0.5f; sum = -1.0f; }

        out[a] = sum;
        gp[a] = dps[a];
        sp_filtered[a] += 0.1f * (sp[a] - sp_filtered[a]);
    }
}
