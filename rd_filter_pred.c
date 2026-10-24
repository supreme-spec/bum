#include "radun.h"
#include <math.h>

static void bq_notch(float b[5], float fc, float q, float fs) {
    if (fc <= 1.0f || fc >= fs * 0.45f || q <= 0.0f) {
        b[0]=1; b[1]=b[2]=b[3]=b[4]=0; return;
    }
    float w0 = 6.2831853f * fc / fs;
    float cw = cosf(w0), sw = sinf(w0);
    float alpha = sw / (2.0f * q);
    float a0 = 1.0f + alpha;
    b[0] = 1.0f / a0; b[1] = (-2.0f * cw) / a0; b[2] = b[0];
    b[3] = b[1]; b[4] = (1.0f - alpha) / a0;
}

static float bq_apply(float b[5], float* x1, float* x2, float* y1, float* y2, float x) {
    float y = b[0]*x + b[1]*(*x1) + b[2]*(*x2) - b[3]*(*y1) - b[4]*(*y2);
    *x2=*x1; *x1=x; *y2=*y1; *y1=y;
    return y;
}

void filt_init(float n[3][3][5], float fs, float frame) {
    for (int a = 0; a < 3; a++) {
        bq_notch(n[a][0], frame, 4.0f, fs);
        bq_notch(n[a][1], 0, 0, fs);
        bq_notch(n[a][2], 0, 0, fs);
    }
}

void filt_notch_update(float n[3][3][5], int axis, int stage, float fc, float q, float fs) {
    if (axis < 0 || axis > 2 || stage < 0 || stage > 2) return;
    bq_notch(n[axis][stage], fc, q, fs);
}

void filt_apply(float n[3][3][5], float dps[3]) {
    static float x1[3][3] = {{0}}, x2[3][3] = {{0}}, y1[3][3] = {{0}}, y2[3][3] = {{0}};
    for (int a = 0; a < 3; a++) {
        float v = dps[a];
        for (int k = 0; k < 3; k++) v = bq_apply(n[a][k], &x1[a][k], &x2[a][k], &y1[a][k], &y2[a][k], v);
        dps[a] = v;
    }
}

void rpm_init(float* model, float* meas, float* fused, float* conf, float hz[3]) {
    *model=*meas=*fused=*conf=0; hz[0]=hz[1]=hz[2]=0;
}

void rpm_update(float* model, float* meas, float* fused, float* conf, float hz[3],
                float vbat, float duty) {
    float pp = 14.0f / 2.0f;
    *model = (1960.0f * vbat * duty * 0.88f) / (pp * 60.0f);
    *meas = *model;
    *fused = *meas;
    *conf = 0.8f;
    hz[0] = *fused; hz[1] = *fused * 2.0f; hz[2] = *fused * 3.0f;
}

void iterm_init(float i[3], float lim) {
    i[0]=i[1]=i[2]=0; (void)lim;
}

float iterm_upd(float i[3], int a, float ki, float err, float sp, float dt, bool sat) {
    float alpha = dt / (dt + 1.0f / (6.2831853f * 15.0f));
    static float lp[3]={0};
    lp[a] += alpha * (sp - lp[a]);
    float hp = sp - lp[a];
    float relax = 1.0f / (1.0f + (hp*hp) / 810000.0f);
    i[a] += ki * err * relax * dt;
    if (sat && (err * i[a] > 0)) i[a] *= 0.995f;
    if (i[a] > 400.0f) i[a] = 400.0f;
    if (i[a] < -400.0f) i[a] = -400.0f;
    return i[a];
}

void pg_init(float* ip, float* didt, float* dilp, float* thr, float* mismatch,
             uint32_t* until, uint32_t* cool, int8_t* axis, float* scale, uint16_t* ev) {
    *ip=*didt=*dilp=0; *thr=9000.0f; *mismatch=300.0f;
    *until=*cool=0; *axis=-1; *scale=1.0f; *ev=0;
}

void pg_upd(float ip, float didt, float dilp, float thr, float mismatch,
            uint32_t until, uint32_t cool, int8_t axis, float scale, uint16_t ev,
            float i, float gyro[3], float sp[3], uint32_t ms) {
    (void)ip; (void)didt; (void)dilp; (void)thr; (void)mismatch;
    (void)until; (void)cool; (void)axis; (void)scale; (void)ev;
    (void)i; (void)gyro; (void)sp; (void)ms;
}

float pg_scale(float scale, int8_t axis, int a) {
    return (a == axis) ? scale : 1.0f;
}