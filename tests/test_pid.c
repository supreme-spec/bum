#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "radun.h"

int main(void) {
    float kp[3] = {1,1,1}, ki[3]={0.1f,0.1f,0.1f}, kd[3]={0.01f,0.01f,0.01f}, kf[3]={0,0,0};
    float out[3]={0}, i[3]={0}, gp[3]={0}, dl[3]={0};
    pid_init(kp,ki,kd,kf,out,i,gp,dl);
    float sp[3]={10,0,0}, dps[3]={0,0,0};
    pid_update(kp,ki,kd,kf,sp,dps,0.001f,1000.0f,out,i,gp,dl);
    printf("PID out[0]=%.3f\n", out[0]);

    assert(fabsf(anti_gravity_scale(900) - 1.0f) < 0.001f);
    assert(fabsf(anti_gravity_scale(1500) - 2.0f) < 0.001f);
    assert(fabsf(anti_gravity_scale(2000) - 3.0f) < 0.001f);

    crsf_t receiver = {.r = 100, .p = -100, .y = 800, .thr = 1500,
                       .aux = 7, .lq = 0, .valid = false};
    bool failsafe_active = false;
    failsafe(&receiver, &failsafe_active);
    assert(failsafe_active);
    assert(receiver.r == 0 && receiver.p == 0 && receiver.y == 0);
    assert(receiver.thr == 1000 && receiver.aux == 0);
    assert(!arming_interlock_update(&receiver, failsafe_active));

    receiver.valid = true;
    receiver.lq = 0xFF;
    receiver.thr = 1200;
    failsafe(&receiver, &failsafe_active);
    assert(!failsafe_active);
    assert(receiver.thr == 1200);
    receiver.r = 0;
    receiver.p = 0;
    receiver.y = 800;
    assert(!arming_interlock_update(&receiver, failsafe_active));
    receiver.y = 0;
    receiver.thr = 1000;
    assert(arming_interlock_update(&receiver, failsafe_active));
    receiver.y = 800;
    assert(arming_interlock_update(&receiver, failsafe_active));
    assert(!dshot_output_available());

    for (int sample = 0; sample < 64; sample++) {
        float phase = 6.2831853f * 100.0f * sample / 3200.0f;
        fft_update_gyro(sinf(phase));
    }
    fft_process();
    float peaks[5];
    int peak_count = fft_get_peaks(peaks, 5);
    assert(peak_count > 0);
    assert(fabsf(peaks[0] - 100.0f) < 0.1f);

    return 0;
}