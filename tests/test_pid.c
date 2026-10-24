#include <stdio.h>
#include <math.h>
#include "radun.h"

int main(void) {
    float kp[3] = {1,1,1}, ki[3]={0.1f,0.1f,0.1f}, kd[3]={0.01f,0.01f,0.01f}, kf[3]={0,0,0};
    float out[3]={0}, i[3]={0}, gp[3]={0}, dl[3]={0};
    pid_init(kp,ki,kd,kf,out,i,gp,dl);
    float sp[3]={10,0,0}, dps[3]={0,0,0};
    pid_update(kp,ki,kd,kf,sp,dps,0.001f,1000.0f,out,i,gp,dl);
    printf("PID out[0]=%.3f\n", out[0]);
    return 0;
}