#pragma once
#include "radun.h"

typedef struct {
    float theta[2];      // [a1, b1] - model params
    float P[2][2];       // Error covariance
    float lambda;        // Forgetting factor (0.995)
    float last_omega;    // Previous gyro rate
    float last_u;        // Previous control output
} rls_t;

extern rls_t axis_rls[3];

float rls_autotune_update(int axis, float current_omega, float pid_output);
void rls_autotune_init(void);