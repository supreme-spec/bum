#include "radun.h"
#include <math.h>

static float rls_P[3] = {1e6f, 1e6f, 1e6f};
static float rls_theta[3] = {1.0f, 1.0f, 1.0f};
static float rls_lambda = 0.99f;

void rls_autotune_init(void) {
    rls_P[0] = rls_P[1] = rls_P[2] = 1e6f;
    rls_theta[0] = rls_theta[1] = rls_theta[2] = 1.0f;
    rls_lambda = 0.99f;
}

float rls_autotune_update(int axis, float current_omega, float pid_output) {
    if (axis < 0 || axis > 2) return 1.0f;
    
    float x = pid_output;
    float y = current_omega;
    
    float K = rls_P[axis] * x / (rls_lambda + x * rls_P[axis] * x);
    
    float error = y - rls_theta[axis] * x;
    
    rls_theta[axis] = rls_theta[axis] + K * error;
    
    rls_P[axis] = (1.0f / rls_lambda) * (rls_P[axis] - K * x * rls_P[axis]);
    
    float scale = 1.0f / rls_theta[axis];
    
    if (scale > 2.0f) scale = 2.0f;
    if (scale < 0.5f) scale = 0.5f;
    
    return scale;
}

