#include "radun.h"
#include <math.h>

void imu_update_attitude(float dps[3], float acc[3], float att[2], float dt) {
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    
    // Normalize accelerometer
    float norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (norm > 0.01f) {
        acc[0] /= norm;
        acc[1] /= norm;
        acc[2] /= norm;
    }
    
    // Gyro integration (simplified)
    float half_dt = 0.5f * dt;
    float gx = dps[0] * 0.0174533f;
    float gy = dps[1] * 0.0174533f;
    float gz = dps[2] * 0.0174533f;
    
    float q0t = q0 + (-q1*gx - q2*gy - q3*gz) * half_dt;
    float q1t = q1 + (q0*gx - q3*gy + q2*gz) * half_dt;
    float q2t = q2 + (q3*gx + q0*gy - q1*gz) * half_dt;
    float q3t = q3 + (-q2*gx + q1*gy + q0*gz) * half_dt;
    
    // Normalize quaternion
    norm = sqrtf(q0t*q0t + q1t*q1t + q2t*q2t + q3t*q3t);
    if (norm > 0.01f) {
        q0 = q0t / norm;
        q1 = q1t / norm;
        q2 = q2t / norm;
        q3 = q3t / norm;
    }
    
    // Convert to roll/pitch
    att[0] = atan2f(2.0f * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3) * 57.29578f;
    att[1] = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
}

void iterm_init(float i[3], float lim) {
    i[0] = i[1] = i[2] = 0.0f;
    // lim parameter is ignored - handled in main loop
}

void rpm_init(float* model, float* meas, float* fused, float* conf, float hz[3]) {
    *model = *meas = *fused = *conf = 0.0f;
    hz[0] = hz[1] = hz[2] = 0.0f;
}

void rpm_update(float* model, float* meas, float* fused, float* conf, float hz[3], float vbat, float duty) {
    *model = 1960.0f * vbat * duty;
    *fused = *model;
    *conf = 0.7f;
    
    // Update filter frequencies based on RPM
    float base_hz = *fused / 60.0f;
    hz[0] = base_hz * 2.0f;
    hz[1] = base_hz * 4.0f;
    hz[2] = base_hz * 8.0f;
}