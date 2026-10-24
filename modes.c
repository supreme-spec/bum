#include "radun.h"
#include <math.h>

void imu_update_attitude(float dps[3], float acc[3], float att[2], float dt) {
    float accel_roll = atan2f(acc[1], acc[2]) * 57.2958f;
    float accel_pitch = atan2f(-acc[0], sqrtf(acc[1]*acc[1] + acc[2]*acc[2])) * 57.2958f;
    const float alpha = 0.98f;
    att[0] = alpha * (att[0] + dps[0] * dt) + (1.0f - alpha) * accel_roll;
    att[1] = alpha * (att[1] + dps[1] * dt) + (1.0f - alpha) * accel_pitch;
    if (att[0] > 180.0f) att[0] -= 360.0f;
    if (att[0] < -180.0f) att[0] += 360.0f;
    if (att[1] > 180.0f) att[1] -= 360.0f;
    if (att[1] < -180.0f) att[1] += 360.0f;
}