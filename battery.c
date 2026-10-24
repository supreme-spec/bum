#include "radun.h"

#define LVC_CELL 3.3f
#define LVC_HYST 3.5f
#define SAG_COMP 0.05f

static float vbat_filtered = 0.0f;
static float vbat_compensated = 0.0f;
static uint8_t cell_count = 0;
static bool lvc_triggered = false;

void battery_init(void) {
    vbat_filtered = 0.0f;
    vbat_compensated = 0.0f;
    cell_count = 0;
    lvc_triggered = false;
}

void battery_update(float vbat, float current) {
    /* Filter voltage */
    vbat_filtered += 0.1f * (vbat - vbat_filtered);

    /* Detect cell count on first run */
    if (cell_count == 0 && vbat_filtered > 10.0f) {
        if (vbat_filtered < 4.3f) cell_count = 1;
        else if (vbat_filtered < 8.6f) cell_count = 2;
        else if (vbat_filtered < 12.9f) cell_count = 3;
        else if (vbat_filtered < 17.2f) cell_count = 4;
        else if (vbat_filtered < 21.5f) cell_count = 5;
        else if (vbat_filtered < 25.8f) cell_count = 6;
        else cell_count = 4; /* Default to 4S */
    }

    /* Voltage sag compensation */
    float sag = current * SAG_COMP;
    vbat_compensated = vbat_filtered + sag;

    /* LVC check */
    if (cell_count > 0) {
        float cell_v = vbat_filtered / cell_count;
        if (cell_v < LVC_CELL) {
            lvc_triggered = true;
        } else if (cell_v > LVC_HYST) {
            lvc_triggered = false;
        }
    }
}

float battery_get_compensated_voltage(void) {
    return vbat_compensated;
}

bool battery_low_voltage(void) {
    return lvc_triggered;
}

uint8_t battery_get_cell_count(void) {
    return cell_count;
}
