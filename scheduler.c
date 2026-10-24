#include "scheduler.h"
#include "radun.h"

#define MAX_TASKS 16
#define GYRO_LOCK_SAMPLES 50

static task_t *tasks[MAX_TASKS];
static uint8_t task_cnt = 0;

static volatile uint32_t gyro_period_us = 0;
static volatile uint32_t last_gyro_us = 0;
static volatile uint32_t gyro_cnt = 0;
static volatile bool gyro_locked = false;

void scheduler_init(void) {
    task_cnt = 0;
    gyro_period_us = 0;
    last_gyro_us = 0;
    gyro_cnt = 0;
    gyro_locked = false;
}

void scheduler_register(task_t *t) {
    if (task_cnt < MAX_TASKS) {
        tasks[task_cnt++] = t;
    }
}

void scheduler_gyro_isr(void) {
    uint32_t now = micros();
    if (last_gyro_us != 0) {
        uint32_t dt = now - last_gyro_us;
        if (gyro_period_us == 0) {
            gyro_period_us = dt;
        } else {
            gyro_period_us = (gyro_period_us * 15 + dt) / 16;
        }
        gyro_cnt++;
        if (gyro_cnt >= GYRO_LOCK_SAMPLES) {
            gyro_locked = true;
        }
    }
    last_gyro_us = now;
}

static task_t *scheduler_pick(uint32_t now) {
    task_t *best = NULL;
    uint32_t best_dyn = 0;

    for (uint8_t i = 0; i < task_cnt; i++) {
        task_t *t = tasks[i];
        uint32_t elapsed = now - t->last_run_us;

        if (t->realtime && gyro_locked) {
            if (gyro_period_us == 0) continue;
            if (elapsed >= gyro_period_us) {
                return t;
            }
            continue;
        }

        if (t->period_us == 0) continue;
        if (elapsed < t->period_us) continue;

        t->age_periods = elapsed / t->period_us;
        if (t->age_periods < 1) t->age_periods = 1;
        uint32_t dyn = (uint32_t)t->prio * t->age_periods + 1;
        if (dyn > best_dyn) {
            best_dyn = dyn;
            best = t;
        }
    }
    return best;
}

void scheduler_run(void) {
    uint32_t now = micros();
    task_t *t = scheduler_pick(now);
    if (!t || !t->fn) return;
    t->fn();
    t->last_run_us = micros();
    t->age_periods = 0;
}
