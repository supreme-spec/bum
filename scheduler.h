#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TASK_PRIO_REALTIME = 0,
    TASK_PRIO_HIGH     = 1,
    TASK_PRIO_MEDIUM   = 2,
    TASK_PRIO_LOW      = 3,
} task_prio_e;

typedef struct task_s {
    const char *name;
    void (*fn)(void);
    uint32_t period_us;
    task_prio_e prio;
    bool realtime;
    uint32_t last_run_us;
    uint32_t age_periods;
} task_t;

void scheduler_init(void);
void scheduler_register(task_t *t);
void scheduler_run(void);
void scheduler_gyro_isr(void);
