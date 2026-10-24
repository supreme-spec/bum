// rd_filter_pred_fixed.c - Исправленные фильтры
#include "radun.h"
#include <math.h>
#include <string.h>

// ============================================================================
// BIQUAD NOTCH FILTER (с управлением состоянием)
// ============================================================================

typedef struct {
    float b[5];          // Коэффициенты: b0,b1,b2,a1,a2
    float x1, x2;        // История входов
    float y1, y2;        // История выходов
    float fc;            // Частота среза
    float q;             // Добротность
    bool active;         // Активен ли фильтр
} biquad_filter_t;

// Инициализация фильтра
void biquad_init(biquad_filter_t *f, float fc, float q, float fs) {
    memset(f, 0, sizeof(biquad_filter_t));
    
    if (fc <= 1.0f || fc >= fs * 0.45f || q <= 0.0f) {
        f->b[0] = 1.0f;
        f->b[1] = f->b[2] = f->b[3] = f->b[4] = 0.0f;
        f->active = false;
    } else {
        float w0 = 6.28318530718f * fc / fs;
        float cw = cosf(w0);
        float sw = sinf(w0);
        float alpha = sw / (2.0f * q);
        float a0 = 1.0f + alpha;
        
        f->b[0] = 1.0f / a0;
        f->b[1] = (-2.0f * cw) / a0;
        f->b[2] = f->b[0];
        f->b[3] = f->b[1];
        f->b[4] = (1.0f - alpha) / a0;
        f->active = true;
        f->fc = fc;
        f->q = q;
    }
}

// Обновление коэффициентов с сохранением состояния
void biquad_update(biquad_filter_t *f, float new_fc, float q, float fs) {
    if (fabsf(new_fc - f->fc) < 0.5f && fabsf(q - f->q) < 0.1f) {
        return;  // Частота не изменилась значительно
    }
    
    // Сохраняем текущее состояние
    float old_x1 = f->x1;
    float old_x2 = f->x2;
    float old_y1 = f->y1;
    float old_y2 = f->y2;
    
    // Инициализируем с новыми параметрами
    biquad_init(f, new_fc, q, fs);
    
    // Восстанавливаем состояние (сглаживаем переход)
    f->x1 = old_x1 * 0.5f;
    f->x2 = old_x2 * 0.5f;
    f->y1 = old_y1 * 0.5f;
    f->y2 = old_y2 * 0.5f;
}

// Применение фильтра
float biquad_apply(biquad_filter_t *f, float x) {
    if (!f->active) return x;
    
    float y = f->b[0] * x + f->b[1] * f->x1 + f->b[2] * f->x2 
              - f->b[3] * f->y1 - f->b[4] * f->y2;
    
    // Update history
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;
    
    return y;
}

// Сброс состояния фильтра
void biquad_reset(biquad_filter_t *f) {
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

// ============================================================================
// FILTER BANK (для 3 осей, 3 гармоник)
// ============================================================================

typedef struct {
    biquad_filter_t notch[3][3];  // [axis][harmonic]
    float last_update_ms[3][3];   // Время последнего обновления
    float target_fc[3][3];        // Целевая частота
    float current_fc[3][3];       // Текущая частота
    float slew_rate;              // Hz/sec slew limit
} filter_bank_t;

static filter_bank_t filter_bank;

// Инициализация банка фильтров
void filter_bank_init(float frame_hz, float fs, float slew_rate) {
    memset(&filter_bank, 0, sizeof(filter_bank_t));
    filter_bank.slew_rate = slew_rate;
    
    // Инициализируем все фильтры
    for (int axis = 0; axis < 3; axis++) {
        // Фиксированный frame notch (112 Hz)
        biquad_init(&filter_bank.notch[axis][0], frame_hz, 4.0f, fs);
        
        // RPM гармоники (изначально выключены)
        biquad_init(&filter_bank.notch[axis][1], 0.0f, 0.0f, fs);
        biquad_init(&filter_bank.notch[axis][2], 0.0f, 0.0f, fs);
        
        filter_bank.target_fc[axis][1] = 0.0f;
        filter_bank.target_fc[axis][2] = 0.0f;
        filter_bank.current_fc[axis][1] = 0.0f;
        filter_bank.current_fc[axis][2] = 0.0f;
    }
}

// Обновление RPM гармоник с slew limiting
void filter_bank_update_rpm(float rpm_hz, float rpm_confidence, uint32_t time_ms) {
    if (rpm_hz < 20.0f || rpm_confidence < 0.3f) {
        // Недостаточно данных - выключаем RPM фильтры
        for (int axis = 0; axis < 3; axis++) {
            filter_bank.target_fc[axis][1] = 0.0f;
            filter_bank.target_fc[axis][2] = 0.0f;
        }
        return;
    }
    
    float harmonics[2] = {rpm_hz, rpm_hz * 2.0f};
    
    for (int axis = 0; axis < 3; axis++) {
        for (int h = 0; h < 2; h++) {
            uint32_t harmonic_idx = h + 1;  // 1 и 2 гармоники
            
            // Slew limiting
            float delta = harmonics[h] - filter_bank.target_fc[axis][harmonic_idx];
            float max_delta = filter_bank.slew_rate * 0.001f *  // Hz/ms
                             (time_ms - filter_bank.last_update_ms[axis][harmonic_idx]);
            
            if (delta > max_delta) delta = max_delta;
            if (delta < -max_delta) delta = -max_delta;
            
            filter_bank.target_fc[axis][harmonic_idx] += delta;
            
            // Обновляем фильтр только если частота изменилась
            if (fabsf(filter_bank.target_fc[axis][harmonic_idx] - 
                     filter_bank.current_fc[axis][harmonic_idx]) > 0.5f) {
                biquad_update(&filter_bank.notch[axis][harmonic_idx],
                             filter_bank.target_fc[axis][harmonic_idx],
                             150.0f,  // Q factor для RPM notch
                             3200.0f);
                filter_bank.current_fc[axis][harmonic_idx] = 
                    filter_bank.target_fc[axis][harmonic_idx];
            }
            
            filter_bank.last_update_ms[axis][harmonic_idx] = time_ms;
        }
    }
}

// Применение всех фильтров к gyro data
void filter_bank_apply(float dps[3]) {
    for (int axis = 0; axis < 3; axis++) {
        float value = dps[axis];
        
        // Применяем все три фильтра
        for (int f = 0; f < 3; f++) {
            if (filter_bank.notch[axis][f].b[0] == 1.0f &&
                filter_bank.notch[axis][f].b[1] == 0.0f &&
                filter_bank.notch[axis][f].b[3] == 0.0f) {
                continue;
            }
            value = biquad_apply(&filter_bank.notch[axis][f], value);
        }
        
        dps[axis] = value;
    }
}

// Сброс всех состояний (при disarm)
void filter_bank_reset(void) {
    for (int axis = 0; axis < 3; axis++) {
        for (int f = 0; f < 3; f++) {
            biquad_reset(&filter_bank.notch[axis][f]);
        }
    }
}

// Получение текущих частот фильтров
void filter_bank_get_frequencies(float fc[3][3]) {
    for (int axis = 0; axis < 3; axis++) {
        for (int f = 0; f < 3; f++) {
            fc[axis][f] = filter_bank.current_fc[axis][f];
        }
    }
}

// ============================================================================
// PROPGUARD - РАБОЧАЯ РЕАЛИЗАЦИЯ
// ============================================================================

typedef struct {
    float ip;                // Предыдущий ток
    float didt;              // Производная тока
    float dilp;              // Сглаженная производная
    float threshold;         // Порог срабатывания (9000 из твоих логов)
    float mismatch_thresh;   // Порог рассогласования gyro/sp (300)
    float scale[3];          // Масштабирование по осям
    uint32_t cooldown_until; // Время окончания cooldown
    uint32_t cooldown_ms;    // Длительность cooldown
    int8_t triggered_axis;   // Сработавшая ось
    float event_value;       // Значение события
    uint32_t event_count;    // Счетчик событий
} propguard_t;

static propguard_t propguard = {0};

// Инициализация PropGuard
void propguard_init(float threshold, float mismatch, uint32_t cooldown_ms) {
    memset(&propguard, 0, sizeof(propguard_t));
    propguard.threshold = threshold;
    propguard.mismatch_thresh = mismatch;
    propguard.cooldown_ms = cooldown_ms;
    propguard.scale[0] = propguard.scale[1] = propguard.scale[2] = 1.0f;
    propguard.triggered_axis = -1;
}

// Обновление PropGuard
void propguard_update(float current, float gyro[3], float sp[3], uint32_t time_ms) {
    // Вычисляем dI/dt
    static uint32_t last_update_ms = 0;
    float dt = (time_ms - last_update_ms) * 0.001f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;  // Минимум 1ms
    
    propguard.didt = (current - propguard.ip) / dt;
    propguard.ip = current;
    
    // Сглаживаем dI/dt (tau ≈ 2.8ms)
    propguard.dilp += 0.35f * (propguard.didt - propguard.dilp);
    
    last_update_ms = time_ms;
    
    // Проверяем cooldown
    if (time_ms < propguard.cooldown_until) {
        // В режиме восстановления
        float progress = (float)(propguard.cooldown_until - time_ms) / propguard.cooldown_ms;
        float recovery_scale = 0.2f + 0.8f * (1.0f - progress);
        
        for (int i = 0; i < 3; i++) {
            if (propguard.scale[i] < recovery_scale) {
                propguard.scale[i] = recovery_scale;
            }
        }
        return;
    }
    
    // Проверяем скачок тока
    if (propguard.dilp > propguard.threshold) {
        // Находим ось с максимальным рассогласованием
        float max_mismatch = 0.0f;
        int max_axis = -1;
        
        for (int i = 0; i < 3; i++) {
            float mismatch = fabsf(gyro[i] - sp[i]);
            if (mismatch > max_mismatch && mismatch > propguard.mismatch_thresh) {
                max_mismatch = mismatch;
                max_axis = i;
            }
        }
        
        if (max_axis >= 0) {
            // Срабатывание защиты
            propguard.triggered_axis = max_axis;
            propguard.event_value = propguard.dilp;
            propguard.event_count++;
            propguard.cooldown_until = time_ms + propguard.cooldown_ms;
            
            // Уменьшаем усиление на сработавшей оси
            propguard.scale[max_axis] = 0.2f;
            
            // Можно добавить логгирование в blackbox
            // flash_log_propguard_event(max_axis, propguard.dilp, mismatch);
        }
    } else {
        // Нормальная работа - постепенно восстанавливаем масштаб
        for (int i = 0; i < 3; i++) {
            if (propguard.scale[i] < 1.0f) {
                propguard.scale[i] += 0.01f;  // 1% восстановления за вызов
                if (propguard.scale[i] > 1.0f) propguard.scale[i] = 1.0f;
            }
        }
    }
}

// Получение масштаба для оси
float propguard_get_scale(int axis) {
    if (axis < 0 || axis > 2) return 1.0f;
    return propguard.scale[axis];
}

// Сброс PropGuard
void propguard_reset(void) {
    propguard.scale[0] = propguard.scale[1] = propguard.scale[2] = 1.0f;
    propguard.triggered_axis = -1;
    propguard.cooldown_until = 0;
}

// Получение диагностики PropGuard
void propguard_get_diagnostics(int8_t *triggered_axis, float *scale, 
                              uint32_t *event_count, float *current_dilp) {
    *triggered_axis = propguard.triggered_axis;
    scale[0] = propguard.scale[0];
    scale[1] = propguard.scale[1];
    scale[2] = propguard.scale[2];
    *event_count = propguard.event_count;
    *current_dilp = propguard.dilp;
}

// ============================================================================
// ИНТЕРФЕЙС ДЛЯ СУЩЕСТВУЮЩЕГО КОДА (обратная совместимость)
// ============================================================================

// filt_*_compat provided as static inline in radun.h

float iterm_upd_compat(float i[3], int a, float ki, float err, float sp, float dt, bool sat) {
    // Сохраняем старую реализацию (она работает)
    float alpha = dt / (dt + 1.0f / (6.2831853f * 15.0f));
    static float lp[3] = {0};
    lp[a] += alpha * (sp - lp[a]);
    float hp = sp - lp[a];
    float relax = 1.0f / (1.0f + (hp*hp) / 810000.0f);
    
    i[a] += ki * err * relax * dt;
    
    if (sat && (err * i[a] > 0)) i[a] *= 0.995f;
    if (i[a] > 400.0f) i[a] = 400.0f;
    if (i[a] < -400.0f) i[a] = -400.0f;
    
    return i[a];
}

void pg_init_compat(float* ip, float* didt, float* dilp, float* thr, float* mismatch,
                   uint32_t* until, uint32_t* cool, int8_t* axis, float* scale, uint16_t* ev) {
    propguard_init(9000.0f, 300.0f, 100);
    *scale = 1.0f;
    *axis = -1;
}

void pg_upd_compat(float ip, float didt, float dilp, float thr, float mismatch,
                  uint32_t until, uint32_t cool, int8_t axis, float scale, uint16_t ev,
                  float i, float gyro[3], float sp[3], uint32_t ms) {
    propguard_update(i, gyro, sp, ms);
}

float pg_scale_compat(float scale, int8_t axis, int a) {
    (void)scale; (void)axis;
    return propguard_get_scale(a);
}
