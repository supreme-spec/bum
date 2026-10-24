/* flash_log.c — запись в SPI flash через RAM буфер (не в ISR!) */
#include "radun.h"
#include <string.h>

#define FLASH_LOG_PAGE_SIZE 256
#define LOG_RING_SIZE 32

typedef struct __attribute__((packed)) {
    uint32_t iteration;
    int16_t  gyro[3];
    int16_t  setpoint[3];
    int16_t  pid_out[3];
    uint16_t motors[4];
    uint16_t vbat;
    uint8_t  cpu_load;
    uint8_t  flags;
} blackbox_frame_t;

static uint32_t log_write_address = 0;
static bool logging_enabled = false;
static uint8_t page_buffer[FLASH_LOG_PAGE_SIZE];
static uint16_t page_buf_offset = 0;

static blackbox_frame_t log_ring[LOG_RING_SIZE];
static volatile uint16_t ring_write_idx = 0;
static volatile uint16_t ring_read_idx = 0;

static void flash_cs_low(void)  { gpio_clr(FLASH_CS_PORT, FLASH_CS_PIN); }
static void flash_cs_high(void) { gpio_set(FLASH_CS_PORT, FLASH_CS_PIN); }
static uint8_t flash_spi_rw(uint8_t byte) { return spi_rw(FLASH_SPI, byte); }

void flash_log_init(void) {
    RCC->APB1 |= (1 << 14);
    gpio_af(GPIOB, GPIO_PIN_13, 5);
    gpio_af(GPIOB, GPIO_PIN_14, 5);
    gpio_af(GPIOB, GPIO_PIN_15, 5);
    gpio_out(GPIOB, GPIO_PIN_12);
    gpio_set(GPIOB, GPIO_PIN_12);
    gpio_out(FLASH_CS_PORT, FLASH_CS_PIN);
    flash_cs_high();
    log_write_address = 0;
    logging_enabled = false;
    page_buf_offset = 0;
    ring_write_idx = ring_read_idx = 0;
}

static void flash_write_enable(void) {
    flash_cs_low(); flash_spi_rw(0x06); flash_cs_high();
}

static void flash_wait_not_busy(void) {
    flash_cs_low();
    flash_spi_rw(0x05);
    while (flash_spi_rw(0xFF) & 0x01);
    flash_cs_high();
}

static void flash_erase_sector(uint32_t address) {
    flash_write_enable();
    flash_cs_low();
    flash_spi_rw(0x20);
    flash_spi_rw((address >> 16) & 0xFF);
    flash_spi_rw((address >> 8)  & 0xFF);
    flash_spi_rw( address        & 0xFF);
    flash_cs_high();
    flash_wait_not_busy();
}

static void flash_flush_page(void) {
    if (page_buf_offset == 0) return;
    flash_write_enable();
    flash_cs_low();
    flash_spi_rw(0x02);
    flash_spi_rw((log_write_address >> 16) & 0xFF);
    flash_spi_rw((log_write_address >> 8)  & 0xFF);
    flash_spi_rw( log_write_address        & 0xFF);
    for (uint16_t i = 0; i < page_buf_offset; i++) {
        flash_spi_rw(page_buffer[i]);
    }
    flash_cs_high();
    flash_wait_not_busy();
    log_write_address += page_buf_offset;
    page_buf_offset = 0;
}

void flash_log(uint32_t loop_cnt, const imu_t *imu, const pid_t *pid,
               const uint16_t motors[4], const bat_t *bat) {
    if (!logging_enabled) return;

    uint16_t next_idx = (ring_write_idx + 1) % LOG_RING_SIZE;
    if (next_idx == ring_read_idx) return;

    blackbox_frame_t *f = &log_ring[ring_write_idx];
    f->iteration = loop_cnt;
    for (int i = 0; i < 3; i++) {
        f->gyro[i]     = (int16_t)(imu->dps[i] * 16.0f);
        f->setpoint[i] = 0;
        f->pid_out[i]  = (int16_t)(pid->out[i] * 10.0f);
    }
    for (int i = 0; i < 4; i++) f->motors[i] = motors[i];
    f->vbat      = (uint16_t)(bat->v * 100.0f);
    f->cpu_load  = 0;
    f->flags     = 0;

    ring_write_idx = next_idx;
}

void flash_log_process(void) {
    while (ring_read_idx != ring_write_idx) {
        const blackbox_frame_t *f = &log_ring[ring_read_idx];

        if (page_buf_offset + sizeof(blackbox_frame_t) > FLASH_LOG_PAGE_SIZE) {
            flash_flush_page();
        }
        memcpy(&page_buffer[page_buf_offset], f, sizeof(blackbox_frame_t));
        page_buf_offset += sizeof(blackbox_frame_t);

        ring_read_idx = (ring_read_idx + 1) % LOG_RING_SIZE;
    }
}

void flash_log_start(void) {
    flash_erase_sector(0);
    log_write_address = 0;
    page_buf_offset = 0;
    ring_write_idx = ring_read_idx = 0;
    logging_enabled = true;
}

void flash_log_stop(void) {
    flash_log_process();
    flash_flush_page();
    logging_enabled = false;
}
