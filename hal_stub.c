/* hal_stub.c — заглушки HAL для host-сборки */
#include "radun.h"
#include <stdlib.h>
#include <time.h>

uint32_t hal_ms(void) { return (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC); }
void delay_ms(uint32_t ms) { (void)ms; }
void delay_us(uint32_t us) { (void)us; }
void hal_init(void) {}
void gpio_out(GPIO_t *g, uint16_t p) { (void)g; (void)p; }
void gpio_af(GPIO_t *g, uint16_t p, uint8_t a) { (void)g; (void)p; (void)a; }
void gpio_set(GPIO_t *g, uint16_t p) { (void)g; (void)p; }
void gpio_clr(GPIO_t *g, uint16_t p) { (void)g; (void)p; }
uint8_t spi_rw(SPI_t *s, uint8_t v) { (void)s; return v; }
void spi_init(SPI_t *s) { (void)s; }
void i2c_write(uint8_t a, uint8_t r, uint8_t v) { (void)a; (void)r; (void)v; }
uint8_t i2c_read(uint8_t a, uint8_t r) { (void)a; (void)r; return 0; }
void uart_init(UART_t *u, uint32_t b) { (void)u; (void)b; }
void NVIC_SetPriority(uint8_t n, uint32_t p) { (void)n; (void)p; }