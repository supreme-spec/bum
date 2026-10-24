TARGET = radun_accif435

CC = arm-none-eabi-gcc
OBJCPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CPU = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
OPT = -O2 -flto
DEFS = -DAT32F435xx -DCLOCK_USE_PLL=0
INCS = -I. -Iradun_fix

CFLAGS = $(CPU) $(OPT) $(DEFS) $(INCS) -Wall -Wextra \
         -ffunction-sections -fdata-sections \
         -Wno-unused-parameter -Wno-unused-function

ASFLAGS = $(CPU)

LDFLAGS = $(CPU) -T AT32F435G.ld -Wl,--gc-sections -nostartfiles \
          -Wl,-Map=$(TARGET).map

SRCS_C = main_fixed.c hal_at32.c bmi270_real.c dshot600.c crsf.c pid.c modes.c \
         rd_filter_pred_fixed.c bmp280.c flash_log.c bmi270_config_file.c \
         radun_advanced.c scheduler.c watchdog.c failsafe.c blackbox_ring.c ubx.c \
         rpm_filter.c battery.c cli.c msp.c usb_cdc.c

SRCS_S = startup.s

OBJS = $(SRCS_C:.c=.o) $(SRCS_S:.s=.o)

all: $(TARGET).hex size

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ -lm

$(TARGET).hex: $(TARGET).elf
	$(OBJCPY) -O ihex $< $@

size: $(TARGET).elf
	$(SIZE) $<

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(ASFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).hex $(TARGET).map

.PHONY: all clean size
