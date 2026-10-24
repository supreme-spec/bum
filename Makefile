TARGET = radun_accif435
CC     = arm-none-eabi-gcc
OBJCPY = arm-none-eabi-objcopy
SIZE   = arm-none-eabi-size

CPU    = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
OPT    = -O2 -flto -ffast-math
DEFS   = -DAT32F435xx
INCS   = -I.

CFLAGS = $(CPU) $(OPT) $(DEFS) $(INCS) -Wall -Wno-unused-function
ASFLAGS = $(CPU)
LDFLAGS = $(CPU) -T AT32F435G.ld -Wl,--gc-sections -nostartfiles

# Исправленные исходные файлы
SRCS_C = main_fixed.c hal.c bmi270_fixed.c dshot_real.c crsf.c pid.c modes.c \
         rd_filter_pred_fixed.c bmp280.c flash_log.c bmi270_config_file.c \
         radun_advanced.c radun_rls_tuning.c attitude.c
SRCS_S = startup.s

OBJS = $(SRCS_C:.c=.o) $(SRCS_S:.s=.o)

all: $(TARGET).hex size

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ -lm
	$(SIZE) $@
$(TARGET).hex: $(TARGET).elf
	$(OBJCPY) -O ihex $< $@
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o: %.s
	$(CC) $(ASFLAGS) -c $< -o $@
size: $(TARGET).elf
	$(SIZE) $(TARGET).elf
clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).hex
.PHONY: all clean size