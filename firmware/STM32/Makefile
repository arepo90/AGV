TARGET   = firmware
BUILD    = build

CROSS    = arm-none-eabi-
CC       = $(CROSS)gcc
AS       = $(CROSS)gcc -x assembler-with-cpp
OBJCOPY  = $(CROSS)objcopy
SIZE     = $(CROSS)size

MCU      = -mcpu=cortex-m0 -mthumb
INCLUDES = -Icmsis/device -Icmsis/core -Iinc
DEFINES  = -DSTM32F051x8
OPT     ?= -Og
CFLAGS   = $(MCU) $(INCLUDES) $(DEFINES) -Wall -Wextra $(OPT) -g3 \
           -ffunction-sections -fdata-sections -std=c11
ASFLAGS  = $(MCU) -g3
LDSCRIPT = stm32f051x8.ld
LDFLAGS  = $(MCU) -T$(LDSCRIPT) -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map \
           -nostartfiles --specs=nano.specs --specs=nosys.specs -lm

# Glob every .c under src/ — modules can be added without editing the Makefile.
SRCS_C   = $(wildcard src/*.c)
SRCS_S   = startup_stm32f051x8.s

OBJS     = $(SRCS_C:%.c=$(BUILD)/%.o) $(SRCS_S:%.s=$(BUILD)/%.o)

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD)/%.o: %.c | $(BUILD)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD):
	mkdir -p $@

JLINK_DEVICE = STM32F051R8
JLINK_IF     = SWD
JLINK_SPEED  = 4000

flash: $(BUILD)/$(TARGET).bin
	@printf "r\nloadbin $< 0x08000000\nr\ng\nexit\n" > $(BUILD)/flash.jlink
	JLinkExe -device $(JLINK_DEVICE) -if $(JLINK_IF) -speed $(JLINK_SPEED) \
	         -autoconnect 1 -nogui 1 -commanderscript $(BUILD)/flash.jlink

erase:
	@printf "r\nerase\nexit\n" > $(BUILD)/erase.jlink
	JLinkExe -device $(JLINK_DEVICE) -if $(JLINK_IF) -speed $(JLINK_SPEED) \
	         -autoconnect 1 -nogui 1 -commanderscript $(BUILD)/erase.jlink

gdbserver:
	JLinkGDBServer -device $(JLINK_DEVICE) -if $(JLINK_IF) -speed $(JLINK_SPEED) -nogui

clean:
	rm -rf $(BUILD)

.PHONY: all flash erase gdbserver clean
