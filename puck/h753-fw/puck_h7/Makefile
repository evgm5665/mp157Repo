# puck_h7 - build with: make   (needs arm-none-eabi-gcc)
TARGET  = puck_h7
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size
MCU     = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
DEFS    = -DSTM32H753xx -DUSE_HAL_DRIVER
INC     = -Iinc -Idrivers/hal/Inc -Idrivers/cmsis
CFLAGS  = $(MCU) $(DEFS) $(INC) -Os -g -Wall -ffunction-sections -fdata-sections -MMD
LDFLAGS = $(MCU) -Tstm32h753zi_flash.ld -Wl,--gc-sections -Wl,-Map=build/$(TARGET).map --specs=nano.specs --specs=nosys.specs
SRC     = $(wildcard src/*.c) $(wildcard drivers/hal/Src/*.c)
OBJ     = $(patsubst %.c,build/%.o,$(SRC)) build/startup.o

all: build/$(TARGET).elf build/$(TARGET).bin
build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@
build/startup.o: src/startup_stm32h753xx.s
	@mkdir -p build
	$(CC) $(MCU) -c $< -o $@
build/$(TARGET).elf: $(OBJ)
	$(CC) $(OBJ) $(LDFLAGS) -o $@
	$(SIZE) $@
build/$(TARGET).bin: build/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@
clean:
	rm -rf build
