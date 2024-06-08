# Flags
CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m4 -mthumb -nostdlib
CPPFLAGS=-DSTM32F446xx \
	 -Ivendor/CMSIS/Device/ST/STM32F4/Include \
	 -Ivendor/CMSIS/CMSIS/Core/Include \
	 -Wl,-Map=$(OUTPUT)/$(PROGRAM).map

LINKER_FILE=linker_script.ld
LDFLAGS=-T $(LINKER_FILE)

# Parameters
PROGRAM=blink
OUTPUT=build

# Building
all: $(PROGRAM).elf

$(PROGRAM).elf: $(OUTPUT)/main.o $(OUTPUT)/startup.o $(OUTPUT)/system_stm32f4xx.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o $(OUTPUT)/$(PROGRAM).elf

$(OUTPUT)/main.o: main.c
	$(CC) $(CFLAGS) $(CPPFLAGS) main.c -c -o $(OUTPUT)/main.o

$(OUTPUT)/startup.o: startup.c
	$(CC) $(CFLAGS) $(CPPFLAGS) startup.c -c -o $(OUTPUT)/startup.o

$(OUTPUT)/system_stm32f4xx.o: vendor/CMSIS/Device/ST/STM32F4/Source/Templates/system_stm32f4xx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) vendor/CMSIS/Device/ST/STM32F4/Source/Templates/system_stm32f4xx.c -c -o $(OUTPUT)/system_stm32f4xx.o

# Clean up
.PHONY: clean
clean:
	rm -f $(OUTPUT)/*

# Flashing
PROGRAMMER=openocd
PROGRAMMER_FLAGS=-f interface/stlink.cfg -f target/stm32f4x.cfg

flash: blink.elf
	$(PROGRAMMER) $(PROGRAMMER_FLAGS) -c "program $(OUTPUT)/$(PROGRAM).elf verify reset exit"
