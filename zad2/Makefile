CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
FLAGS = -mthumb -mcpu=cortex-m4
CPPFLAGS = -DSTM32F411xE
CFLAGS = $(FLAGS) -Wall -Wextra -g \
-O2 -ffunction-sections -fdata-sections \
-I/home/mikolaj/arm/stm32/inc \
-I/home/mikolaj/arm/stm32/CMSIS/Include \
-I/usr/include \
-I/home/mikolaj/arm/stm32/CMSIS/Device/ST/STM32F4xx/Include 
LDFLAGS = $(FLAGS) -Wl,--gc-sections -nostartfiles \
-L/home/mikolaj/arm/stm32/lds -Tstm32f411re.lds
vpath %.c /home/mikolaj/arm/stm32/src

OBJECTS = leds_main.o startup_stm32.o delay.o gpio.o buttons.o
TARGET = leds
.SECONDARY: $(TARGET).elf $(OBJECTS)
all: $(TARGET).bin
%.elf : $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@

%.bin : %.elf
	$(OBJCOPY) $< $@ -O binary

clean :
	rm -f *.bin *.elf *.hex *.d *.o *.bak *~
