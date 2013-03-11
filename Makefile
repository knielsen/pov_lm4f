TARGET=lm4f_pov

BINDIR=/home/knielsen/devel/study/stellaris-arm/install/bin
CC=$(BINDIR)/arm-none-eabi-gcc
LD=$(BINDIR)/arm-none-eabi-ld
OBJCOPY=$(BINDIR)/arm-none-eabi-objcopy
LM4FLASH=/home/knielsen/devel/study/stellaris-arm/lm4tools/lm4flash/lm4flash

STARTUP=startup_gcc
LINKSCRIPT=$(TARGET).ld

ARCH_CFLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -ffunction-sections -fdata-sections -DTARGET_IS_BLIZZARD_RA1
INC=-I/home/knielsen/devel/study/stellaris-arm/SW-EK-LM4F120XL-9453 -DPART_LM4F120H5QR
CFLAGS=-g -Os  -std=c99 -Wall -pedantic $(ARCH_CFLAGS) $(INC)
LDFLAGS=--entry ResetISR --gc-sections

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(TARGET).bin $(STARTUP).o $(LINKSCRIPT)

$(TARGET).o: $(TARGET).c

$(STARTUP).o: $(STARTUP).c

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.elf: %.o
	$(LD) $(LDFLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP).o $<

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash:
	$(LM4FLASH) $(TARGET).bin

clean:
	rm -f $(TARGET).o $(TARGET).elf $(TARGET).bin $(STARTUP).o

.PHONY: all clean flash
