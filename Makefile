TARGET=lm4f_pov

GCCDIR=/home/knielsen/devel/study/stellaris-arm/install
BINDIR=$(GCCDIR)/bin
CC=$(BINDIR)/arm-none-eabi-gcc
LD=$(BINDIR)/arm-none-eabi-ld
OBJCOPY=$(BINDIR)/arm-none-eabi-objcopy
LM4FLASH=/home/knielsen/devel/study/stellaris-arm/lm4tools/lm4flash/lm4flash

STARTUP=startup_gcc
LINKSCRIPT=$(TARGET).ld

FP_LDFLAGS= -L$(GCCDIR)/arm-none-eabi/lib/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16 -lm -L$(GCCDIR)/lib/gcc/arm-none-eabi/4.6.2/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16 -lgcc -lc

ARCH_CFLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -DTARGET_IS_BLIZZARD_RA1
INC=-I/home/knielsen/devel/study/stellaris-arm/SW-EK-LM4F120XL-9453 -DPART_LM4F120H5QR
CFLAGS=-g -Os  -std=c99 -Wall -pedantic $(ARCH_CFLAGS) $(INC)
LDFLAGS=--entry ResetISR --gc-sections

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(TARGET).o $(STARTUP).o $(LINKSCRIPT)

$(TARGET).o: $(TARGET).c

$(STARTUP).o: $(STARTUP).c

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.elf: %.o
	$(LD) $(LDFLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP).o $< $(FP_LDFLAGS)

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	$(LM4FLASH) $(TARGET).bin

clean:
	rm -f $(TARGET).o $(TARGET).elf $(TARGET).bin $(STARTUP).o

tty:
	stty -F/dev/ttyACM0 raw -echo -hup cs8 -parenb -cstopb 115200

cat:
	cat /dev/ttyACM0

.PHONY: all clean flash tty cat
