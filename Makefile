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

ARCH_CFLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -DTARGET_IS_BLIZZARD_RA2 -fno-math-errno
INC=-I/home/knielsen/devel/study/stellaris-arm/SW-EK-LM4F120XL-9453 -DPART_LM4F120H5QR
CFLAGS=-g -O3  -std=c99 -Wall $(ARCH_CFLAGS) $(INC)
LDFLAGS=--entry ResetISR --gc-sections

OBJS = $(TARGET).o gfx.o pov_config.o serial_dbg.o sdcard.o ev_fat.o

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(OBJS) $(STARTUP).o $(LINKSCRIPT)
	$(LD) $(LDFLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP).o $(OBJS) $(FP_LDFLAGS)

$(TARGET).o: $(TARGET).c gfx.h serial_dbg.h pov_config.h nrf24l01p.h sdcard.h
gfx.o: gfx.c gfx.h tlc_lookup.h
pov_config.o: pov_config.c pov_config.h gfx.h
serial_dbg.o: serial_dbg.h
sdcard.o: pov.h serial_dbg.h ev_fat.h
ev_fat.c: ev_fat.h

$(STARTUP).o: $(STARTUP).c

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

tlc_lookup.h: mk_tlc_lookup.pl
	perl mk_tlc_lookup.pl > tlc_lookup.h

flash: $(TARGET).bin
	$(LM4FLASH) $(TARGET).bin

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(STARTUP).o

tty:
	stty -F/dev/ttyACM0 raw -echo -hup cs8 -parenb -cstopb 500000

cat:
	cat /dev/ttyACM0

.PHONY: all clean flash tty cat
