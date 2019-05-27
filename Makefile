CROSS_COMPILE:=arm-none-eabi-
CC:=$(CROSS_COMPILE)gcc
LD:=$(CROSS_COMPILE)gcc
OBJS:=startup.o system.o
LSCRIPT:=target.ld
OBJCOPY:=$(CROSS_COMPILE)objcopy

CFLAGS:=-mcpu=cortex-m7 -mthumb -g -ggdb -Wall -Wno-main -Wstack-usage=200 -ffreestanding -Wno-unused -nostdlib 
ASFLAGS+=-mthumb -mlittle-endian -mthumb-interwork -ggdb -ffreestanding -mcpu=cortex-m7
LDFLAGS:=-T $(LSCRIPT) -Wl,-gc-sections -Wl,-Map=image.map -nostdlib

all: single-bank.bin dualbank.bin
	@echo "Type 'make single' or 'make dualbank' to change your target settings."


single: single-bank.bin FORCE
	@openocd -f single.cfg &
	@sleep 3
	@echo "shutdown" | nc localhost 4444

dualbank: dualbank.bin FORCE
	@openocd -f dualbank.cfg &
	@sleep 3
	@echo "shutdown" | nc localhost 4444 &>/dev/null

check:
	@openocd -f check.cfg 2>&1| grep Bank &
	@sleep 1
	@echo
	@echo
	@echo "shutdown" | nc localhost 4444 &>/dev/null


single-bank.bin: single-bank.elf
	$(OBJCOPY) -O binary $^ $@

dualbank.bin: dualbank.elf
	$(OBJCOPY) -O binary $^ $@

single-bank.elf: $(OBJS) $(LSCRIPT) main-single.o
	$(LD) $(LDFLAGS) $(OBJS) main-single.o -o $@


dualbank.elf: $(OBJS) $(LSCRIPT) main-dualbank.o
	$(LD) $(LDFLAGS) $(OBJS) main-dualbank.o -o $@

startup.o: startup.c

main.o: main.c

clean:
	rm -f *.o *.bin *.elf *.map

FORCE:
