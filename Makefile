
include common/stm32f0_make_flags.mk

all: test

objects = main.o setup.o spbrk.o $(SYSTEM_INIT).o timer.o serial_port.o events.o

libs_out = lib/gps.a

clean:
	-rm -rf *.o *.elf *.bin *.lst

test: main.elf
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.elf main.bin
	$(OD) $(ODFLAGS) main.elf > main.lst

main.elf: startup.o $(objects) $(libs_out)
	@ echo "..linking"
	$(LD) $(LFLAGS) -o main.elf $(objects) startup.o $(libs_out)

$(objects): %.o : %.cpp  
	$(CC) $(CFLAGS) $< -o $@

startup.o: $(STARTUP)
	$(CC) $(CFLAGS) -o startup.o $(STARTUP) 

upload : test
	st-flash write main.bin 0x8000000

$(libs_out) : lib/%.a : libraries/%/src
	make -C $<
