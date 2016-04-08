
include common/stm32f0_make_flags.mk


object_files = main.o setup.o spbrk.o $(SYSTEM_INIT).o timer.o serial_port.o events.o
objects = $(patsubst %.o,obj/%.o,$(object_files))

lib_files = gps.a ap_math.a
libs = $(patsubst %.a,lib/%.a,$(lib_files))

all: test

test: bin/main.elf
	@ echo "...copying"
	$(CP) $(CPFLAGS) bin/main.elf bin/main.bin
	$(OD) $(ODFLAGS) bin/main.elf > bin/main.lst
	$(SIZ) -A bin/main.elf

bin/main.elf:  $(objects) obj/startup.o $(libs)
	mkdir -p bin
	@ echo "..linking"
	$(LD) $(LFLAGS) -o bin/main.elf $(objects) obj/startup.o $(libs)

$(objects): obj/%.o : %.cpp  
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@

obj/startup.o: $(STARTUP)
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@ 

$(libs) : lib/%.a : libraries/%/src
	make -C $<

.PHONY: clean all test upload
clean:
	-rm -rf obj/*.o bin/*.elf bin/*.bin bin/*.lst

upload : test
	st-flash write bin/main.bin 0x8000000

