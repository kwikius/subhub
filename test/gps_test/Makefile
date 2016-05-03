
include ../../common/mk/stm32f0_make_flags.mk

object_files = main.o setup.o gps_sp.o led.o passthrough.o
objects = $(patsubst %.o,obj/%.o,$(object_files))

lib_files = apm/gps.a apm/math.a system.a
libs = $(patsubst %.a,../../lib/%.a,$(lib_files))

all: test


test: bin/main.elf
	@ echo "...copying"
	$(CP) $(CPFLAGS) bin/main.elf bin/main.bin
	$(OD) $(ODFLAGS) bin/main.elf > bin/main.lst
	$(SIZ) -A bin/main.elf

bin/main.elf:  $(objects) $(libs)
	mkdir -p bin
	@ echo "..linking"
	$(LD) $(LFLAGS) -o bin/main.elf $(objects) $(libs)

$(objects): obj/%.o : %.cpp  
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@

obj/startup.o: $(STARTUP)
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@ 

$(libs) : ../../lib/%.a : ../../libraries/%/src
	make -C $<

.PHONY: clean cleanlibs all test upload
clean:
	-rm -rf obj/*.o bin/*.elf bin/*.bin bin/*.lst

cleanlibs:
	for libdir in $(patsubst %.a,%,$(lib_files)) ; do \
      make -C ../../libraries/$$libdir/src clean; \
   done

clean_gps:
	make -C ../../libraries/apm/gps/src clean

upload : test
	/home/andy/bin/stm32flash -b 115200 -f -v -w bin/main.bin /dev/ttyUSB0

