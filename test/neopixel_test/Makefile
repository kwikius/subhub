
include ../../common/mk/stm32f0_make_flags.mk

local_object_files = main.o setup.o light_show_examples.o

local_objects = $(patsubst %.o,obj/%.o,$(local_object_files))

library_object_files = neopixel.o usart.o systick.o

library_objects = $(patsubst %.o,obj/%.o,$(library_object_files))

objects = $(local_objects) $(library_objects)

lib_files =  system.a 
libs = $(patsubst %.a,../../lib/%.a,$(lib_files))

EXTRA_DEFINES = -DNEOPIXEL_USE_TIM2

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

$(local_objects): obj/%.o : %.cpp  
	mkdir -p obj
	$(CC) $(CFLAGS) $(EXTRA_DEFINES) $< -o $@

obj/neopixel.o  : ../../libraries/neopixel/neopixel.cpp  
	$(CC) $(CFLAGS) $(EXTRA_DEFINES) $< -o $@

obj/usart.o  : ../../libraries/usart/usart.cpp  
	$(CC) $(CFLAGS) $< -o $@

obj/systick.o  : ../../libraries/systick/systick.cpp  
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

upload_st : test
	/home/andy/bin/stm32flash -b 115200 -f -v -w bin/main.bin /dev/ttyUSB0

upload: test
	st-flash write bin/main.bin 0x8000000
