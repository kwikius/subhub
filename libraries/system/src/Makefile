include ../../../common/mk/stm32f0_make_flags.mk

object_files = spbrk.o system_initf0.o system.o

objects = $(patsubst %.o,obj/%.o,$(object_files))

# project archive dir
OUTPUT_ARCHIVE_FILE = ../../../lib/system.a

MALLOC_SRC = $(QUAN_INCLUDE_PATH)/quan_matters/src/stm32/malloc_free.cpp

all : test

test : $(OUTPUT_ARCHIVE_FILE)

$(OUTPUT_ARCHIVE_FILE) : $(objects) obj/startup.o obj/malloc.o
	$(AR) rcs $@ $(objects) obj/startup.o obj/malloc.o

$(objects) : obj/%.o : %.cpp
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@

obj/startup.o: $(STARTUP)
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@ 

obj/malloc.o: $(MALLOC_SRC)
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@ 

clean: 
	rm -f obj/*.o
	rm -f $(OUTPUT_ARCHIVE_FILE)




