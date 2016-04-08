include ../../../common/mk/stm32f0_make_flags.mk

object_files = spbrk.o system_initf0.o

objects = $(patsubst %.o,obj/%.o,$(object_files))

# project archive dir
OUTPUT_ARCHIVE_FILE = ../../../lib/system.a

all : test

test : $(OUTPUT_ARCHIVE_FILE)

$(OUTPUT_ARCHIVE_FILE) : $(objects) obj/startup.o
	$(AR) rcs $@ $(objects) obj/startup.o

$(objects) : obj/%.o : %.cpp
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@

obj/startup.o: $(STARTUP)
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@ 

clean: 
	rm -f obj/*.o
	rm -f $(OUTPUT_ARCHIVE_FILE)




