include ../../../common/mk/stm32f0_make_flags.mk

object_files = ap_math.o edc.o

objects = $(patsubst %.o,obj/%.o,$(object_files))

# project archive dir
OUTPUT_ARCHIVE_FILE = ../../../lib/ap_math.a

all : test

test : $(OUTPUT_ARCHIVE_FILE)

$(OUTPUT_ARCHIVE_FILE) : $(objects)
	$(AR) rcs $@ $(objects)

$(objects) : obj/%.o : %.cpp
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@

clean: 
	rm -f obj/*.o
	rm -f $(OUTPUT_ARCHIVE_FILE)




