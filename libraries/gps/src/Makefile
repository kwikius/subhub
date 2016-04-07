include ../../../common/stm32f0_make_flags.mk

src_objects = gps.o

objects = $(patsubst %.o,obj/%.o,$(src_objects))

# project archive dir
OUTPUT_ARCHIVE_FILE = ../../../lib/gps.a
 
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




