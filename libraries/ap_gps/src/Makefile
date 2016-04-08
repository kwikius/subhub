include ../../../common/mk/stm32f0_make_flags.mk

src_objects = AP_GPS.o AP_GPS_UBLOX.o AP_GPS_GSOF.o AP_GPS_MTK.o \
AP_GPS_MTK19.o AP_GPS_NMEA.o AP_GPS_SBF.o AP_GPS_SBP.o \
AP_GPS_SIRF.o  GPS_Backend.o 

objects = $(patsubst %.o,obj/%.o,$(src_objects))

# project archive dir
OUTPUT_ARCHIVE_FILE = ../../../lib/ap_gps.a

CFLAGS += -fmax-errors=1

.PHONY :  all test clean
 
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




