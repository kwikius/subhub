
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.


include ../../../../common/mk/stm32f0_make_flags.mk

src_objects = AP_GPS.o AP_GPS_UBLOX.o AP_GPS_GSOF.o AP_GPS_MTK.o \
AP_GPS_MTK19.o AP_GPS_NMEA.o AP_GPS_SBF.o AP_GPS_SBP.o \
AP_GPS_SIRF.o  GPS_Backend.o 

objects = $(patsubst %.o,obj/%.o,$(src_objects))

# project archive dir
OUTPUT_ARCHIVE_FILE = ../../../../lib/apm/gps.a

CFLAGS += -fmax-errors=1

.PHONY :  all test clean
 
all : test

test : $(OUTPUT_ARCHIVE_FILE)

$(OUTPUT_ARCHIVE_FILE) : $(objects)
	mkdir -p ../../../../lib/apm
	$(AR) rcs $@ $(objects)

$(objects) : obj/%.o : %.cpp
	mkdir -p obj
	$(CC) $(CFLAGS) $< -o $@

clean: 
	rm -f obj/*.o
	rm -f $(OUTPUT_ARCHIVE_FILE)




