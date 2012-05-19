#  -*- Makefile -*-

#  Copyright (C) 2009 by Walter C. Pelissero

#  Author: Walter C. Pelissero <walter@pelissero.de>
#  Project: fans control

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2, or (at
# your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

RM = rm -f
# The CPU
MCU = attiny45
# adjust this to the CPU frequency
F_CPU = 1000000
CFLAGS = -Wall -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) -gstabs
LDFLAGS = 
LDLIBS =
CC = avr-gcc -std=gnu99
CXX = avr-c++
OBJCOPY = avr-objcopy
AVRDUDE = avrdude
# use the parallel port programmer
PROGRAMMER = stk200
AVRDUDE_PORT = /dev/ppi0
SIZE = avr-size


default: all

all: fans.hex

fans.bin: fans.o
	$(CXX) $(CFLAGS) $(LDFLAGS) -o $@ fans.o $(LDLIBS)

clean:
	$(RM) *.o *.bin *.hex

dist: clean

upload: fans.hex
	$(AVRDUDE) -F -V -p $(MCU) -P $(AVRDUDE_PORT) -c $(PROGRAMMER) -b 19200 -U lfuse:w:0x62:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m -U flash:w:fans.hex 

######################################################################

.SUFFIXES: .bin .hex

.o.bin:
	$(CXX) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

.c.o:
	$(CXX) $(CFLAGS) -c $<

.c.bin:
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

.bin.hex:
	$(OBJCOPY) -R .eeprom -O ihex $< $@
	@$(SIZE) $@
