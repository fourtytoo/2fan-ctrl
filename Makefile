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

# The CPU
MCU = attiny45

# Adjust this to the CPU frequency.  The internal clock of the
# ATtiny45 runs from an internal 8MHz scillator, which is devided by 8
# if fuse bit CKDIV8 is set to 0 (which is what is done below with
# lfuse=0x62).  So the actual speed of the CPU is 1MHz.
F_CPU = 1000000

# -Os for size optimisation
CFLAGS = -Wall -Os -DF_CPU=$(F_CPU)UL
LDFLAGS = 
LDLIBS =
CC = avr-gcc -std=gnu99 -mmcu=$(MCU)
CXX = avr-c++ -mmcu=$(MCU)
OBJCOPY = avr-objcopy
# You can add flags  -F -V to rescue a badly mangled chip.  See avrdude(1).
AVRDUDE = avrdude
# for the parallel port programmer
#PROGRAMMER = stk200
AVRDUDE_PORT = /dev/ppi0
# for the USBasp programmer
PROGRAMMER = usbasp
AVRDUDE_PORT != usbconfig | awk -F : '/USBasp/ { print $$1 }'
SIZE = avr-size
RM = rm -f

# see  http://www.engbedded.com/fusecalc/
FUSES = -U lfuse:w:0x62:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

default: all

all: sketch.hex

clean:
	$(RM) *.o *.bin *.hex

dist: clean

fuses:
	$(AVRDUDE) -p $(MCU) -P $(AVRDUDE_PORT) -c $(PROGRAMMER) $(FUSES)

upload: sketch.hex
	$(AVRDUDE) -p $(MCU) -P $(AVRDUDE_PORT) -c $(PROGRAMMER) -U flash:w:sketch.hex 

######################################################################

.SUFFIXES: .bin .hex

.o.bin:
	$(CXX) $(LDFLAGS) -o $@ $< $(LDLIBS)

.c.o:
	$(CXX) $(CFLAGS) -c $<

.bin.hex:
	$(OBJCOPY) -R .eeprom -O ihex $< $@
	@$(SIZE) $@
