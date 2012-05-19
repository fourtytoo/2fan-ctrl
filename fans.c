/* -*- C++ -*- */

/*  main.c --- main program

 Copyright (C) 2009 by Walter C. Pelissero

 Author: Walter C. Pelissero <walter@pelissero.de>
 Project: PC fan control

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2, or (at
your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; see the file COPYING.  If not, write to
the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.*/

#include <util/delay.h>
#include <avr/io.h>

// pins on port B
#define FAN1_PIN PB0		// pin 5
#define FAN2_PIN PB1		// pin 6
#define NTC_PIN PB2		// pin 7
#define POT1_PIN PB3		// pin 2
#define POT2_PIN PB4		// pin 3
// these should match the three above
#define NTC_ADC 1		// ADC1
#define POT1_ADC 3		// ADC3
#define POT2_ADC 2		// ADC2

//////////////////////////////////////////////////////////////////////

#define GRANULARITY 100		// in ms
#define MINIMUM_ON 50		// x * GRANULARITY
#define SPINUP 10		// x * GRANULARITY

#define MIN_TEMP 300

#define HIGH 1
#define LOW 0
#define FULL_SPEED 255
#define STOP 0
#define INPUT 1
#define OUTPUT 0

// most motors don't even start below half their rated voltage
#define MINIMUM_SPEED ((FULL_SPEED) * 7 / 10)



#define getPortBits(port,mask) (*(&(port) - 2) & (mask))
#define getPortBit(port,bit) getPortBits((port), _BV(mask))
#define setPortBits(port,mask,bits) ((port) = ((port) & ~(mask)) | ((bits) & (mask)))
#define setPortBit(port,bit,value) setPortBits((port), _BV(bit), (((value) == HIGH) ? _BV(bit) : 0))
#define setPortBitsDirection(port,mask,direction) setPortBits(*(&(port) - 1), (mask), (((direction) == OUTPUT) ? (mask) : 0))
#define setPortBitDirection(port,bit,direction) setPortBitsDirection((port), _BV(bit), (direction))
// just like setPortBit/getPortBit -wcp1/4/12.
#define setRegisterBit(reg, bit, on) ((reg) = ((reg) & ~_BV(bit)) | ((on) ? _BV(bit) : 0))
#define getRegisterBit(reg, bit) ((reg) & _BV(bit))

static void
delay_ms (int time)
{
  for (; time > 0; --time)
    _delay_ms(1);
}

#if 0				// not used yet -wcp28/3/12.
static void
delay_us (int time)
{
  for (; time > 0; --time)
    _delay_us(1);
}
#endif

static void
delay_s (int time)
{
  for (; time > 0; --time)
    _delay_ms(1000);
}



static void
setFANspeed (int fan, unsigned speed)
{
  // COM0A1 = Clear OC0A (PB0) on Compare Match when up-counting.
  // COM0B1 = Clear OC0B (PB1) on Compare Match when up-counting.
  if (fan == PB0)
    {
      OCR0A = speed;
      // disconnect the PWM if speed = 0
      setRegisterBit(TCCR0A, COM0A1, speed > 0);
    }
  else				// PB1
    {
      OCR0B = speed;
      // disconnect the PWM if speed = 0
      setRegisterBit(TCCR0A, COM0B1, speed > 0);
    }
}

static unsigned
getFANspeed (int fan)
{
  if (fan == PB0)
    return OCR0A;
  // PB1
  return OCR0B;
}

static void
test_fans ()
{
  setFANspeed(FAN1_PIN, FULL_SPEED);
  delay_s(2);
  setFANspeed(FAN1_PIN, STOP);
  delay_ms(50);
  setFANspeed(FAN2_PIN, FULL_SPEED);
  delay_s(2);
  setFANspeed(FAN2_PIN, STOP);
  delay_s(4);
  setFANspeed(FAN1_PIN, FULL_SPEED);
  setFANspeed(FAN2_PIN, FULL_SPEED);
  delay_s(2);
  setFANspeed(FAN1_PIN, STOP);
  setFANspeed(FAN2_PIN, STOP);
}

static unsigned
readAnalogPin (int adc)
{
  uint8_t low, high;

  ADMUX = (adc & 0x0F);

  delay_ms(1);
  // start the conversion
  setRegisterBit(ADCSRA, ADSC, 1);
  // ADSC is cleared when the conversion finishes
  while (getRegisterBit(ADCSRA, ADSC));

  // IMPORTANT: we have to read ADCL first; doing so both ADCL and
  // ADCH get locked until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  low = ADCL;
  high = ADCH;

  // combine the two bytes
  return (high << 8) | low;
}

// just link readAnalogPin, but with lower (8 bit) resolution
static unsigned
readAnalogPinLR (int adc)
{
  ADMUX =
    _BV(ADLAR) |
    (adc & 0x0F);

  delay_ms(1);
  // start the conversion
  setRegisterBit(ADCSRA, ADSC, 1);
  // ADSC is cleared when the conversion finishes
  while (getRegisterBit(ADCSRA, ADSC));

  return ADCH;
}

static unsigned
readPot (int adc)
{
  return readAnalogPinLR(adc);
}

static void
init_PWM ()
{  
  // set PB0 and PB1 to lowest PWM.  These are used in setFANspeed().
  OCR0A = OCR0B = 0;
  // WGM 0&1 = Fast PWM, TOP=0xFF
  TCCR0A =
    _BV(WGM00) |
    _BV(WGM01);
  // the following two bits of TCCR0A are set in setFANspeed():
  // COM0A1 = Clear OC0A (PB0) on Compare Match when up-counting.
  // COM0B1 = Clear OC0B (PB1) on Compare Match when up-counting.

  // stop timer
  TCCR0B = 0;
  // CS00 = run at clock frequency (no prescaling)
  TCCR0B =
    // _BV(WGM02) |
    _BV(CS00);
}

static void
init ()
{
  // set a2d prescale factor (clock speed) to 128
  setRegisterBit(ADCSRA, ADPS2, 1);
  setRegisterBit(ADCSRA, ADPS1, 1);
  setRegisterBit(ADCSRA, ADPS0, 1);
  // enable a2d conversions
  setRegisterBit(ADCSRA, ADEN, 1);

  setPortBitDirection(PORTB, FAN1_PIN, OUTPUT);
  setPortBitDirection(PORTB, FAN2_PIN, OUTPUT);
  init_PWM();
  setFANspeed(FAN1_PIN, STOP);
  setFANspeed(FAN2_PIN, STOP);
  setPortBitDirection(PORTB, NTC_PIN, INPUT);
  setPortBitDirection(PORTB, POT1_PIN, INPUT);
  setPortBitDirection(PORTB, POT2_PIN, INPUT);
}

class Fan
{
 public:
  Fan(int pin);
  void setSpeed (unsigned percent);

 private:
  int pin;
  unsigned spinup;
  unsigned stay_on;
};

Fan::Fan (int pin_number)
{
  pin = pin_number;
  spinup = stay_on = 0;
}


// here speed is in the range 0-100
void
Fan::setSpeed (unsigned percent)
{
  if (spinup)
    --spinup;
  else
    {
      unsigned speed;

      if (percent > 100)
	percent = 100;
      if (stay_on)
	--stay_on;
      if (percent == 0 && !stay_on)
	setFANspeed(pin, STOP);
      else
	{
	  if (getFANspeed(pin) == STOP)
	    {
	      spinup = SPINUP;
	      stay_on = MINIMUM_ON;
	      speed = FULL_SPEED;
	    }
	  else
	    speed = MINIMUM_SPEED + (percent * (FULL_SPEED - MINIMUM_SPEED) / 100);
	  setFANspeed(pin, speed);
	}
    }
}

static void
loop ()
{
  Fan fan1(FAN1_PIN);
  Fan fan2(FAN2_PIN);

  for (;;)
    {
      unsigned temp = readAnalogPin(NTC_ADC);
      unsigned min = readPot(POT1_ADC);
      unsigned range = readPot(POT2_ADC);
      unsigned fan1_speed, fan2_speed;

      temp -= MIN_TEMP;
      min >>= 1;
      range >>= 1;
      fan1_speed = (temp > min ? ((temp - min) * 100 / range) : 0);
      if (fan1_speed > 100)
	{
	  fan2_speed = fan1_speed - 100;
	  fan1_speed = 100;
	}
      else
	fan2_speed = 0;
      fan1.setSpeed(fan1_speed);
      fan2.setSpeed(fan2_speed);

      delay_ms(GRANULARITY);
    }
}

int
main ()
{
  init();
  delay_ms(100);
  test_fans();
  delay_s(5);
  loop();
  // shouldn't reach this point
  return 0;
}
