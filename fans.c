/* -*- C++ -*- */

/*  main.c --- main program

 Copyright (C) 2012 by Walter C. Pelissero

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


// The PWM duty cycle will tell the voltage applied to a fan and thus
// its speed.  Most motors don't even budge below half their rated
// voltage.  Here we use a conservative 70%, but you should check your
// fans data sheet or do some experiments.  If you find yourself using
// too high values here, it may mean that the fans you are trying to
// employ don't like to have their speed modulated at all; choose
// another type.
#ifndef MINIMUM_DUTY_CYCLE
# define MINIMUM_DUTY_CYCLE	7/10
#endif

// How often the main loop is run.  Every 0.1s should be alright.
#ifndef GRANULARITY
# define GRANULARITY 100		// in ms
#endif

// How long the fan should be kept running before actually turning
// completely off.  This off-delay will resemble a sort of hysteresis
// to the naked eye.  The purpose is to avoid repeated on-off-on
// transitions when the temperature is floating around the trigger
// value.  This value is proportional to the GRANULARITY (above).
#ifndef TRAIL_ON
# define TRAIL_ON 50		// x * GRANULARITY
#endif

// How long to run the fan at full voltage on start, so that it will
// gain momentum.  This is proportional to the GRANULARITY (above).
#ifndef SPINUP
# define SPINUP 15		// x * GRANULARITY
#endif

//  You don't want to start the fans below this temperature.  This is
//  actually a voltage measured on NTC_PIN.  The value is between 0
//  and 1023, the latter being Vcc.  So, if Vcc is around 5V, here 300
//  means something about 1.46V.  This is what you would read at about
//  22-25 degrees centigrade.
#ifndef MIN_TEMP
# define MIN_TEMP 300
#endif

// Temperature variations are damped by this factor. The bigger the
// slower the fan speed variations.  A value of 1 gets rid of the
// damping.
#ifndef DAMP_FACTOR
# define DAMP_FACTOR 10
#endif

// How quickly the fan speed should ramp up with the increase of
// temperature. 100 means 1:1 (linearly).  That is, the fan speed is
// proportional to temperature.  Anything below 100 doesn't make much
// sense.  Values above 100 will try to compensate the temperature
// more aggressively.
#ifndef SPEED_GAIN
# define SPEED_GAIN 100
#endif

// end of tuning knobs
//////////////////////////////////////////////////////////////////////


// You may want to change the following only if you change the
// hardware schematics.  In most cases you should leave them alone.

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


#define HIGH 1
#define LOW 0
#define FULL_SPEED 255
#define STOP 0
#define MINIMUM_SPEED ((FULL_SPEED) * MINIMUM_DUTY_CYCLE)
#define INPUT 1
#define OUTPUT 0

#define setRegisterBits(reg,mask,bits) ((reg) = ((reg) & ~(mask)) | ((bits) & (mask)))
#define setPortBitsDirection(port,mask,direction) setRegisterBits(*(&(port) - 1), (mask), (((direction) == OUTPUT) ? (mask) : 0))
#define setPortBitDirection(port,bit,direction) setPortBitsDirection((port), _BV(bit), (direction))
#define setRegisterBit(reg, bit, on) setRegisterBits((reg), _BV(bit), ((on) ? _BV(bit) : 0))
#define getRegisterBit(reg, bit) ((reg) & _BV(bit))


static void
delay_ms (int time)
{
  for (; time > 0; --time)
    _delay_ms(1);
}

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
  delay_s(3);
  setFANspeed(FAN1_PIN, STOP);
  delay_ms(50);
  setFANspeed(FAN2_PIN, FULL_SPEED);
  delay_s(3);
  setFANspeed(FAN2_PIN, STOP);
  delay_s(5);
  setFANspeed(FAN1_PIN, FULL_SPEED);
  setFANspeed(FAN2_PIN, FULL_SPEED);
  delay_s(3);
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

// just like readAnalogPin(), but with lower (8 bit) resolution
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
  unsigned getSpeed ();

 private:
  int pin;
  unsigned spinup;
  unsigned trail_on;
};

Fan::Fan (int pin_number)
{
  pin = pin_number;
  spinup = trail_on = 0;
  setSpeed(0);
}

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
      if (percent == 0)
	{
	  if (trail_on)
	    --trail_on;
	  else
	    setFANspeed(pin, STOP);
	}
      else
	{
	  unsigned current_speed = getFANspeed(pin);

	  trail_on = TRAIL_ON;
	  if (current_speed == STOP)
	    {
	      spinup = SPINUP;
	      speed = FULL_SPEED;
	    }
	  else
	    speed = MINIMUM_SPEED + (percent * (FULL_SPEED - MINIMUM_SPEED) / 100);
	  setFANspeed(pin, speed);
	}
    }
}

unsigned
Fan::getSpeed ()
{
  return getFANspeed(pin);
}

static void
loop ()
{
  unsigned old_temp = readAnalogPin(NTC_ADC);
  Fan fan1(FAN1_PIN);
  Fan fan2(FAN2_PIN);

  for (;;)
    {
      unsigned temp = readAnalogPin(NTC_ADC);
      unsigned min = readPot(POT1_ADC);
      unsigned range = readPot(POT2_ADC);
      unsigned fan1_speed, fan2_speed;

      // Damp the temperature value averaging it with the old one.
      // The more weight on the old one the less dynamic the
      // variations.
      temp = (((DAMP_FACTOR - 1) * old_temp) + temp) / DAMP_FACTOR;
      old_temp = temp;
      // Reduce the temperature offset, because we are not interested
      // in temperatures below MIN_TEMP.
      temp -= MIN_TEMP;
      // Scale down min and range pot values because we don't need so
      // much value range.
      min >>= 1;
      range >>= 1;
      fan1_speed = (temp > min ? ((temp - min) * SPEED_GAIN / range) : 0);
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
