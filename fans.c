/* -*- C++ -*- */

/*  main.c --- main program

 Copyright (C) 2012-2014 by Walter C. Pelissero

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

// How often the main loop is run.  Every 1/10th of a second should be
// alright.  Remember that if you change this, the values that are
// proportional to it will have a different meaning in real terms.
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
//  22-25 degrees Celsius.
#ifndef MIN_TEMP
# define MIN_TEMP 300
#endif

// Fans take turns at which one starts first.
// This helps evenly spread wear on both fans.
#ifndef SWAP_FANS
# define SWAP_FANS 1
#endif

// A cumulative moving average is applied to the temperature readings,
// thus damping the value and causing less variations to the fans
// speed.  The bigger the window, the smoother (and slower) the
// transitions.  A value of 1 gets rid of the damping altogether.
#ifndef CMA_WINDOW
# define CMA_WINDOW 50
#endif

// The PWM duty cycle will tell the voltage applied to a fan and thus
// its speed.  Most motors don't even budge around half their rated
// voltage.  You should check your fans data sheet or do some
// experiments.  If you find yourself using too high values here, it
// may mean that the fans you are trying to employ don't like to have
// their speed modulated at all; choose another type.
#ifndef FAN1_MIN_DC
# define FAN1_MIN_DC 70		// 70% minimum duty cycle
#endif
#ifndef FAN2_MIN_DC
# define FAN2_MIN_DC 70		// 70% minimum duty cycle
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
#define INPUT 1
#define OUTPUT 0

#define setRegisterBits(reg,mask,bits) ((reg) = ((reg) & ~(mask)) | ((bits) & (mask)))
#define setPortBitsDirection(port,mask,direction) setRegisterBits(*(&(port) - 1), (mask), (((direction) == OUTPUT) ? (mask) : 0))
#define setPortBitDirection(port,bit,direction) setPortBitsDirection((port), _BV(bit), (direction))
#define setRegisterBit(reg, bit, on) setRegisterBits((reg), _BV(bit), ((on) ? _BV(bit) : 0))
#define getRegisterBit(reg, bit) ((reg) & _BV(bit))


static void
delay_ms (unsigned time)
{
  for (; time > 0; --time)
    _delay_ms(1);
}

inline static void
delay_100ms (unsigned time)
{
  for (; time > 0; --time)
    _delay_ms(100);
}

static void
delay_s (unsigned time)
{
  for (; time > 0; --time)
    delay_100ms(10);
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

static unsigned
readNTC ()
{
  return readAnalogPin(NTC_ADC);
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
  Fan(int pin, unsigned minimum_duty_cycle);
  void setSpeed (unsigned speed);
  unsigned getSpeed ();
  void updateSpeed (unsigned percent);

private:
  int pin;
  unsigned spinup;
  unsigned trail_on;
  unsigned minimum_speed;
};

Fan::Fan (int pin_number, unsigned minimum_duty_cycle)
{
  pin = pin_number;
  minimum_speed = FULL_SPEED * minimum_duty_cycle / 100;
  spinup = trail_on = 0;
}

void
Fan::setSpeed (unsigned speed)
{
  setFANspeed(pin, speed);
}

void
Fan::updateSpeed (unsigned percent)
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
	    setSpeed(STOP);
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
	    speed = minimum_speed + (percent * (FULL_SPEED - minimum_speed) / 100);
	  setSpeed(speed);
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
  unsigned old_temp = readNTC();
  Fan f1(FAN1_PIN, FAN1_MIN_DC);
  Fan f2(FAN2_PIN, FAN2_MIN_DC);
  Fan *fan1 = &f1, *fan2 = &f2;
#if SWAP_FANS == 1
  bool must_swap = false;
#endif // SWAP_FANS

  for (;;)
    {
      unsigned temp = readNTC();
      unsigned min = readPot(POT1_ADC);
      unsigned range = readPot(POT2_ADC);
      unsigned fan1_speed, fan2_speed;

      // Damp the temperature value averaging it with the old one.
      // The more weight on the old one the less dynamic the
      // variations.
      temp = (((CMA_WINDOW - 1) * old_temp) + temp) / CMA_WINDOW;
      old_temp = temp;
      // Reduce the temperature offset, because we are not interested
      // in temperatures below MIN_TEMP.
      temp = (temp > MIN_TEMP) ? (temp - MIN_TEMP) : 0;
      // Scale down min and range pot values because we don't need so
      // much value range.
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
      fan1->updateSpeed(fan1_speed);
      fan2->updateSpeed(fan2_speed);
#if SWAP_FANS == 1
      if (!must_swap && fan1_speed > 0)
	must_swap = true;
      if (must_swap && fan1->getSpeed() == 0 && fan2->getSpeed() == 0)
	{
	  Fan *t = fan1;
	  fan1 = fan2;
	  fan2 = t;
	  must_swap = false;
	}
#endif // SWAP_FANS
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
