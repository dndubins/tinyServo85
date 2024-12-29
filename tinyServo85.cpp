// tinyServo85.cpp file for tinyServo85.h library.
// Control up to 5 servos from pins PB0...PB4
// using CTC mode of Timer1 (ATtiny85)
// Clock speed = 16MHz
// Transferability: This library is only designed to work on the ATtiny85.
// Author: D.Dubins
// Date: 29-Dec-24
// Last Updated: 29-Dec-24
//
// The library maps specific servo numbers to the following pins:
// servo 0: PB0
// servo 1: PB1
// servo 2: PB2
// servo 3: PB3
// servo 4: PB4
#include "tinyServo85.h"
#include <avr/io.h>
#include <avr/interrupt.h>

unsigned int tinyServo85::servo_PWs[NSVO] = { 1500, 1500, 1500, 1500, 1500 };   // pulse-width of mid-points of each servo
bool tinyServo85::servo_attached[NSVO] = { false, false, false, false, false }; // each pin starts out unattached
volatile bool tinyServo85::pinStates[NSVO] = { 0, 0, 0, 0, 0 };                 // to store pin states
bool tinyServo85::timer1_enabled = false;     // to keep track if Timer1 is enabled
unsigned long tinyServo85::servo_tLast = 0UL; // To store the time the last servo was used (timeout function)
volatile int tinyServo85::cycles = 0;         // to hold #cycles ISR ran within one period of the servo signal
byte tinyServo85::PORTMASK = 0b00000000;                                      // to store attached servos (saves a few steps inside ISR)

tinyServo85::tinyServo85() {
}

void tinyServo85::attachServo(byte servo_num) { // function to attach servo
  if (servo_num < 5) {
    tinyServo85::servo_attached[servo_num] = true;   // Set servo_attached to true
    DDRB |= (1 << (PB0 + servo_num));  // Set servo pin in DDRB to OUTPUT mode
	tinyServo85::PORTMASK |= (1 << (PB0 + servo_num));  // Set servo pin to OUTPUT mode
  }
}

void tinyServo85::detachServo(byte servo_num) { // function to detach servo
  if (servo_num < 5) {
    tinyServo85::servo_attached[servo_num] = false;   // Set servo_attached to false
    PORTB &= ~(1 << (PB0 + servo_num)); // Set servo pin low
    DDRB &= ~(1 << (PB0 + servo_num));  // Set servo pin to INPUT mode (less chatter when not doing anything)
    tinyServo85::PORTMASK &= ~(1 << (PB0 + servo_num));  // Set servo pin to OUTPUT mode
 }
} 

void tinyServo85::setServo(byte servo_num, int angle) {
  if(!tinyServo85::timer1_enabled)tinyServo85::enableTimerInterrupt();     // enable Timer1 in case it timed out
  int pulse_width = map(angle, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // convert angle to pulse width in microseconds
  pulse_width = constrain(pulse_width, SVOMINPULSE, SVOMAXPULSE);          // constrain pulse width to min and max
  if (pulse_width != tinyServo85::servo_PWs[servo_num] && tinyServo85::servo_attached[servo_num]) { // Disable interrupts only if signal changes and servo is attached
    cli();                                                                 // Disable interrupts. It's best to update volatile global variables with interrupts diabled.
    tinyServo85::servo_PWs[servo_num] = pulse_width;                       // Store new pulse_width in servo_PWs
    sei();                                                                 // Enable interrupts. Spend as little time in "disabled interrupt land" as possible.
    tinyServo85::servo_tLast = millis();                                   // Record time servo was last used for timeout function
  }
}

void tinyServo85::homeServos() {  // function to home servos
  // Note: servo will not home unless it is first enabled.
  for (byte i = 0; i < NSVO; i++) {
    tinyServo85::setServo(i, 90);  // send all servos to middle position
  }
  delay(1000);  // wait for servos to home
}

void tinyServo85::setCTC() {  // function to set the registers of the ATtiny84 for Timer 1 CTC mode
   // CTC Match Routine using Timer 1 (ATtiny85)
  // Formula: frequency=fclk/((OCR1C+1)*N)
  cli();                      // clear interrupts
  GTCCR = _BV(PSR1);          // reset the Timer1 prescaler
  TIMSK |= _BV(OCIE1A);       // interrupt on Compare Match A  
  TCCR1 = 0;                  // clear out any previous bits in TCCR1
  TCCR1 |= _BV(CTC1);         // set CTC1 bit to clear timer/counter on compare match (CTC mode)
  TCCR1 |= _BV(CS13); // prescaler=128
  OCR1C = 249; // Set betw 1-255 (fclk=16MHz, prescaler=128, OCR1C=249 --> 500Hz)
  sei();       // enable interrupts
}

ISR(TIM1_COMPA_vect) {  // This is the ISR that will turn off the pins at the correct widths
  //The function micros() does not advance inside the ISR.
  //TCNT1 starts at 0 and counts up. Each increment lasts 8 microseconds. We are going to use this as a timer.
  //8 microseconds gives us (2500-500us)/8us =250 steps. This is fine for a 180 degree servo. For a 360 degree servo,
  //if more steps are needed, you can set the timer to N=8, OCR1A=19999 and this will give 2000 steps, but require
  //more attention by the ISR.
  if (tinyServo85::cycles == 0) {
    TCNT1 = 187;        // offset the counting routine so that the ISR starts at 504 microseconds
    PORTB |= tinyServo85::PORTMASK;  // set correct servo pin high
    for (byte i = 0; i < NSVO; i++) {
      tinyServo85::pinStates[i] = HIGH;
    }
  } else if (tinyServo85::cycles == 9) {
    TCNT1 = 62;  // last cycle length is adjusted to account for offset
  }

  if (tinyServo85::cycles == 1) {  // only do this routine over the pulse width range
    while (TCNT1 < 249) {
      for (byte i = 0; i < NSVO; i++) {
        if (tinyServo85::servo_attached[i] && tinyServo85::pinStates[i] && (tinyServo85::servo_PWs[i] <= (504 + ((unsigned int)TCNT1 * 8)))) {
          // Turn off the servo pin if the timer exceeds the pulse width
          PORTB &= ~(1 << (PB0 + i));  // Set correct servo pin low
          tinyServo85::pinStates[i] = LOW;
        }
      }
    }
	PORTB &= ~tinyServo85::PORTMASK;	// cheap insurance if a pin doesn't go low
  }  // end if

  if (tinyServo85::cycles < 9) { // we are only using 1/10th of the 50 Hz cycle to adjust pulse width.
    tinyServo85::cycles++;
  } else {
    tinyServo85::cycles = 0;
  }
}

void tinyServo85::enableTimerInterrupt() {
  TIMSK |= (1 << OCIE1A);   // enable Timer1
  tinyServo85::timer1_enabled = true; 
}

void tinyServo85::disableTimerInterrupt() {
  TIMSK &= ~(1 << OCIE1A);  // disable Timer1
  tinyServo85::timer1_enabled = false;
}

void tinyServo85::servo_timeout_check() {  // check to shut down Timer1 after SVOTIMEOUT msec have elapsed since last servo move
  if (((millis() - tinyServo85::servo_tLast) > SVOTIMEOUT) && tinyServo85::timer1_enabled) {
    tinyServo85::disableTimerInterrupt();  // disable Timer1
  }
}
