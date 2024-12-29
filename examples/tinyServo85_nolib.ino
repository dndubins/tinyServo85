// tinyServo85_nolib.ino
// Control up to 5 servos from pins PB0...PB4
// using CTC mode of Timer1 (ATtiny85)
// Clock speed = 16MHz
// Transferability: This is a very specific sketch! Will only work on the ATtiny85.
// However, if you have a good understanding of timers and CTC mode, you can adapt it to
// non-PWM pins of other MCUs as well.
// Author: D.Dubins
// Date: 29-Dec-24
// Last Updated: 29-Dec-24
//
// Notes: I have a potentiometer wired as a voltage divider on pin PB2. This would conflict with the sketch if 8 servos
// are attached.
//
//The following are the ATtiny85 pins by function:
//------------------------------------------------
//Pin 1: PB5 / Reset (momentary switch btw Pin1 and GND to reset)
//Pin 2: PB3 / Analog Input 3 (A3) / Digital Pin 3 / PCINT3
//Pin 3: PB4 / Analog Input 2 (A2) / Digital Pin 4 / PCINT4
//Pin 4: GND
//Pin 5: PB0 / Digital Pin 0 / PWM supported / AREF / MOSI / PCINT0
//Pin 6: PB1 / Digital Pin 1 / PWM supported / MISO / PCINT1
//Pin 7: PB2 / Analog Input 1 (A1) / SCK / Digital Pin 2 / PCINT2
//Pin 8: Vcc+

//#define SERIALDEBUG  // comment out for no serial debugging. 
//A SoftwareSerial connection will conflict with pins PB3 and PB4 if servos are attached.

#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef SERIALDEBUG
#include <SoftwareSerial.h>
SoftwareSerial mySerial(PB3, PB4);  // Start the software serial monitor on PB3(RX) and PB4(TX)
#endif

#define NSVO 5            // number of servos to control (up to 5)
#define SVOMAXANGLE 179   // maximum angle for servo.
// Note: The cycle of Timer1 has been adjusted to fit 249 divisions within 50-2000ms. Changing SVOMINPULSE 
// and SVOMAXPULSE outside the range 504-2485 will adversely impact the functionality of the timing routine.
#define SVOMINPULSE 504   // minimum pulse width in microseconds for servo signal (0 degrees). Default: 504
#define SVOMAXPULSE 2485  // maximum pulse width in microseconds for servo signal (for maximum angle). Default: 2485
#define SVOTIMEOUT 500    // timeout in ms to disable servos. Should be long enough to attain setpoint.
#define SVO1 PB0          // starting pin in bank for servo 0. PB0 for first pin in BANKB. (default:PB0).

//This sketch allows you to make servo0 what you want.
//Make sure you declare your servo pins sequentially.
//If SVO1 is PB0, and NSVO is 3, then PB0 will be servo0, PB1 will be servo1, and PB2 will be servo2.
//The servos are allocated in order, so make sure you don't go beyond PB4.

//change the dimensions of the following arrays to match NSVO
unsigned int servo_PWs[NSVO] = { 1500, 1500, 1500, 1500, 1500 }; // Pulse widths in microseconds (default to center position)
bool servo_attached[NSVO] = { 0, 0, 0, 0, 0 };                   // Servo attachment status
bool timer1_enabled = false;                                     // to keep track if Timer1 is enabled
unsigned long servo_tLast = 0UL;                                 // To store the time the last servo was used (timeout function)
volatile bool pinStates[NSVO] = { 0, 0, 0, 0, 0 };               // to store pin states
byte PORTMASK = 0b00000000;                                      // to store attached servos (saves a few steps inside ISR)
volatile int cycles = 0;                                         // to hold #cycles ISR ran within one period of the servo signal

void setup() {
#ifdef SERIALDEBUG
  mySerial.begin(9600);  // Start the software serial monitor (for debugging)
#endif
  setCTC();  // set CTC mode to start a 50Hz timer for the servo signals
  // attach servos here
  attachServo(0);  // pin PB0 is servo0
  attachServo(1);  // pin PB1 is servo1
  attachServo(2);  // pin PB2 is servo2
  attachServo(3);  // pin PB3 is servo3
  attachServo(4);  // pin PB4 is servo4

  // if you would like to detach a specific servo at any time:
  //detachServo(0); // detach servo0
  //detachServo(1); // detach servo1
  //detachServo(2); // detach servo2
  //detachServo(3); // etc ...
#ifdef SERIALDEBUG
  mySerial.println("Starting serial monitor.");
#endif
  //homeServos();  // start sketch by sending attached servos to home position
}

void loop() { 
  // Uncomment the timeout check below for disabling Timer1, if the servos don't receive a command after
  // SVOTIMEOUT msec. This servo_timeout_check() is optional. Temporarily turning off Timer1 will free
  // the mcu to do other things. You can also manually suspend Timer1 with the command "disableTimerInterrupt();".
  //servo_timeout_check();  // if servos are inactive, stop Timer1 (less trouble for other routines)

  // Uncomment to rock all servos simultaneously, at full speed.
  for (int i = 0; i < NSVO; i++) {
    setServo(i, 0);  //0
  }
  delay(1000);

  for (int i = 0; i < NSVO; i++) {
    setServo(i, SVOMAXANGLE);  // SVOMAXANGLE
  }
  delay(1000);

  // Uncomment to rock the first 3 servos smoothly using moveTo(), with a 10ms delay between steps:
  //moveTo(0, 0, 0, 10);
  //moveTo(SVOMAXANGLE, SVOMAXANGLE, SVOMAXANGLE, 10);

  // Uncomment to rock servo0 very slowly
  /*for (int i = 0; i < SVOMAXANGLE; i++) {
    setServo(0, i);
    delay(100); // 0.5s delay should let you see each angle
  }
  for (int i = SVOMAXANGLE; i >=0; i--) {
    setServo(0, i);
    delay(100);
  }*/

  // Uncomment to rock all servos at full speed through 0-SVOMAXANGLE, sequentially.
  /*for (int i = 0; i < NSVO; i++) {
    setServo(i, 0);
    delay(1000);
    setServo(i, SVOMAXANGLE);
    delay(1000);
  }*/

  // Uncomment for quick potentiometer control:
  /*int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading on pin A3 & remap to angle.
  setServo(0, location);  // write new location to servo0
  setServo(1, location);  // write new location to servo1
  setServo(2, location);  // write new location to servo2
  delay(50);              // wait a bit to reduce jittering
  */
  // Uncomment for potentiometer control of all servos with slower movement:
  //int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading on pin A3 & remap to angle.
  //moveTo(location, location, location, 5);  // move to new location, delay=4 ms between steps

  // Uncomment for potentiometer control, disabling the timer between similar readings within a tolerance:
  /*#define TOL 1 // Tolerance to ignore readings. A greater than 1 degree difference will be sent to the servo.
  static int lastlocation;
  int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A3 & remap to angle.
  if(abs(location-lastlocation)>TOL){  // if readings are more than 1 degreee away
    setServo(0, location);  // write new location to servo0
    setServo(1, location);  // write new location to servo1
    setServo(2, location);  // write new location to servo2
  }else{
    disableTimerInterrupt(); // disable timers while waiting for a different reading
  }
  lastlocation=location;
  delay(50);              // wait a bit to reduce jittering
  */
}

void attachServo(byte servo_num) {  // function to attach servo
  if (servo_num < NSVO) {
    servo_attached[servo_num] = true;       // Set servo_attached to true
    DDRB |= (1 << (SVO1 + servo_num));      // Set servo pin to OUTPUT mode
    PORTMASK |= (1 << (SVO1 + servo_num));  // Set servo pin to OUTPUT mode
  }
}

void detachServo(byte servo_num) {  // function to detach servo
  if (servo_num < NSVO) {
    servo_attached[servo_num] = false;       // Set servo_attached to false
    PORTB &= ~(1 << (SVO1 + servo_num));     // Set servo pin low
    DDRB &= ~(1 << (SVO1 + servo_num));      // Set servo pin to INPUT mode (less chatter when not doing anything)
    PORTMASK &= ~(1 << (SVO1 + servo_num));  // Set servo pin to OUTPUT mode
  }
}

void setServo(byte servo_num, int angle) {                                 // function to set servo position
  if (!timer1_enabled) enableTimerInterrupt();                             // enable Timer1 in case it timed out
  int pulse_width = map(angle, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // convert angle to pulse width in microseconds
  pulse_width = constrain(pulse_width, SVOMINPULSE, SVOMAXPULSE);          // constrain pulse width to min and max
  if (pulse_width != servo_PWs[servo_num] && servo_attached[servo_num]) {  // Disable interrupts only if signal changes and servo is attached
    cli();                                                                 // Disable interrupts. It's best to update volatile global variables with interrupts diabled.
    servo_PWs[servo_num] = pulse_width;                                    // Store new pulse_width in servo_PWs
    sei();                                                                 // Enable interrupts. Spend as little time in "disabled interrupt land" as possible.
  }
  servo_tLast = millis();  // record time servo was last used for timeout function
}

void homeServos() {  // function to home servos
  // Note: servo will not home unless it is first enabled.
#ifdef SERIALDEBUG
  mySerial.println("Homing servos.");
#endif
  for (byte i = 0; i < NSVO; i++) {
    setServo(i, 0);  // send all servos to middle position
  }
  delay(1000);  // wait for servos to home
}

void moveTo(int s0, int s1, int s2, int wait) {  // function for controlling all servos slowly, simultaneously.
  // wait=0: as fast as possible.
  // Note: Change structure of moveTo arguments to match # servos (add coordinates s3, s4 ... as needed).
  int loc[NSVO] = { s0, s1, s2 };  // create array for loc’ns
  static int pos[NSVO];            // remembers last value of pos
  if (wait == 0) {                 // if wait=0, move as fast as possible
    for (int i = 0; i < NSVO; i++) {
      setServo(i, loc[i]);  // write new position to servos
    }
  } else {
    int dev = 0;  // to track deviation
    do {
      dev = 0;
      for (int i = 0; i < NSVO; i++) {  // moves servos one step
        if (loc[i] > pos[i]) pos[i]++;  // add 1 to pos[i]
        if (loc[i] < pos[i]) pos[i]--;  // subtr 1 from pos[i]
        dev += abs(pos[i] - loc[i]);    // calculate deviation
      }
      for (int i = 0; i < NSVO; i++) {
        setServo(i, pos[i]);  // write new position to servos
      }
      delay(wait);      // slow down movement
    } while (dev > 0);  // stop when location attained
  }                     // end if
}

void setCTC() {  // function to set the registers of the ATtiny85 for Timer 1 CTC mode
  // CTC Match Routine using Timer 1 (ATtiny85)
  // Formula: frequency=fclk/((OCR1C+1)*N)
  cli();                      // clear interrupts
  GTCCR = _BV(PSR1);          // reset the Timer1 prescaler
  TIMSK |= _BV(OCIE1A);       // interrupt on Compare Match A  
  TCCR1 = 0;                  // clear out any previous bits in TCCR1
  TCCR1 |= _BV(CTC1);         // set CTC1 bit to clear timer/counter on compare match (CTC mode)
  //Now, set the prescalers to what you would like.
  //TCCR1 |= _BV(CS10);       // prescaler=1
  //TCCR1 |= _BV(CS11);       // prescaler=2
  //TCCR1 |= _BV(CS11) |  _BV(CS10); // prescaler=4
  //TCCR1 |= _BV(CS12); // prescaler=16
  //TCCR1 |= _BV(CS12) |  _BV(CS11); // prescaler=32
  //TCCR1 |= _BV(CS12) |  _BV(CS11) |  _BV(CS10); // prescaler=64
  TCCR1 |= _BV(CS13); // prescaler=128
  //TCCR1 |= _BV(CS13) |  _BV(CS10); // prescaler=256
  //TCCR1 |= _BV(CS13) |  _BV(CS11); // prescaler=512
  //TCCR1 |= _BV(CS13) |  _BV(CS11) |  _BV(CS10); // prescaler=1024
  //TCCR1 |= _BV(CS13) |  _BV(CS12); // prescaler=2048
  //TCCR1 |= _BV(CS13) |  _BV(CS12) |  _BV(CS10); // prescaler=4096
  //TCCR1 |= _BV(CS13) |  _BV(CS12) |  _BV(CS11); // prescaler=8192
  //TCCR1 |= _BV(CS13) |  _BV(CS12) |  _BV(CS11) |  _BV(CS10); // prescaler=16384
  OCR1C = 249; // Set betw 1-255 (fclk=16MHz, prescaler=128, OCR1C=249 --> 500Hz)
  sei();       // enable interrupts
}

ISR(TIMER1_COMPA_vect) {
  if (cycles == 0) {
    TCNT1 = 187;        // offset the counting routine so that the ISR starts at 504 microseconds
    PORTB |= PORTMASK;  // set correct servo pin high
    for (byte i = 0; i < NSVO; i++) {
      pinStates[i] = HIGH;
    }
  } else if (cycles == 9) {
    TCNT1 = 62;  // last cycle length is adjusted to account for offset
  }

  if (cycles == 1) {  // only do this routine over the pulse width range
    while (TCNT1 < 249) {
      for (byte i = 0; i < NSVO; i++) {
        if (servo_attached[i] && pinStates[i] && (servo_PWs[i] <= (504 + ((unsigned int)TCNT1 * 8)))) {
          // Turn off the servo pin if the timer exceeds the pulse width
          PORTB &= ~(1 << (SVO1 + i));  // Set correct servo pin low
          pinStates[i] = LOW;
        }
      }
    }
	PORTB &= ~PORTMASK;	// cheap insurance if a pin doesn't go low
  }  // end if

  if (cycles < 9) { // we are only using 1/10th of the 50 Hz cycle to adjust pulse width.
    cycles++;
  } else {
    cycles = 0;
  }
}

void enableTimerInterrupt() {  // run this if you'd like to (re)enable CTC timer interrupt
#ifdef SERIALDEBUG
  mySerial.println("Timer1 enabled.");
#endif
  TIMSK |= (1 << OCIE1A);  // Enable Timer1 Compare Match A interrupt
  timer1_enabled = true;
}

void disableTimerInterrupt() {  // run this if you'd like to disable CTC timer interrupt. This will disable all servos.
#ifdef SERIALDEBUG
  mySerial.println("Timer1 disabled.");
#endif
  TIMSK &= ~(1 << OCIE1A);  // Disable Timer1 Compare Match A interrupt
  timer1_enabled = false;
}

void servo_timeout_check() {  // tol is added for potentiometer control. Default should be zero.
  if (((millis() - servo_tLast) > SVOTIMEOUT) && timer1_enabled) {
    disableTimerInterrupt();  // disable Timer1
  }
}
