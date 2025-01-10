/*  bruteForceServo.ino sketch (written for any non-PWM enabled pin)
 *  David Dubins 19-Dec-24
 *  Last Modified: 29-Dec-24
 *  This sketch does not require any additional libraries and does not use Timer1. It is a blocking version of the library. 
 *  This sketch inspired me to write tinyServo84 and tinyServo85.
 *  This sketch should work reasonably well on any microcontroller where delayMicroseconds() works.
 *  Optional direct port commands are included for the ATtiny85, for pins PB0, PB1, and PB2. These can be changed to any other digital pin
 *  capable of OUTPUT mode (PB3, PB4 can be added).
 *
 *  Connections:
 *  ============
 *  Servos: 
 *  Brown Wire - GND
 *  Red Wire - +5V
 *  Servo 0 Yellow Wire - PB0
 *  Servo 1 Yellow Wire - PB1
 *  Servo 2 Yellow Wire - PB2
 */

#define NSVO 3  // number of servos to control
// Servo Parameters:
byte sPin[NSVO] = { 0, 1, 2 };  // pin #s for servos PB0(D0), PB1(D1), PB2(D2)
#define SVOMAXANGLE 179         // maximum angle for servo.
#define SVOMINPULSE 500         // minimum pulse width in microseconds for servo signal (0 degrees). Default: 500
#define SVOMAXPULSE 2500        // maximum pulse width in microseconds for servo signal (for maximum angle). Default: 2500
#define SVOMAXTIME 1000         // maximum time it should take for servo to rotate at full speed between low and high limits.

void setup() {
  for (int i = 0; i < NSVO; i++) pinMode(sPin[i], OUTPUT);  // set all servo pins to OUTPUT mode
  homeServos();
}

void loop() {

  // Uncomment to move all 3 servos smoothly to different locations, using moveTo():
  /*moveTo(0, 179, 0, 1);      //go to 0,179,0. Larger last number = slower movement.
  moveTo(90, 90, 179, 1);    //go to 90,90,179
  moveTo(179, 0, 90, 1);     //go to 179,0,90
  moveTo(0, 0, 0, 1);        //go to 0,0,0
  moveTo(179, 179, 179, 1);  //go to 179,179,179
  moveTo(0, 0, 0, 1);        //go to 0,0,0
  moveTo(179, 179, 179, 1);  //go to 179,179,179
  */

  // Uncomment to move all 3 servos simultaneously to different locations:
  servoWriteAll(0, 179, 0, 1000);       //go to 0,179,0, signal should last 1000 msec
  servoWriteAll(90, 90, 179, 1000);     //go to 90,90,179, signal should last 1000 msec
  servoWriteAll(179, 0, 90, 1000);      //go to 179,0,90, signal should last 1000 msec
  servoWriteAll(0, 0, 0, 1000);         //go to 0,0,0 signal should last 1000 msec
  servoWriteAll(179, 179, 179, 1000);   //go to 179,179,179 signal should last 1000 msec

  // Uncomment to move servo according to a potentiometer reading:
  //int location = map(analogRead(A7, 0, 1023, SVOMAXANGLE, 0); // read voltage from a potentiometer wiper on pin A7 and rescale to servo angle.
  //servoWriteSame(location, 100);  // write location to all 3 servos

  // Uncomment to rock second servo only at full speed:
  //servoWrite(sPin[0], 0, 500);    // send servo 1 to angle 0, duration = 500 msec
  //servoWrite(sPin[0], 179, 500);  // send servo 1 to angle 179, duration = 500 msec
}

void moveTo(int s0, int s1, int s2, int wait) {  // routine for controlling 3 servos slowly, simultaneously.
  // wait=0: as fast as possible. do not use wait < 10 msec.
  // Change structure of moveTo based on # servos needed (add coordinates)
  int loc[NSVO] = { s0, s1, s2 };                       // create array for loc’ns
  static int pos[NSVO] = {90, 90, 90};                  // keep track of current position. Start somewhere convenient for your setup (90 degrees in this case).
  int dev = 0;                                          // to track deviation
  for (int i = 0; i < NSVO; i++) loc[i] = constrain(loc[i], 0, SVOMAXANGLE);  // impose limits for servos
  if (wait == 0) {                                      // if wait is zero, do the job as fast as possible.
    servoWriteAll(pos[0], pos[1], pos[2], SVOMAXTIME);  // write new position to servos
  } else {                                              // do the job in steps
    do {
      dev = 0;
      for (int i = 0; i < NSVO; i++) {  // moves servos one step
        if (loc[i] > pos[i]) pos[i]++;  // add 1 to pos[i]
        if (loc[i] < pos[i]) pos[i]--;  // subtr 1 from pos[i]
        dev += abs(pos[i] - loc[i]);    // calculate deviation
      }
      servoWriteAll(pos[0], pos[1], pos[2], wait);  // write new position to servos
    } while (dev > 0);                              // stop when location attained
  }
}

void homeServos() {         // even servo motors can be homed
  servoWriteSame(0, 1000);  // send the same signal to all 3 servos
}

void servoWrite(byte pin, byte angle, unsigned int dur) {                   // write a signal to a single servo for time dur (msec)
                                                                            // 50Hz custom duty cycle routine for servo control
                                                                            // This routine should be portable to other MCUs.
  unsigned long timer1 = millis();                                          // start the timer
  unsigned int tON = map(angle, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // pulse width usually 1000-2000us (full range 500-2500us)
  unsigned int tOFF = 20000 - tON;                                          // a 50 Hz pulse has a period of 20,000 us. tOFF should be 20,000-tON.
  while (millis() - timer1 < dur) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(tON);
    digitalWrite(pin, LOW);
    delayMicroseconds(tOFF);
  }
}

void servoWriteSame(byte angle, unsigned int dur) {  // write the same signal to all 3 servos
                                                     // This routine was written for the ATtiny85. Consult PORT architecture if using for other MCUs.
  unsigned long timer1 = millis();
  unsigned int tON = map(angle, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);
  unsigned int tOFF = 20000 - tON;  // a 50 Hz pulse has a period of 20,000 us. tOFF should be 20,000-tON.
  while (millis() - timer1 < dur) {
    for (int i = 0; i < NSVO; i++) digitalWrite(sPin[i], HIGH);  // set all servo pins HIGH
    // Faster: (but ATtiny85 specific)
    //PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB2);  // set pins PB0, PB1, and PB2 HIGH at the same time
    delayMicroseconds(tON);
    for (int i = 0; i < NSVO; i++) digitalWrite(sPin[i], LOW);  // set all servo pins LOW
    // Faster: (but ATtiny85 specific)
    //PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2));  // set pins PA2, PA3, and PA4 LOW at the same time
    delayMicroseconds(tOFF);
  }
}

void servoWriteAll(byte s0, byte s1, byte s2, unsigned int dur) {  // write different signals to all servos (more efficient)
                                                                   // This routine was written for the ATtiny85. Consult PORT architecture if using for other MCUs.
  unsigned long timer1 = millis();
  unsigned int tON0 = map(s0, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // tON for Servo 0.
  unsigned int tON1 = map(s1, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // tON for Servo 1.
  unsigned int tON2 = map(s2, 0, SVOMAXANGLE, SVOMINPULSE, SVOMAXPULSE);  // tON for Servo 2.
  while (millis() - timer1 < dur) {                                       // repeat for time dur in msec
    unsigned long timer2 = micros();                                      // start the microsecond timer
    for (int i = 0; i < NSVO; i++) digitalWrite(sPin[i], HIGH);              // set all servo pins HIGH
    // Faster (but ATtiny85 specific):
    //PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB2);                      // set pins PB0, PB1, and PB2 HIGH at the same time
    while (micros() - timer2 < 20000) {                                   // a 50 Hz pulse has a period of 20,000 us.
      if (micros() - timer2 > tON0) digitalWrite(sPin[0], LOW);           // turn off PA2 at the right time
      if (micros() - timer2 > tON1) digitalWrite(sPin[1], LOW);           // turn off PA3 at the right time
      if (micros() - timer2 > tON2) digitalWrite(sPin[2], LOW);           // turn off PA4 at the right time
    // Faster (but ATtiny85 specific):
    //if (micros() - timer2 > tON0) PORTB &= ~(1 << PB0);  // turn off PB0 at the right time
    //if (micros() - timer2 > tON1) PORTB &= ~(1 << PB1);  // turn off PB1 at the right time
    //if (micros() - timer2 > tON2) PORTB &= ~(1 << PB2);  // turn off PB2 at the right time
    }
  }
}
