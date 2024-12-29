// 1servo.ino: Example sketch for tinyServo85
// tinySero85 version 1.0.0
// Author: D.Dubins
// Date: 29-Dec-24
// Controlling 1 servo on any digital pin from PB0 to PB4 on ATtiny85 (clock frequency=16MHz)
// The library maps specific servo numbers to the following pins:
// servo 0: PB0
// servo 1: PB1
// servo 2: PB2
// servo 3: PB3
// servo 4: PB4

#include "tinyServo85.h"
byte motor1 = 2; // motor1 will be connected to PB2 (using guide above)

// Change these values to suit your application:
#define SVOMAXANGLE 179   // maximum angle for servo.
// Note: The cycle of Timer1 has been adjusted to fit 249 divisions within 50-2000ms. Changing SVOMINPULSE 
// and SVOMAXPULSE outside the range 504-2485 will adversely impact the functionality of the timing routine.
#define SVOMINPULSE 504   // minimum pulse width in microseconds for servo signal (0 degrees). Default: 504
#define SVOMAXPULSE 2485  // maximum pulse width in microseconds for servo signal (for maximum angle). Default: 2485
#define SVOTIMEOUT 500    // timeout in ms to disable servos.

tinyServo85 myServos;     // declare object called myServos of class tinyServo85

void setup() {
  myServos.setCTC();
  myServos.attachServo(motor1); // attach motor1
  myServos.homeServos(); // home all attached servos
}

void loop() {
  // Uncomment the timeout check below for disabling Timer1, if the servos don't receive a command after
  // SVOTIMEOUT msec. This servo_timeout_check() is optional. Temporarily turning off Timer1 will free
  // the mcu to do other things. You can also manually suspend Timer1 with the command "myServos.disableTimerInterrupt();".
  myServos.servo_timeout_check();  // if servos are inactive, stop Timer1 (less trouble for other routines)
  
  // Uncomment for rocking motor1 at full speed.
  myServos.setServo(motor1, 0);
  delay(1000);
  myServos.setServo(motor1, SVOMAXANGLE);
  delay(1000);

  // Uncomment to rock motor1 using moveTo(), with a 10ms delay between steps:
  //moveTo(motor1, 0, 10);
  //moveTo(motor1, SVOMAXANGLE, 10);

  // Uncomment to move motor1 slowly (0.5 second steps)
  /*for (int i = 0; i < SVOMAXANGLE; i++) {
    myServos.setServo(motor1, i);
    delay(500); // 0.5s delay should let you see each angle
  }
  for (int i = SVOMAXANGLE; i >=0; i--) {
    myServos.setServo(motor1, i);
    delay(500);
  }*/

  // Uncomment for potentiometer control:
  //int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A3 & remap to angle.
  //myServos.setServo(motor1, location);  // write new location to servo 0
  //delay(50);              // wait a bit to reduce jittering

  // Uncomment for potentiometer control of motor1 with smoother movement:
  //int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A3 & remap to angle.
  //moveTo(motor1, location, 5);  // move to new location, delay=4 ms between steps

  // Uncomment for potentiometer control, disabling the timer between similar readings within a tolerance:
  /*#define TOL 1 // Tolerance to ignore readings. A greater than 1 degree difference will be sent to the servo.
  static int lastlocation;
  int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A3 & remap to angle.
  if(abs(location-lastlocation)>TOL){  // if readings are more than 1 degreee away
    myServos.setServo(motor1, location);  // write new location to servo 0
  }else{
    myServos.disableTimerInterrupt(); // disable timers while waiting for a different reading
  }
  lastlocation=location;
  delay(50);              // wait a bit to reduce jittering
  */
}

void moveTo(byte servo_num, int s0, int wait) { // routine to move servo slower (more smoothly)
  int loc = s0; // store s0 to loc
  static int pos; // keep track of current position
  if (wait == 0) { // option to move as fast as possible
    myServos.setServo(servo_num, loc); // send new location to servo
  } else {
    int dev = 0; // to keep track of deviation from s0
    do {
      dev = 0;
      if (loc > pos) pos++;
      if (loc < pos) pos--;
      dev += abs(pos - loc);
      myServos.setServo(servo_num, pos);
      delay(wait); // wait a bit (slows down movement)
    } while (dev > 0);
  }
}
