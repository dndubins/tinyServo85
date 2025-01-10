// 3servos.ino: Example sketch for tinyServo85
// tinyServo85 version 1.0.0
// Author: D. Dubins
// Date: 29-Dec-24
// Controlling 3 servos from PB0 to PB2 on ATtiny85 (clock frequency=16MHz)
// The library maps specific servo numbers to the following pins:
// servo 0: PB0
// servo 1: PB1
// servo 2: PB2
// servo 3: PB3
// servo 4: PB4

#include "tinyServo85.h"

// Change these values to suit your application:
#define NSERVO 3          // number of servos to control (up to 5)
#define SVOMAXANGLE 179   // maximum angle for servo.
// Note: The cycle of Timer1 has been adjusted to fit 249 divisions within 50-2000ms. Changing SVOMINPULSE 
// and SVOMAXPULSE outside the range 504-2485 will adversely impact the functionality of the timing routine.
#define SVOMINPULSE 504   // minimum pulse width in microseconds for servo signal (0 degrees). Default: 504
#define SVOMAXPULSE 2485  // maximum pulse width in microseconds for servo signal (for maximum angle). Default: 2485
#define SVOTIMEOUT 500    // timeout in ms to disable servos.

tinyServo85 myServos;  // declare object called myServos of class tinyServo85

void setup() {
  myServos.setCTC();
  for (int i = 0; i < NSERVO; i++) {  // Attach servo 0 on PA0, servo 1 on PA1 ... up to NSERVO,
    myServos.attachServo(i);          // following the guide in the sketch header.
  } 
  myServos.homeServos();      // home any attached servos
}

void loop() {
  // Uncomment the timeout check below for disabling Timer1, if the servos don't receive a command after
  // SVOTIMEOUT msec. This servo_timeout_check() is optional. Temporarily turning off Timer1 will free
  // the mcu to do other things. You can also manually suspend Timer1 with the command "myServos.disableTimerInterrupt();".
  //myServos.servo_timeout_check();  // if servos are inactive, stop Timer1 (less trouble for other routines)

  // Uncomment to rock all servos simultaneously, at full speed.
  for (int i = 0; i < NSERVO; i++) {
    myServos.setServo(i, 0);
  }
  delay(1000);
  for (int i = 0; i < NSERVO; i++) {
    myServos.setServo(i, SVOMAXANGLE);
  }
  delay(1000);

  // Uncomment to rock all servos smoothly using moveTo(), with a 10ms delay between steps:
  //moveTo(0, 0, 0, 10);
  //moveTo(SVOMAXANGLE, SVOMAXANGLE, SVOMAXANGLE, 10);
  
  // Uncomment to rock servo 1 slowly
  /*for (int i = 0; i < SVOMAXANGLE; i++) {
    myServos.setServo(1, i);
    delay(500); // 0.5s delay should let you see each angle
  }
  for (int i = SVOMAXANGLE; i >=0; i--) {
    myServos.setServo(1, i);
    delay(500);
  }*/

  // Uncomment to rock all servos at full speed through 0-SVOMAXANGLE, sequentially.
  /*for (int i = 0; i < NSERVO; i++) {
    myServos.setServo(i, 0);
    delay(1000);
    myServos.setServo(i, SVOMAXANGLE);
    delay(1000);
  }*/

  // Uncomment for potentiometer control:
  /*int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A3 & remap to angle.
  myServos.setServo(0, location);  // write new location to servo 0
  myServos.setServo(1, location);  // write new location to servo 1
  myServos.setServo(2, location);  // write new location to servo 2
  delay(50);                      // wait a bit to reduce jittering
  */
  
  // Uncomment for potentiometer control of all servos with smoother movement:
  //int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A3 & remap to angle.
  //myServos.moveTo(location, location, location, 5);  // move to new location, delay=4 ms between steps

  // Uncomment for potentiometer control, disabling the timer between similar readings within a tolerance:
  /*#define TOL 1 // Tolerance to ignore readings. A greater than 1 degree difference will be sent to the servo.
  static int lastlocation;
  int location = map(analogRead(A3), 1023, 0, 0, SVOMAXANGLE); // take pot reading from pin A3 & remap to angle.
  if(abs(location-lastlocation)>TOL){  // if readings are more than 1 degreee away
    myServos.setServo(0, location);  // write new location to servo 0
    myServos.setServo(1, location);  // write new location to servo 1
    myServos.setServo(2, location);  // write new location to servo 2
  }else{
    myServos.disableTimerInterrupt(); // disable timers while waiting for a different reading
  }
  lastlocation=location;
  delay(50);              // wait a bit to reduce jittering
  */
}

void moveTo(int s0, int s1, int s2, int wait) {  // routine for controlling all servos slowly, simultaneously.
  // wait=0: as fast as possible.
  // Note: Change structure of moveTo arguments to match # servos (add coordinates s3, s4 ... as needed).
  int loc[NSERVO] = { s0, s1, s2 };  // create array for loc’ns
  static int pos[NSERVO] = {90, 90, 90};  // remembers last value of pos. Start somewhere convenient for your setup (90 degrees in this case).
  if (wait == 0) {                   // if wait=0, move as fast as possible
    for (int i = 0; i < NSERVO; i++) {
      myServos.setServo(i, loc[i]);  // write new position to servos
    }
  } else {
    int dev = 0;  // to track deviation
    do {
      dev = 0;
      for (int i = 0; i < NSERVO; i++) {  // moves servos one step
        if (loc[i] > pos[i]) pos[i]++;    // add 1 to pos[i]
        if (loc[i] < pos[i]) pos[i]--;    // subtr 1 from pos[i]
        dev += abs(pos[i] - loc[i]);      // calculate deviation
      }
      for (int i = 0; i < NSERVO; i++) {
        myServos.setServo(i, pos[i]);  // write new position to servos
      }
      delay(wait);      // slow down movement
    } while (dev > 0);  // stop when location attained
  }                     // end if
}
