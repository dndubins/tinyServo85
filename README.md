<h1>Readme file for Arduino tinyServo85 Library</h1>

tinyServo85 is a library that can control up to 5 servos with the ATtiny85 microprocessor.

I made a similar library for the ATtiny84, and then thought it would be fun to create one for the ATtiny85.<p>

* The example sketch "1servo.ino" illustrates a few basic moves controlling 1 servo.
* The example sketch "3servos.ino" illustrates a few basic moves controlling 3 servos.
* The example sketch "5servos.ino" illustrates a few basic moves controlling 5 servos.
* For those of you who don't like libraries, I included a library-free version in a self-contained sketch, "tinyServo85_nolib.ino".
* The example sketch "bruteForceServo.ino" should drive a servo on most digital pins regardless of the mcu. It's the sketch you know you could have written in 5 minutes.

The functions available in the library include:
```
attachServo(byte servo_num); // to attach the servo
detachServo(byte servo_num); // to detach the servo, which also sets the corresponding signal pin to INPUT mode
setServo(byte servo_num, int angle); // to set the servo to a specific angle
homeServos(); // routine to home the servos
setCTC(); // sets the timer to CTC mode at 50Hz rollover
enableTimerInterrupt(); // re-enable the timer
disableTimerInterrupt(); // disable the timer
servo_timeout_check(); // a timer for servo inactivity to temporarily disable the timer
```
To use the library, copy the download to the Library directory.<p>
 
Technical notes:
- tinyServo85 was designed only for the ATTiny85, although the example sketch bruteForceServo.ino should work on other platforms.
- The library assumes a clock speed of 16MHz.
