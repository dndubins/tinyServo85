#ifndef tinyServo85_h
#define tinyServo85_h

#include <Arduino.h>

#define NSVO 5            // number of servos to control (up to 5)
#define SVOMAXANGLE 179   // maximum angle for servo.
#define SVOMINPULSE 504   // minimum pulse width in microseconds for servo signal (0 degrees)
#define SVOMAXPULSE 2485  // maximum pulse width in microseconds for servo signal (for maximum angle)
#define SVOTIMEOUT 500    // timeout in ms to disable servos.

class tinyServo85 {
  public:
    tinyServo85();
    void attachServo(byte servo_num);
    void detachServo(byte servo_num);
    void setServo(byte servo_num, int angle);
    void homeServos();
    void setCTC();
    void enableTimerInterrupt();
    void disableTimerInterrupt();
    void servo_timeout_check();
    static unsigned int servo_PWs[NSVO]; // Pulse widths in microseconds
    static bool servo_attached[NSVO];    // Servo attachment status
	static volatile bool pinStates[NSVO];       // to store pin states
    static bool timer1_enabled;		     // To keep track if Timer1 is enabled
    static unsigned long servo_tLast;	 // To store the time the last servo was used (timeout function)
	static volatile int cycles;          // to hold #cycles ISR ran within one period of the servo signal
	static byte PORTMASK;                // to store attached servos (saves a few steps inside ISR)
};

#endif
