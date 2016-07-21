/*
 Artica CC - http://artica.cc

 Gyro Servo Lib - To use Servo Motors as an Add-on of Gyro

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#ifndef ServoControl_h
#define ServoControl_h
#include "Arduino.h"
#include <Servo.h>

// Default pin for the servo
#define SERVORANGE_DEFAULT_SERVO_PIN 9
// Minimum value the servo can move to 
#define SERVORANGE_DEFAULT_MINUMUM_ANGLE 60
// Maximum value the servo can move to 
#define SERVORANGE_DEFAULT_MAXIMUM_ANGLE 140

// Number of degrees the servo moves on each step
#define SERVORANGE_DEFAULT_STEP_ANGLE 1
#define SERVORANGE_DEFAULT_STEP_DELAY 20

class GyroServoControl
{
	public:
		// ---------------------------------------------------------------------
		// Initialization and configuration
		// Default constructor, uses pin SERVORANGE_DEFAULT_SERVO_PIN as default, 
		// SERVORANGE_DEFAULT_MINUMUM_ANGLE and SERVORANGE_DEFAULT_MAXIMUM_ANGLE 
		// as the minimum and maximum angles
		GyroServoControl();
		// Custom constructor, allows choosing the servo pin and type,
		// plus the minimum and maximum values for the servo rotation
		GyroServoControl(int servoPin, bool typeServo,
				   int minimumServoAngle = SERVORANGE_DEFAULT_MINUMUM_ANGLE, 
				   int maximumServoAngle = SERVORANGE_DEFAULT_MAXIMUM_ANGLE );

		// Configure the pin for the servo, the servo type, and
		// the minimum and maximum values for the servo rotation
		void Begin(int pinServo = SERVORANGE_DEFAULT_SERVO_PIN, 
				   bool typeServo = true,
				   int minimumServoAngle = SERVORANGE_DEFAULT_MINUMUM_ANGLE,
				   int maximumServoAngle = SERVORANGE_DEFAULT_MAXIMUM_ANGLE );

		// ---------------------------------------------------------------------
		// Movement methods
		// Move the servo to a specific angle (in degrees)
		void MoveTo(int angle);
		// Returns the current angle of the servo
		int GetPosition();

		// Configure a specific range and speed for the swiping movement of the servo
		void SetRange(int rangeMinAngle, int rangeMaxAngle, int incrementServo, int delayTime);

		// Move the servo within a specific range
		void Range();
		
	private:	
		// Configures all the necessary variables for the class
		void configure( int servoPin, bool typeServo, int minimumServoAngle, int maximumServoAngle);
		// Initializes the output pins for the servo
		void initialize();
		// IO pin the servo is connected to
		int _servoPin;
		// Type of servo
		bool _typeServo;
		// Minimum angle the servo can reach
		int _minimumServoAngle;
		// Maximum angle the servo can reach
		int _maximumServoAngle;
		
		// Is the servo initialized?
		// The initialization occurs when Begin is called, or when it is used the first time
		bool _initialized;

		// Current position of the servo
		int _servoPos;

		// Object used to handle the servo (using the Servo library)
		Servo _servo;

		// Last millis value the servo moved
		unsigned long _timeLastMovement;
		// Current direction of the sweeping movement of the servo
		bool _sweep_left;
		// Minimum value for the sweeping motion
		int _servoMinRange;
		// Maximum value for the sweeping motion
		int _servoMaxRange;
		// Number of degrees to move the servo on each update
		int _incrementServo;
		// Delay between each servo update
		unsigned long _delayTime;
};

#endif
