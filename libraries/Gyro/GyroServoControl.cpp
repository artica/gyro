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


#include "GyroServoControl.h"

// ---------------------------------------------------------------------
// Initialization and configuration

// Default constructor, uses pin SERVORANGE_DEFAULT_SERVO_PIN as default, 
// SERVORANGE_DEFAULT_MINUMUM_ANGLE and SERVORANGE_DEFAULT_MAXIMUM_ANGLE 
// as the minimum and maximum angles
GyroServoControl::GyroServoControl()
{
	configure( SERVORANGE_DEFAULT_SERVO_PIN, true, 
		SERVORANGE_DEFAULT_MINUMUM_ANGLE, SERVORANGE_DEFAULT_MAXIMUM_ANGLE );
}

// Custom constructor, allows choosing the servo pin and type,
// plus the parameters for the sweeping movement
GyroServoControl::GyroServoControl( int servoPin, bool typeServo,
						int minimumServoAngle, int maximumServoAngle )
{
	configure( servoPin, typeServo, minimumServoAngle, maximumServoAngle );
}

// Configure the pin for the servo, the servo type, and optionally
// the minimum and maximum values for the servo rotation
void GyroServoControl::Begin(int servoPin, bool typeServo, int minimumServoAngle, int maximumServoAngle )
{
	configure( servoPin, typeServo, minimumServoAngle, maximumServoAngle );	
	initialize();		
}

// Configure the necessary values to initialize the ServoControl class
void GyroServoControl::configure(int servoPin, bool typeServo, int minimumServoAngle, int maximumServoAngle )
{
	_initialized = false;
	// Base configuration for the servo
	_servoPin = servoPin;
	_typeServo = typeServo;
	_minimumServoAngle = minimumServoAngle;
	_maximumServoAngle = maximumServoAngle;
	
	// The initial servo position is the middle of the servo bounds
	_servoPos = ( _minimumServoAngle + _maximumServoAngle ) / 2;
		
	// Configuration for the default swiping movement
	_servoMinRange = minimumServoAngle;
	_servoMaxRange = maximumServoAngle;
	_incrementServo = SERVORANGE_DEFAULT_STEP_ANGLE;
	_delayTime = SERVORANGE_DEFAULT_STEP_DELAY;
	// Auxiliar variables for the swiping movement
	_timeLastMovement = 0;
	_sweep_left = false;
}

// Initializes all the necessary variables and pins so they can be used properly
void GyroServoControl::initialize()
{
	_servo.attach(_servoPin, _minimumServoAngle, _maximumServoAngle);	
	_initialized = true;	
}

// ---------------------------------------------------------------------
// Movement methods

// Move the servo to a specific angle (in degrees)
void GyroServoControl::MoveTo(int angle)
{
	//Serial.print("MoveTo="); Serial.println(angle);
	// Initializes the pins and variables, if they haven't been initialized yet
	if (!_initialized) initialize();
	_servoPos = angle;
	// The movement angle depends on the servo type
	if (_typeServo)
	{
		_servo.write(_servoPos);
	}
	else
	{
		_servo.write(180-_servoPos);
	}
}

// Returns the current angle of the servo
int GyroServoControl::GetPosition()
{
	return _servoPos;
}

// Configure a specific range and speed for the swiping movement of the servo
void GyroServoControl::SetRange(int servoMinRange, int servoMaxRange, int incrementServo, int delayTime)
{
	_servoMinRange = servoMinRange;
	_servoMaxRange = servoMaxRange;
	_incrementServo = incrementServo;
	_delayTime = delayTime;	
}

//Manages the servo rotation between the maximum and minimum values received as arguments
void GyroServoControl::Range()
{
	// Checks if it's time to move the servo again
	if ( millis() - _timeLastMovement > _delayTime )
	{
		// If it reaches the minimum or maximum values, change direction
		if(_servoPos >= _servoMaxRange) _sweep_left = true;
		else if(_servoPos <= _servoMinRange) _sweep_left = false;

		// Rotate the servo left or right, depending on the current sweep direction
		if (_sweep_left) MoveTo(_servoPos - _incrementServo);
		else MoveTo(_servoPos + _incrementServo);
		
		// Saves the time the movement was performed
		_timeLastMovement = millis();
	}
}
