/*
 Artica CC - http://artica.cc

 Motoruino 2 Motor Slave Interface Lib

 V0.0 - Serras and Tarquinio - 23/11/2015
 	 Motor Slave Interface Libs

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#ifndef _MOTORUINO2MOTOR_H_
#define _MOTORUINO2MOTOR_H_

#include "Arduino.h"


#define DEFAULTDIR1 -1
#define DEFAULTDIR2 -1

class Motoruino2Motor
{
	public:



		// ---------------------------------------------------------------------
		// Initialization and configuration
		// Default constructor, initializes the motors in the default pins (3,5,6,11) 
		Motoruino2Motor();

		// SLOW: using "Pulse Width" time calculation for Distance
		// FAST: using "Tick count" for an amount of time for distance
		enum EncoderMode {NONE, SLOW_ENCODER , FAST_ENCODER};

		bool config_motor(char _dir1 = DEFAULTDIR1, char _dir2 = DEFAULTDIR2);

		// Assign the speed and direction for both motors directly
		// The speed values are in the range [ -255, 255 ]
		bool setSpeedPWM( short speedLeft, short speedRight );
		bool setSpeedPWMLeft( short speedLeft);
		bool setSpeedPWMRight( short speedRight);


		// Sets the H Bridge PWM Independently - Can be used for other applications example as a LED Driver
		bool pwmSet(char IN1_M1, char IN2_M1, char IN1_M2, char IN2_M2);

		// Encoder Functions
		// Set speed in rpm (+rpm and -rpm) return false if encoder not set
		bool setSpeedRPM(short speedLeft = 0, short speedRight = 0);
		bool setSpeedRPMLeft( short speedLeft);
		bool setSpeedRPMRight( short speedRight);

		bool isEncoderSet();			// Returns True if encoder configured (Started)
		bool startEncoder(enum EncoderMode mode, long ticksPerTurn, float diameter);	// diameter in cm
		bool stopEncoder();

		// Reset the absolute of the distance
		bool resetDistance();

		// Reset the absolute Angle
		bool resetAngle();

		// Current Control Params
		bool getCurrentValue(unsigned short * Value);
		bool startMotorTrigger(unsigned short Value);
		bool stopMotorTrigger();



	private:

		char dir1;
		char dir2;

		short PWMLeft;
		short PWMRight;

		short RPMLeft;
		short RPMRight;

		// Is the sensor initialized?
		// The initialization occurs when Begin is called, or when it is used the first time
		bool _initialized;
};
#endif
