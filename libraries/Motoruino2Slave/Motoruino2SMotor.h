/*
 Artica CC - http://artica.cc

 MotoruinoSlave Motor Control Lib
  - Encoder control (FAST: pulse width count SLOW: pulses count)
  - PID to set the Speed
  - Goto Distance function
  - Control PWM directly
  - ADC Current Measurement

 V0.0 - Serras and Tarquinio - 23/11/2015
 	 - Start of the project

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#ifndef Motoruino2SMotor_h
#define Motoruino2SMotor_h

#include "Arduino.h"
#include "Motoruino2SPINS.h"

// DEFAULT DEFINITIONS OF THE MOTOR ROTATION
#define M1_DIRECTION -1
#define M2_DIRECTION 1

// Default PID Controller Factors
#define KP_WHEEL	0.1
#define KD_WHEEL	0.2
#define KI_WHEEL	0.0

#define KP_MOTOR	0.007
#define KD_MOTOR	0.001
#define KI_MOTOR	0.0

// Interrupt Routines Definitions
#define TIMER1_RESET_RACIO 	10				// How many "ticks" before reset the Encoder - T1 Default freq 245Hz / 10 = 24,5Hz
#define TIMER1_FREQ 		245				// Timer1 frequency 245Hz - DO NOT CHANGE

#define CURRENT_POOLING_TIME 100			// Pooling time for motor current ms


class Motoruino2SPID
{
	public:
		Motoruino2SPID();

		void config(float _Kp = KP_WHEEL, float _Kd = KD_WHEEL, float _Ki = KI_WHEEL );

		float updatePID(float cte);

	private:
		volatile float pidPrevCTE, pidSumCTE;
		volatile bool pidFirstTime;
		volatile float factorP,factorD, factorI;
		volatile float pidRetValue;

};

class Motoruino2SMotor
{
	public:

		// -- PROPERTIES --
		// Types of encoders:
		// OPTICAL WHEEL - optical encoders directly applied to wheel: interrupt on change, measure by time - SLOWER
		// OPTICAL MOTOR - optical encoder applied on the "back" of the motor: interrupt on rising, measure by tick count - FASTER
		enum EncoderMode {NONE, SLOW_ENCODER , FAST_ENCODER};

		// TODO: Implement the methods to get values
		static volatile float desiredFrequency1;			// The goal frequency
		static volatile float desiredFrequency2;

		static volatile signed short counterTotalEncoder1;		// The "Pulse Counter" in each "Interrupt Tick Time"
		static volatile signed short counterTotalEncoder2;

		static volatile signed short counterEncoder1;		// The "Pulse Counter" in each "Interrupt Tick Time"
		static volatile signed short counterEncoder2;

		static volatile float frequency1;					// The current frequency
		static volatile float frequency2;

		static volatile float distance1;					// The distance (since last reset)
		static volatile float distance2;

//		// Current PWM Values for the motors
		static volatile unsigned char pwmIn1M1;				// PWM independent H Bridge values
		static volatile unsigned char pwmIn2M1;
		static volatile unsigned char pwmIn1M2;
		static volatile unsigned char pwmIn2M2;

		// Local var for the type of encoder selected
		volatile enum EncoderMode mode;						// What type of encoder (fast or slow)
		volatile bool isEncoderDefined;					// encoders configured? ... "fail safe flag"

		// Speed control var to apply to motor when "set speed" - PID reply
		static volatile float motorSpeed1;					// PID PWM raw Values (between 0 and 255)
		static volatile float motorSpeed2;					// PID PWM raw Values (between 0 and 255)


		// Used in Goto Distance - Maximum Speed Set
		static volatile long pwmGoalSpeed;						// PWM - Raw motor drive

		static volatile bool adcTriggerFlag;

		volatile float angle;
		volatile short angleLeftMotorPwm;
		volatile float angleRightMotorPwm;


		// -- METHODS --
		// Constructor
		Motoruino2SMotor();

		// Config Methods
		void config(int dir1 = M1_DIRECTION, int dir2 = M2_DIRECTION);

		void startEncoders(enum EncoderMode _mode, short _reduction_ratio  , float _wheels_diam_cm );
		void stopEncoders();

		// Encoder Methods
		long getSpeed(unsigned char motor);
		void setRPMMotorSpeed(short _DesiredFrequencyLeft, short _DesiredFrequencyRight);
		void resetDistance(unsigned char motor);
		float getDistance(unsigned char motor); // in cm

		// Overlaps any previous speed setup
		void setPWMMotorSpeed(short speedLeft, short speedRight);

		// Get ADC Current Value
		static unsigned short getADC();

		void setADCTrigger(unsigned short treshold);
		void startAdcTrigger();
		void stopAdcTrigger();

		static bool motorBlocked();

	private:

		// -- PROPERTIES --

		static Motoruino2SPID pid_1;
		static Motoruino2SPID pid_2;

		static volatile short reduction_ratio;			// = resolution of encoder per loop
		volatile float wheels_diam;						// = wheel diameter selected on the configuration (in cm)
		static volatile float perimeter_pertick;		// = (2*pi*(wheels_diam/2)/reduction_ratio)

		// aux var to decrement the ticks before reset encoder count
		static volatile unsigned short timer1Dec;

		static volatile int m1_dir, m2_dir;				// Motor Direction (1 or -1) define when configured

		static unsigned long oldMicros_1;
		static bool period1Updated;
		static unsigned long oldMicros_2;
		static bool period2Updated;

		static volatile bool automatic_motors_control;			// speed control flag ... set autonomous = true; set pwm = false

		static volatile short PWM_L_OLD, PWM_R_OLD;
		static volatile bool adcBlockedFlag;
		static volatile unsigned short adcTrigger;
		static volatile long startMotorMillis;



		// -- METHODS --

		static void computePWM(int speedLeft, int speedRight);

		static void interruptTimer1_Motor();
		static void interruptEncoder1_Motor();
		static void interruptEncoder2_Motor();

		static void interruptTimer1_Wheel();
		static void interruptEncoder1_Wheel();
		static void interruptEncoder2_Wheel();
};

extern Motoruino2SMotor motor;



#endif



