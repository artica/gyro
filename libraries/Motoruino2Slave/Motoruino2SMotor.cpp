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

#include "Arduino.h"
#include "Motoruino2SMotor.h"
#include "TimerOne.h"
#include "digitalWriteFast.h"  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/



// All Variables used on interrupts must be static, otherwise it was not possibe
volatile float  Motoruino2SMotor::desiredFrequency1;
volatile float  Motoruino2SMotor::desiredFrequency2;
volatile float  Motoruino2SMotor::motorSpeed1;
volatile float  Motoruino2SMotor::motorSpeed2;
volatile signed short  Motoruino2SMotor::counterEncoder1;
volatile signed short  Motoruino2SMotor::counterEncoder2;
volatile signed short  Motoruino2SMotor::counterTotalEncoder1;
volatile signed short  Motoruino2SMotor::counterTotalEncoder2;
volatile float  Motoruino2SMotor::frequency1;
volatile float  Motoruino2SMotor::frequency2;
volatile float Motoruino2SMotor::distance1;
volatile float Motoruino2SMotor::distance2;
volatile unsigned short Motoruino2SMotor::timer1Dec;
volatile int Motoruino2SMotor::m1_dir;
volatile int Motoruino2SMotor::m2_dir;
unsigned long Motoruino2SMotor::oldMicros_1;
bool Motoruino2SMotor::period1Updated;
unsigned long Motoruino2SMotor::oldMicros_2;
bool Motoruino2SMotor::period2Updated;
Motoruino2SPID Motoruino2SMotor::pid_1;
Motoruino2SPID Motoruino2SMotor::pid_2;
volatile float Motoruino2SMotor::perimeter_pertick;
volatile bool Motoruino2SMotor::automatic_motors_control;
volatile short Motoruino2SMotor::reduction_ratio;
volatile long Motoruino2SMotor::pwmGoalSpeed;
volatile unsigned char Motoruino2SMotor::pwmIn1M1;
volatile unsigned char Motoruino2SMotor::pwmIn2M1;
volatile unsigned char Motoruino2SMotor::pwmIn1M2;
volatile unsigned char Motoruino2SMotor::pwmIn2M2;
volatile bool Motoruino2SMotor::adcTriggerFlag, Motoruino2SMotor::adcBlockedFlag;
volatile unsigned short Motoruino2SMotor::adcTrigger;
volatile long Motoruino2SMotor::startMotorMillis;
volatile short Motoruino2SMotor::PWM_L_OLD, Motoruino2SMotor::PWM_R_OLD;

Motoruino2SMotor motor;

Motoruino2SMotor::Motoruino2SMotor()
{
	config();
}


// -- Config Motor -- Define the direction of both motoruino motors
void Motoruino2SMotor::config(int dir1, int dir2)
{
	m1_dir = dir1; m2_dir = dir2;

	//Serial.print(":");Serial.print(dir1);Serial.print(" ");Serial.print(dir2);Serial.println(" ");

	mode = NONE;

	desiredFrequency1 = 0;	desiredFrequency2 = 0;
	motorSpeed1 = 0;		motorSpeed2 = 0;
	counterEncoder1 = 0;	counterEncoder2 = 0;
	frequency1 = 0;			frequency2 = 0;
	distance1 = 0;			distance2 = 0;

	oldMicros_1 = 0; 		oldMicros_2 = 0;
	period1Updated = false;	period2Updated = false;

	perimeter_pertick = 0;

	adcBlockedFlag = false;
	adcTrigger = 0;
	adcTriggerFlag = false;

	automatic_motors_control = false;

	isEncoderDefined = false;

	timer1Dec = TIMER1_RESET_RACIO;

	pinMode(M1_IN1, OUTPUT);
	pinMode(M1_IN2, OUTPUT);
	pinMode(M2_IN1, OUTPUT);
	pinMode(M2_IN2, OUTPUT);

	analogWrite(M1_IN1, 0);
	analogWrite(M1_IN2, 0);
	analogWrite(M2_IN1, 0);
	analogWrite(M2_IN2, 0);

	adcTriggerFlag = false;

	computePWM(0, 0);
}

// -- ADC Trigger function ---
unsigned short Motoruino2SMotor::getADC()
{
	return analogRead(MOTOR_ADC);
}

void Motoruino2SMotor::startAdcTrigger()
{
	Serial.println("START TRIG");

	adcTriggerFlag = true;
	startMotorMillis = millis();
}

void Motoruino2SMotor::stopAdcTrigger()
{
	Serial.println("STOP TRIG");
	adcTriggerFlag = false;
}

// --- Config Limit Value ---
void Motoruino2SMotor::setADCTrigger(unsigned short treshold)
{
	Serial.println("SET TRIG");
	adcTriggerFlag = true;

	adcTrigger = treshold;
}


void Motoruino2SMotor::stopEncoders(void)
{
	detachInterrupt(0);
	detachInterrupt(1);
	Timer1.detachInterrupt();
	isEncoderDefined = false;
}
// Define the type of encoder; the reductionRacio (ticks per rotation) and Wheel Diameter
void Motoruino2SMotor::startEncoders(enum EncoderMode _mode, short _reduction_ratio  , float _wheels_diam_cm )
{
	mode = _mode;
	reduction_ratio = _reduction_ratio;
	wheels_diam = _wheels_diam_cm;

	if (!reduction_ratio) reduction_ratio = 1;

	// Calculate the Perimeter per Encoder TICK divide by reduction
	perimeter_pertick = (PI*(wheels_diam)) / (reduction_ratio) ;

	pinMode(ENC1_INT, INPUT);
	pinMode(ENC2_INT, INPUT);
	pinMode(ENC1_IO, INPUT);
	pinMode(ENC2_IO, INPUT);

	switch (mode)
	{
		case SLOW_ENCODER:

			pid_1.config(KP_WHEEL, KI_WHEEL, KD_WHEEL);
			pid_2.config(KP_WHEEL, KI_WHEEL, KD_WHEEL);

			attachInterrupt(0, interruptEncoder1_Wheel, CHANGE);
			attachInterrupt(1, interruptEncoder2_Wheel, CHANGE);

			Timer1.attachInterrupt(interruptTimer1_Wheel);

 			break;

		case FAST_ENCODER:

			Serial.print("Fast Enc ");
			Serial.print(_reduction_ratio);
			Serial.print(" ");
			Serial.println(_wheels_diam_cm);

			pid_1.config(KP_MOTOR, KI_MOTOR, KD_MOTOR);
			pid_2.config(KP_MOTOR, KI_MOTOR, KD_MOTOR);

			attachInterrupt(0, interruptEncoder1_Motor, RISING);
			attachInterrupt(1, interruptEncoder2_Motor, RISING);

			//Timer1.initialize(125);							//ms...
			Timer1.attachInterrupt(interruptTimer1_Motor);

			break;


		default:
			break;
	};

	frequency1 = 0;		motorSpeed1 = 0;
	frequency2 = 0;		motorSpeed2 = 0;

	computePWM(0, 0);

	isEncoderDefined = true;

}
long Motoruino2SMotor::getSpeed(unsigned char motor)
{
	if (motor == 1)
		return frequency1;
	else if (motor == 2)
		return frequency2;
	return 0;
}
void Motoruino2SMotor::setRPMMotorSpeed(short _DesiredFrequencyLeft, short _DesiredFrequencyRight)
{


	automatic_motors_control = true;


		desiredFrequency1 = (long) _DesiredFrequencyLeft;
		desiredFrequency2 = (long) _DesiredFrequencyRight;
}


void Motoruino2SMotor::resetDistance(unsigned char motor)
{
	if (motor == 1)
		distance1 = 0;
	else if (motor == 2)
		distance2 = 0;
}
float Motoruino2SMotor::getDistance(unsigned char motor)
{
	if (!isEncoderDefined) return 0;

	if (motor == 1)
		return distance1;
	else if (motor == 2)
		return distance2;

	return 0;
}

short returnMaxSpeed(short speedLeft, short speedRight)
{
	short speedLeftAbs = abs(speedLeft);
	short speedRightAbs = abs(speedRight);

	if (speedLeftAbs > speedRightAbs)
	{
			return speedLeft;
	} else
	{
			return speedRight;
	}
}

// Function that overlaps the "Automatic Speed Control Functions" and updates values
void Motoruino2SMotor::setPWMMotorSpeed(short speedLeft, short speedRight)
{
	automatic_motors_control = false;

	motorSpeed1 = speedLeft;
	motorSpeed2 = speedRight;

	computePWM(speedLeft, speedRight);
}



bool Motoruino2SMotor::motorBlocked()
{


	if (adcTriggerFlag && ((millis() - startMotorMillis) > CURRENT_POOLING_TIME))
	{

		if (getADC() > adcTrigger){

			startMotorMillis = millis();

			automatic_motors_control = false;

			analogWrite(M1_IN1, 0);
			analogWrite(M1_IN2, 0);
			analogWrite(M2_IN1, 0);
			analogWrite(M2_IN2, 0);

			Serial.println("FAILED");

			adcBlockedFlag = true;

			return true;

		} else {

			if (adcBlockedFlag && ((millis() - startMotorMillis) > (CURRENT_POOLING_TIME*5)))
			{

				startMotorMillis = millis();

				analogWrite(M1_IN1, pwmIn1M1);
				analogWrite(M1_IN2, pwmIn2M1);
				analogWrite(M2_IN1, pwmIn1M2);
				analogWrite(M2_IN2, pwmIn2M2);

				adcBlockedFlag = false;

				return false;
			}
		}

	}

	return false;
}


// PWM Raw Motor Functions Set -> Direct PWM action
void Motoruino2SMotor::computePWM(int speedLeft, int speedRight)
{

	if (adcBlockedFlag) return;


	// Sets the correct rotation of the motor
	speedLeft *= m1_dir;
	speedRight *= m2_dir;

	// Sets the M2 (Left) Motor
	if (speedLeft < 0)
	{
		pwmIn1M2 = 0;
		pwmIn2M2 = (byte) speedLeft * -1;
	}
	else
	{
		pwmIn1M2 = (byte) speedLeft;
		pwmIn2M2 = 0;
	}

	// Sets the M1 (Right) Motor
	if (speedRight < 0)
	{
		pwmIn1M1 = 0;
		pwmIn2M1 = (byte) speedRight * -1;
	}
	else
	{
		pwmIn1M1 = (byte) speedRight;
		pwmIn2M1 = 0;
	}

	analogWrite(M1_IN1, pwmIn1M1);
	analogWrite(M1_IN2, pwmIn2M1);
	analogWrite(M2_IN1, pwmIn1M2);
	analogWrite(M2_IN2, pwmIn2M2);

}




// ---------------------------------- Interrupts --------------------------------------------

void Motoruino2SMotor::interruptEncoder1_Motor()
{
	static volatile bool EncoderBSet1;
	  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
	EncoderBSet1 = digitalReadFast(ENC1_IO);   // read the input pin
	counterEncoder1 -= (EncoderBSet1 ? -1 : +1);

}

void Motoruino2SMotor::interruptEncoder2_Motor()
{
	static volatile bool EncoderBSet2;
	  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
	EncoderBSet2 = digitalReadFast(ENC2_IO);   // read the input pin
	counterEncoder2 -= (EncoderBSet2 ? -1 : +1);
}

void Motoruino2SMotor::interruptTimer1_Motor()
{
	if (!timer1Dec--)
	{
		timer1Dec = TIMER1_RESET_RACIO;



		// Process Encoder1
		counterEncoder1 *= m1_dir;
		distance1 += counterEncoder1*perimeter_pertick;
		frequency1 = (float)counterEncoder1*(float)TIMER1_FREQ/TIMER1_RESET_RACIO;


		counterTotalEncoder1 += counterEncoder1;
		counterEncoder1=0;

		if (desiredFrequency1)
			motorSpeed1 +=  pid_1.updatePID(desiredFrequency1 - frequency1);

		if (motorSpeed1 > 254) motorSpeed1 = 254;
		if (motorSpeed1 < -254) motorSpeed1 = -254;



		// Process Encoder2
		counterEncoder2 *= m2_dir;

		distance2 += counterEncoder2*perimeter_pertick;
		frequency2 = (float)counterEncoder2*(float)TIMER1_FREQ/TIMER1_RESET_RACIO;
		counterTotalEncoder2 += counterEncoder2;
		counterEncoder2=0;

		if (desiredFrequency2)
			motorSpeed2 +=  pid_2.updatePID(desiredFrequency2 - frequency2);

		if (motorSpeed2 > 254) motorSpeed2 = 254;
		if (motorSpeed2 < -254) motorSpeed2 = -254;

		if (automatic_motors_control) computePWM(motorSpeed1, motorSpeed2);


	}

}



void Motoruino2SMotor::interruptEncoder1_Wheel()
{
	unsigned long microsData = micros();
	long Period_1;
	signed char sig = -1 * (m1_dir);

	Period_1 =  microsData - oldMicros_1;
	oldMicros_1 = microsData;

	if (Period_1 == 0) Period_1 = 1;

	if (Period_1 > 10000000)
	{
		Period_1 = 10000000;
	}
	else
	{


		// Invert Signal if other side (XOR)
		if ((digitalReadFast(ENC1_INT) == HIGH) && (digitalReadFast(ENC1_IO) == HIGH))
		{
			sig = -sig;
			//Period_1 = -Period_1;
		}
		else if ((digitalReadFast(ENC1_INT) == LOW) && (digitalReadFast(ENC1_IO) == LOW))
		{
			sig = -sig;
			//Period_1 = -Period_1;
		}

		Period_1 *= sig;

	}

	period1Updated = true;

	//frequency1 = 60*(1000000/(2*Period_1))/(reduction_ratio);
	frequency1 = 30000000/(Period_1*reduction_ratio);

	// divide by two because it enters the interrupt twice (on change)
	distance1 += sig*perimeter_pertick/2;

	if (desiredFrequency1)
		motorSpeed1 =  pid_1.updatePID(desiredFrequency1 - frequency1);

	if (motorSpeed1 > 254) motorSpeed1 = 254;
	if (motorSpeed1 < -254) motorSpeed1 = -254;

	if (automatic_motors_control) computePWM(motorSpeed1, motorSpeed2);

}


void Motoruino2SMotor::interruptEncoder2_Wheel()
{
	unsigned long microsData = micros();
	long Period_2;
	signed char sig = -1 * (m2_dir);

	Period_2 =  microsData - oldMicros_2;
	oldMicros_2 = microsData;


	if (Period_2 == 0) Period_2 = 1;

	if (Period_2 > 10000000)
	{
		Period_2 = 10000000;

	}
	else
	{
		//Period_2 *= -1*(m2_dir);
		// Invert Signal if other side (XOR)
		if ((digitalReadFast(ENC2_INT) == HIGH) && (digitalReadFast(ENC2_IO) == HIGH))
		{
			sig = -sig;//Period_2 = -Period_2;
		}
		else if ((digitalReadFast(ENC2_INT) == LOW) && (digitalReadFast(ENC2_IO) == LOW))
		{
			sig = -sig;//Period_2 = -Period_2;
		}

		Period_2 *= sig;
	}

	period2Updated = true;

	//frequency2 = (500000/Period_2);
	frequency2 = 30000000/(Period_2*reduction_ratio);

	// divide by two because it enters the interrupt twice (on change)
	distance2 += sig*perimeter_pertick/2;

	if (desiredFrequency2)
		motorSpeed2 =  pid_2.updatePID(desiredFrequency2 - frequency2);

	if (motorSpeed2 > 254) motorSpeed2 = 254;
	if (motorSpeed2 < -254) motorSpeed2 = -254;

	if (automatic_motors_control) computePWM(motorSpeed1, motorSpeed2);
}


void Motoruino2SMotor::interruptTimer1_Wheel()
{
	if (!timer1Dec--)
	{
		timer1Dec = TIMER1_RESET_RACIO;

		if (period1Updated == false)
		{
			// Update with frequency = 0!!
			if (desiredFrequency1)
				motorSpeed1 =  motorSpeed1 + pid_1.updatePID(desiredFrequency1)/2;

			if (motorSpeed1 > 254) motorSpeed1 = 254;
			if (motorSpeed1 < -254) motorSpeed1 = -254;

		} else period1Updated = false;


		if (period2Updated == false)
		{
			// Update with frequency = 0!!
			if (desiredFrequency2)
				motorSpeed2 = motorSpeed2 + pid_2.updatePID(desiredFrequency2)/2;

			if (motorSpeed2 > 254) motorSpeed2 = 254;
			if (motorSpeed2 < -254) motorSpeed2 = -254;



		} else period2Updated = false;

		if (automatic_motors_control) computePWM(motorSpeed1, motorSpeed2);
	}

}

// ---------------------------------- PID Controller --------------------------------------------


Motoruino2SPID::Motoruino2SPID()
{
	config();
}


void Motoruino2SPID::config(float _Kp, float _Kd, float _Ki)
{
  pidFirstTime = true;
  pidSumCTE =0;
  factorP = _Kp;
  factorD = _Kd;
  factorI = _Ki;
  pidRetValue =0;
}

float Motoruino2SPID::updatePID(float cte)
{

    if(pidFirstTime)
    {
      pidFirstTime = false;
      pidPrevCTE = cte;
    }

    float proportional = cte * factorP;

    float d = pidPrevCTE - cte;
    pidPrevCTE = cte;

    float derivative = d * factorD;

    pidSumCTE += cte;
    float integrative = pidSumCTE * factorI;

    pidRetValue = proportional + derivative + integrative;



  return pidRetValue;
}


