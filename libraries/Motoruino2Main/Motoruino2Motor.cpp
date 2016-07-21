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


#include "Motoruino2Motor.h"
#include "Motoruino2Comm.h"
#include "Motoruino2.h"

// ---------------------------------------------------------------------
// Initialization and configuration

// Default constructor, initializes the motors in the default pins (3,5,6,11) 
Motoruino2Motor::Motoruino2Motor()
{
	// Flag revealing that all configurations have ended - Last thing to do here
	dir1 = 1;
	dir2 = 1;

	_initialized = true;
}

bool Motoruino2Motor::config_motor(char _dir1, char _dir2)
{
	dir1 = _dir1;
	dir2 = _dir2;

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addCharData( (char) dir1);
	motoruino2Comm.addCharData( (char) dir2);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(CFGM);

}

// Get the Current Value
bool Motoruino2Motor::getCurrentValue(unsigned short * Value)
{

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyShort(GADM, (short*) Value);
}

// Start the Trigger
bool Motoruino2Motor::startMotorTrigger(unsigned short Value)
{

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addShortData( Value );

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(SADM);
}

// Stop the Trigger
bool Motoruino2Motor::stopMotorTrigger()
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(STDM);
}

// ResetDistance
bool Motoruino2Motor::resetDistance()
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(RSTD);
}

// ResetAngle
bool Motoruino2Motor::resetAngle()
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(RSTA);
}

// Set speed in rpm
bool Motoruino2Motor::setSpeedRPM(short speedLeft, short speedRight)
{
	RPMLeft = speedLeft;
	RPMRight = speedLeft;


	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);
	motoruino2Comm.addShortData(speedLeft);
	motoruino2Comm.addShortData(speedRight);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(SSPR);
}

bool Motoruino2Motor::setSpeedRPMLeft(short speedLeft)
{
	return setSpeedRPM(speedLeft, RPMRight);


}
bool Motoruino2Motor::setSpeedRPMRight( short speedRight)
{
	return setSpeedRPM(RPMLeft, speedRight);
}


//Receive each motor velocity as argument
bool Motoruino2Motor::setSpeedPWM(short speedLeft, short speedRight)
{

	PWMLeft = speedLeft;
	PWMRight = speedRight;

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addShortData( (short) speedLeft);
	motoruino2Comm.addShortData( (short) speedRight);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(MMOT);
}

bool Motoruino2Motor::setSpeedPWMLeft( short speedLeft)
{

	return setSpeedPWM(speedLeft, PWMRight);


}
bool Motoruino2Motor::setSpeedPWMRight( short speedRight)
{

	return setSpeedPWM(PWMLeft, speedRight);
}




//Set the PWM of the H Bridge independently
bool Motoruino2Motor::pwmSet(char IN1_M1, char IN2_M1, char IN1_M2, char IN2_M2)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addCharData( (char) IN1_M1);
	motoruino2Comm.addCharData( (char) IN2_M1);
	motoruino2Comm.addCharData( (char) IN1_M2);
	motoruino2Comm.addCharData( (char) IN2_M2);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(SPWM);
}

bool Motoruino2Motor::isEncoderSet()
{

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(GENC);

}
bool Motoruino2Motor::startEncoder(enum EncoderMode mode, long ticksPerTurn, float diameter)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addCharData( (char) mode);
	motoruino2Comm.addLongData( ticksPerTurn);
	motoruino2Comm.addFloatData(  diameter);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(STTE);
}
bool Motoruino2Motor::stopEncoder()
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(STPE);
}



