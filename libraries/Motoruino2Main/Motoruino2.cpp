/*
 Artica CC - http://artica.cc

 Motoruino 2 Main Libs

 V0.0 - Serras and Tarquinio - 23/11/2015
 	 Extend Motoruino 2 Board to included sensors

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#include "Motoruino2Comm.h"
#include "Motoruino2.h"

Motoruino2::Motoruino2() : Motoruino2IMU(), Motoruino2Motor(), Motoruino2Power(){


}

// Moves forward while turning on a specific angle, in degrees
// With angle = 0 degrees the movement is straight forward
bool Motoruino2::moveForwardArc (int angularSpeed, unsigned int distance)
{
	// @TODO fazer isto no micro secundario
	// Fazer cenas fixes aqui! :P

	return false;
}

bool Motoruino2::turnToDegree(short angle, short speed)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addShortData( angle);		// Angle
	motoruino2Comm.addShortData( speed);		// Speed

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(TTDG);
}




// ---------------------------------------------------------------------
// Movement methods
// Stops the movement completely
void Motoruino2::stopMove()
{
	// @TODO fazer isto no micro secundario
	setSpeedPWM(0, 0);
}

// go forward
bool Motoruino2::moveToDistance(signed long distance, unsigned short speed)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addLongData( (signed long) distance);		// Distance
	motoruino2Comm.addShortData( (unsigned short) speed);


	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(MTDI);
}


//
bool Motoruino2::reachEncoderAngle(float angle, short speedl, short speedr)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addFloatData( (float) angle);
	motoruino2Comm.addShortData( (short) speedl);
	motoruino2Comm.addShortData( (short) speedr);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(RTAN);
}


// turn right
bool Motoruino2::turnRight (int angle)
{
	if (angle < -180 || angle > 180) return false;

	// First Reset Gyro
	resetGyro(true, false,false);

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addLongData( angle );		// Angle

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(TTDG);
}

// turn left
bool Motoruino2::turnLeft(int angle)
{
	if (angle < -180 || angle > 180) return false;

	// First Reset Gyro
	resetGyro(true, false,false);

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addLongData( angle * -1);		// Angle

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(TTDG);
}

// -------------------------------------------------------------------------
// Global Functions
bool Motoruino2::saveData(unsigned char * buffer, unsigned char size)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	for (int i = 0; i < size; i++)
		motoruino2Comm.addCharData( buffer[i] );		// Angle

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(SBKD);
}
bool Motoruino2::getData(unsigned char * buffer, unsigned char size)
{
	// Prepare the Parameters
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// set the command
	motoruino2Comm.commBuff[0] = (char) GBKD;

	// Send the Command and expect 2 bytes
	if (!motoruino2Comm.sendData(motoruino2Comm.commBuff, 100, (char*) buffer, 1 + size)) return false;

	// Validate the received message
	if ((enum _ReceivedOrder) buffer[0] != GBKD) return false;

	return true;
}



