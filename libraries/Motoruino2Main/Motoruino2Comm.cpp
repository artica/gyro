/*
 Artica CC - http://artica.cc

 Motoruino 2 Communication Libs with Slave micro-controller

 V0.0 - Serras and Tarquinio - 23/11/2015
 	 Communication Lib to interface with the slave micro-controller

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */


#include "Wire.h"
#include "Motoruino2Comm.h"

Motoruino2Comm motoruino2Comm;

// Default constructor, initializes the motors in the default pins (3,5,6,11)
Motoruino2Comm::Motoruino2Comm()
{
	_initialized = false;
}

void Motoruino2Comm::commConfig()
{
	if (_initialized) return;

	Wire.begin();

	buffIndex = 1;		// Take Command into account
	commBuff[0] = ((char)RECEIVED_NONE);	// Start with NONE CMD
	commBuff[1] = 0;

	OrderRequest = RECEIVED_NONE;
	CommReplyNow = REPLY_NONE;
	CommReplyLater= NO_REPLY;

	// Flag revealing that all configurations have ended - Last thing to do here
	_initialized = true;
}



// ---------------------------------------------------------------------
// SendFunction
// ---------------------------------------------------------------------
bool Motoruino2Comm::sendData(char* sendArray,  unsigned short replyDelay, char* replyArray, char ReplySize)
{
	int i = 0;
	long start = 0;

	if (!_initialized) return false;

	Wire.beginTransmission(address);


	for (int x = 0; x < buffIndex; x++ )
	{
		Wire.write(sendArray[x]);
	}

	Wire.endTransmission();    // stop transmitting

	delay(5);

	// request ReplySize bytes from slave device #SLAVE_ADD
	if (ReplySize){

		delay(replyDelay);

		Wire.requestFrom((short)address, (int) ReplySize);

		// Wait for Reply
		start = millis();

		while (!Wire.available() && (millis() - start < 10));

		//Serial.println((short) Wire.available());

		while (Wire.available() && (millis() - start < 10000))    // slave may send less than requested
		{
			replyArray[i++] = Wire.read(); // receive a char as character
		}

		if (millis() - start > 10000) return false;

	}

	return true;
}

//--------------------------------------------------------------
// AUX FUNCTIONS ... TODO: TEST FLOAT VALUE
//--------------------------------------------------------------



bool Motoruino2Comm::sendDataReplyBool(char cmd)
{

	//char * ReceivedData = (char*) &buff;
	commBuff[0] = (char) cmd;

	// Send the Command - one byte, do not wait and receive the command plus true or false
	if (!sendData(commBuff,  0, commBuff, CMD_SIZE + 1)) return false;

	// Validate the received message
	if ((char)cmd != commBuff[0]) return false;
	if ((enum _ReplyNowOrder) commBuff[1] != TRUE) return false;

	return true;
}

bool Motoruino2Comm::sendDataReplyChar(char cmd, char *replyData)
{
	commBuff[0] = (char) cmd;
	*replyData = 0;

	// Send the Command - one byte, do not wait and receive the command plus data
	if (!sendData(commBuff,  0, commBuff, CMD_SIZE + 1)) return false;

	// Validate and process the received message - Heading
	if ((char)cmd != commBuff[0]) return false;

	*replyData = commBuff[1];

	return true;
}

bool Motoruino2Comm::sendDataReplyLong(char cmd, long *replyData)
{

	char * LongData;
	commBuff[0] = (char) cmd;

	LongData = (char*) replyData;

	*replyData = 0;

	// Send the Command - one byte, do not wait and receive the command plus data
	if (!sendData(commBuff,  0, commBuff, CMD_SIZE + 4)) return false;

	// Validate and process the received message - Heading
	if ((char) cmd != commBuff[0]) return false;


	for (int i = CMD_SIZE; i < (CMD_SIZE + 4); i++)
	{
		LongData[i-CMD_SIZE] = commBuff[i];
	}

	return true;
}

bool Motoruino2Comm::sendDataReplyShort(char cmd, short *replyData)
{
	char * ShortData;
	commBuff[0] = (char) cmd;

	ShortData = (char*) replyData;

	*replyData = 0;

	// Send the Command - one byte, do not wait and receive the command plus data
	if (!sendData(commBuff,  2, commBuff, CMD_SIZE + 2)) return false;

	// Validate and process the received message - Heading
	if ((char) cmd != commBuff[0]) return false;


	for (int i = CMD_SIZE; i < (CMD_SIZE + 2); i++)
	{
		ShortData[i-CMD_SIZE] = commBuff[i];
	}

	return true;

}

bool Motoruino2Comm::sendDataReplyFloat(char cmd, float *replyData)
{

	char * FloatData;
	commBuff[0] = (char) cmd;

	FloatData = (char*) replyData;

	*replyData = 0;

	// Send the Command - one byte, do not wait and receive the command plus data
	if (!sendData(commBuff,  0, commBuff, CMD_SIZE + 4)) return false;

	// Validate and process the received message - Heading
	if ((char) cmd != commBuff[0]) return false;


	for (int i = CMD_SIZE; i < (CMD_SIZE + 4); i++)
	{
		FloatData[i-CMD_SIZE] = commBuff[i];
	}

	return true;
}

//--------------------------------------------------------------
// Help Add Data after command to Send to slave

void Motoruino2Comm::startNewComm(unsigned char add)
{
	address = add;

	buffIndex = 1;		// Take Command into account
	commBuff[0] = ((char)RECEIVED_NONE);	// Start with NONE CMD
	commBuff[1] = 0;
}

void Motoruino2Comm::addCharData(char reply)
{
	commBuff[buffIndex] = reply;

	buffIndex = buffIndex + 1;
}


void Motoruino2Comm::addShortData(short data)
{
	for (unsigned char i = buffIndex; i < buffIndex + 2; i++)
		commBuff[i] = 0x00FF & (data >> ((i-buffIndex)*8));

	buffIndex = buffIndex + 2;
}

void Motoruino2Comm::addLongData(long data)
{
	for (unsigned char i = buffIndex; i < buffIndex + 4; i++)
		commBuff[i] = 0x000000FF & (data >> ((i-buffIndex)*8));

	buffIndex = buffIndex + 4;
}

void Motoruino2Comm::addFloatData(float data)
{
	char * FloatData;
	FloatData = ( char* ) &data;

	for (unsigned char i = buffIndex; i < buffIndex + 4; i++)
		commBuff[i] = FloatData[i-buffIndex];

	buffIndex = buffIndex + 4;

}


//void Motoruino2Comm::copyBytes(char* Buffer, char *s1, char count)
//{
//	for (unsigned char i = count; i--; ) Buffer[i] = s1[i];
//}

