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

#ifndef _MOTORUINO2COMM_H_
#define _MOTORUINO2COMM_H_

#include "Arduino.h"
#include "Motoruino2Protocol.h"

#define COMM_BUFF_SIZE 10

class Motoruino2Comm
{
	public:
		// ---------------------------------------------------------------------
		// Initialization and configuration
		// Default constructor
		Motoruino2Comm();
		void commConfig();

		// ---------------------------------------------------------------------
		// Called ALLWAYS before any send
		void startNewComm(unsigned char address);

		// Help Add Data after command to Send to slave
		void addCharData(char reply);
		void addShortData(short data);
		void addLongData(long data);
		void addFloatData(float data);

		// Send data
		bool sendDataReplyBool(char cmd);
		bool sendDataReplyChar(char cmd, char *replyData);
		bool sendDataReplyLong(char cmd, long *replyData);
		bool sendDataReplyShort(char cmd, short *replyData);
		bool sendDataReplyFloat(char cmd, float *replyData);

		//void copyBytes(char* Buffer, char *s1, char count);

		// Sends and Receive Data to slave
		bool sendData(char* sendArray, unsigned short replyDelay, char* replyArray, char ReplySize);

		enum _ReceivedOrder OrderRequest;
		enum _ReplyNowOrder CommReplyNow ;
		enum _ProcessLaterOrders CommReplyLater;

		char commBuff[COMM_BUFF_SIZE];		// Send/Receive Buffer

	private:

		unsigned char address;

		char buffIndex;						// index pointer for Data

		bool _initialized;

		// ---------------------------------------------------------------------
		// Test Response from I2C
		bool hasResponse();
		// ---------------------------------------------------------------------
		// Gets Response from I2C
		char* getResponse();
};

extern Motoruino2Comm motoruino2Comm;
#endif
