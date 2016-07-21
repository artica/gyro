/*
 Artica CC - http://artica.cc

 MotoruinoSlave lib

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

#include "Wire.h"

static void InterruptReceiveI2C(int howMany);
static void InterruptRequestI2C();

// ------------------------------------------------------------------------------
// Start the I2C communication
// ------------------------------------------------------------------------------
void commConfig() {
	CommReplyLater = NO_REPLY;

	Wire.begin(WIRE_ADDRESS);           	// join i2c bus with address #4
	Wire.onReceive(InterruptReceiveI2C); 	// register Call Back Event/Function
	Wire.onRequest(InterruptRequestI2C); 	// register Call Back Event/Function
}

// ------------------------------------------------------------------------------
// ISR ROUTINE - Callback - Received I2C from Master Atmel32u4
// ------------------------------------------------------------------------------
static void InterruptReceiveI2C(int howMany) {
	unsigned char ReceivedDataPointer = 0;

	// if received more than it can handle returns
	if (howMany > BUFFER_SIZE)
		return;

	// Flush the Buffer from wire
	while (0 < Wire.available())
		receivedBuffer[ReceivedDataPointer++] = Wire.read();

	// PARSER to launch launch the order
	CommParseCommand(howMany);
}

// ------------------------------------------------------------------------------
// ISR ROUTINE - Callback - Will Send I2C to Master Atmel32u4
// ------------------------------------------------------------------------------
static void InterruptRequestI2C() {
	if (readyFlagToReply) {
		Wire.write(sendBuffer, sendLength);
	} else {
		Serial.println("X");
		Wire.write("NO DATA");
	}

}

// ------------------------------------------------------------------------------
// Aux function fast string comparison
// ------------------------------------------------------------------------------
bool are_equal(char *s1, char *s2, unsigned char start, unsigned char size) {
	for (int i = size; i--;) {
		if (s1[i + start] != s2[i + start])
			return false;
		// They are different
	}
	return true;  // They are be the same
}

