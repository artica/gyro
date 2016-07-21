/*
 Artica CC - http://artica.cc

 Gyro Bumper "Master Wire" Lib
 - Communicates with I2C/Wire with the Bumper Gyro PCB
 - Uses 2 APDS-9960 Sensors on Front (Gyro eyes - Left and Right)
 - Uses 1 APDS-9960 Sensor on bottom (Center Bottom Sensor)
 - Uses 2 Line Sensors (Left and Right) QRE1113-Analog from Sparkfun
 - Have one RGB Led on bottom

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#include "GyroBumper.h"
#include "Motoruino2Comm.h"

GyroBumper::GyroBumper() {
	_initialized = false;
}

// Configure the pins for the front bumpers
void GyroBumper::begin(	_BumperConfig lconfig,
						_BumperConfig cconfig,
						_BumperConfig rconfig ) {

	unsigned short configValue = 0;

	OrderRequestBumper = RX_NONE;
	CommReplyLaterBumper = NO_RX;
	GestureState = NONE;


	//	add Left Nible
	configValue = (lconfig & 0x0F);

	//	add Center Nible
	configValue = (configValue << 4) | (cconfig & 0x0F);

	//	add Right Nible
	configValue = (configValue << 4) | (rconfig & 0x0F);

	Serial.print("CONFIG:");
	Serial.println(configValue, HEX);

	delay(2000);

	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	motoruino2Comm.addShortData(configValue);

	//char * ReceivedData = (char*) &buff;
	motoruino2Comm.commBuff[0] = (char) CFG;

	// Send the Command - one byte, do not wait and receive the command plus true or false
	motoruino2Comm.sendData(motoruino2Comm.commBuff,  70, motoruino2Comm.commBuff, CMD_SIZE + 1);

	_initialized = true;
}

void GyroBumper::setTriggerBumper(unsigned char trigg) {
	trigger = trigg;
}

// ---------------------------------------------------------------------
// Bumper sensor methods

// ---------------- LEFT --------------------------------------------------------

bool GyroBumper::CheckLeftBumper() {
	if (!_initialized) return false;
	unsigned short value = 0;
	getLeftProximity(&value);
	if (value > trigger)
		return true;
	else
		return false;
}

bool GyroBumper::getLeftProximity(unsigned short * proximity) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GLPROXIMITY, (short *) proximity);
}

bool GyroBumper::getLeftGesture(_GestureState * gestureState) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GLGESTURE, (short *) gestureState);
}

bool GyroBumper::getLeftRed(unsigned short * red) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GLRED, (short *) red);
}
bool GyroBumper::getLeftGreen(unsigned short * green) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GLGREEN, (short *) green);
}
bool GyroBumper::getLeftBlue(unsigned short * blue) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GLBLUE, (short *) blue);
}
bool GyroBumper::getLeftAmbient(unsigned short * ambient) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GLAMBIENT, (short *) ambient);
}

bool GyroBumper::getLeftLine(unsigned short * line) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GLLINE, (short *) line);
}

bool GyroBumper::setLeftLED(unsigned char value) {
	if (!_initialized) return false;
	// Prepare the Params
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);

	motoruino2Comm.addCharData((char) value);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(SLLED);
}

// ---------------- RIGHT --------------------------------------------------------
bool GyroBumper::CheckRightBumper() {
	if (!_initialized) return false;
	unsigned short value = 0;
	getRightProximity(&value);
	if (value > trigger)
		return true;
	else
		return false;
}

bool GyroBumper::getRightProximity(unsigned short * proximity) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GRPROXIMITY, (short *) proximity);
}

bool GyroBumper::getRightGesture(_GestureState * gestureState) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GRGESTURE, (short *) gestureState);
}

bool GyroBumper::getRightRed(unsigned short * red) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GRRED, (short *) red);
}
bool GyroBumper::getRightGreen(unsigned short * green) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GRGREEN, (short *) green);
}
bool GyroBumper::getRightBlue(unsigned short * blue) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GRBLUE, (short *) blue);
}
bool GyroBumper::getRightAmbient(unsigned short * ambient) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GRAMBIENT, (short *) ambient);
}

bool GyroBumper::getRightLine(unsigned short * line) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GRLINE, (short *) line);
}

bool GyroBumper::setRightLED(unsigned char value) {
	if (!_initialized) return false;
	// Prepare the Params
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);

	motoruino2Comm.addCharData((char) value);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(SRLED);
}

// ---------------- CENTER --------------------------------------------------------
bool GyroBumper::getCenterProximity(unsigned short * proximity) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GCPROXIMITY, (short *) proximity);
}

bool GyroBumper::getCenterGesture(_GestureState * gestureState) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GCGESTURE, (short *) gestureState);
}

bool GyroBumper::getCenterRed(unsigned short * red) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GCRED, (short *) red);
}
bool GyroBumper::getCenterGreen(unsigned short * green) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GCGREEN, (short *) green);
}
bool GyroBumper::getCenterBlue(unsigned short * blue) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GCBLUE, (short *) blue);
}
bool GyroBumper::getCenterAmbient(unsigned short * ambient) {
	if (!_initialized) return false;
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	return motoruino2Comm.sendDataReplyShort(GCAMBIENT, (short *) ambient);
}

bool GyroBumper::setCenterLED(unsigned char value) {
	if (!_initialized) return false;
	// Prepare the Params
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);
	motoruino2Comm.addCharData((char) value);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(SCLED);
}

bool GyroBumper::setRGB(unsigned char red, unsigned char green,
		unsigned char blue) {
	if (!_initialized) return false;
	// Prepare the Params
	motoruino2Comm.startNewComm(BUMPER_ADDRESS);

	motoruino2Comm.addCharData((char) red);
	motoruino2Comm.addCharData((char) green);
	motoruino2Comm.addCharData((char) blue);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(SRGB);
}

