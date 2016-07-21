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

#define SHORT_DATA 	1+2
#define LONG_DATA 	1+4
#define FLOAT_DATA 	1+4

// ------------------------------------------------------------------------------
// Set the flag for TWI/I2C ready to reply
// ------------------------------------------------------------------------------
// ------------ Prepare the Reply with data --------------
void ReplyData(unsigned char bufferSize) {
	CommReplyLater = NO_REPLY;
	sendLength = bufferSize;					// Buffer Size
	readyFlagToReply = true;
}
// ------------ Prepare the Reply as true --------------
void replyTrue(void) {
	sendBuffer[1] = TRUE;
	sendLength = 1 + 1;				// Buffer Size = 1 (cmd) + 1 (data = TRUE)
	CommReplyLater = NO_REPLY;
	readyFlagToReply = true;
}
// ------------ Prepare the Reply as false --------------
void replyFalse(void) {
	sendBuffer[1] = FALSE;
	sendLength = 1 + 1;				// Buffer Size = (cmd) + 1 (data = FALSE)
	CommReplyLater = NO_REPLY;
	readyFlagToReply = true;
}

// ------------------------------------------------------------------------------
// Pooling event to REPLY ORDERS updated - can be interrupted
// ------------------------------------------------------------------------------
void UpdateComm() {
	// REPLY TO MASTER (the received "Wire" Orders)
	switch (CommReplyLater) {
	// Do nothing
	case NO_REPLY:
		break;

	case REPLY_START_IMU:
		// TODO: Config Wake config - Indent Start of Accelerometer/Gyro and Magneto
		imuLSM9DS0.startFlagIMU = true;
		replyTrue();
		break;

	case REPLY_STOP_IMU:
		// TODO: Config Sleep config
		imuLSM9DS0.startFlagIMU = false;
		replyTrue();
		break;

		// Wait for calibration to end
	case REPLY_CALIBRATE_GYRO:

		if (imuLSM9DS0.calibrateGyro()) {		// Test if Calibration has ended
			replyTrue();
			Serial.println("CALIB G OK");
		} else {
			replyFalse();
			Serial.println("CALIB G NOK");
		}
		break;

	case REPLY_CALIBRATE_ANGLE:
		if (imuLSM9DS0.calibrateAngle()) {		// Test if Calibration has ended
			replyTrue();
			Serial.println("CALIB A OK");
		} else {
			replyFalse();
			Serial.println("CALIB A NOK");
		}
		break;

	case REPLY_GET_BATTERY:
		FillShortData(power.getBattery());
		ReplyData(SHORT_DATA);
		break;

	case REPLY_GET_MX:
		FillShortData(imuLSM9DS0.getMagnetometerX());
		ReplyData(SHORT_DATA);
		break;
	case REPLY_GET_MY:
		FillShortData(imuLSM9DS0.getMagnetometerY());
		ReplyData(SHORT_DATA);
		break;
	case REPLY_GET_MZ:
		FillShortData(imuLSM9DS0.getMagnetometerZ());
		ReplyData(SHORT_DATA);
		break;

	case REPLY_GET_GX:
		FillShortData(imuLSM9DS0.getGyroscopeX());
		ReplyData(SHORT_DATA);
		break;
	case REPLY_GET_GY:
		FillShortData(imuLSM9DS0.getGyroscopeY());
		ReplyData(SHORT_DATA);
		break;
	case REPLY_GET_GZ:
		FillShortData(imuLSM9DS0.getGyroscopeZ());
		ReplyData(SHORT_DATA);
		break;

	case REPLY_GET_AX:
		FillShortData(imuLSM9DS0.getAccelerometerX());
		ReplyData(SHORT_DATA);
		break;
	case REPLY_GET_AY:
		FillShortData(imuLSM9DS0.getAccelerometerY());
		ReplyData(SHORT_DATA);
		break;
	case REPLY_GET_AZ:
		FillShortData(imuLSM9DS0.getAccelerometerZ());
		ReplyData(SHORT_DATA);
		break;

		// Reply the Gyroscope
	case REPLY_GET_GHEADING:
		FillShortData(imuLSM9DS0.getGyroHeading());
		ReplyData(SHORT_DATA);
		break;
	case REPLY_GET_GPITCH:
		FillShortData(imuLSM9DS0.GetGyroPitch());
		ReplyData(SHORT_DATA);
		break;
	case REPLY_GET_GROLL:
		FillShortData(imuLSM9DS0.getGyroRoll());
		ReplyData(SHORT_DATA);
		break;

		// Reply the Magnetometer
	case REPLY_GET_MHEADING:
		FillFloatData(imuLSM9DS0.getMagHeading());
		ReplyData(FLOAT_DATA);
		break;
	case REPLY_GET_MPITCH:
		FillFloatData(imuLSM9DS0.getMagPitch());
		ReplyData(FLOAT_DATA);
		break;
	case REPLY_GET_MROLL:
		FillFloatData(imuLSM9DS0.getMagRoll());
		ReplyData(FLOAT_DATA);
		break;

		// Reply the Accelerometer
	case REPLY_GET_ACC_AVERAGEX:
		FillFloatData(imuLSM9DS0.getAccelAvgX());
		ReplyData(FLOAT_DATA);
		break;
	case REPLY_GET_ACC_AVERAGEY:
		FillFloatData(imuLSM9DS0.getAccelAvgY());
		ReplyData(FLOAT_DATA);
		break;
	case REPLY_GET_ACC_AVERAGEZ:
		FillFloatData(imuLSM9DS0.getAccelAvgZ());
		ReplyData(FLOAT_DATA);
		break;

		// Get Distance
	case REPLY_GET_DISTANCE_1:
		FillLongData(motor.getDistance(1));
		ReplyData(LONG_DATA);
		break;
	case REPLY_GET_DISTANCE_2:
		FillLongData(motor.getDistance(2));
		ReplyData(LONG_DATA);
		break;

		// Get Angle
	case REPLY_GET_ANGLEX:
		FillFloatData(imuLSM9DS0.getAngleX());
		ReplyData(FLOAT_DATA);
		break;
	case REPLY_GET_ANGLEY:
		FillFloatData(imuLSM9DS0.getAngleY());
		ReplyData(FLOAT_DATA);
		break;

		// Already Implemented:
		//   The main loop() will keep the robot chasing the defined angle
		// - Only runs one time to configure and ACK to Master
		// - If the master wants to know if it reached, it must query again
	case REPLY_GOTO_DEGREES:

		goalAngle = getShortData(1); // Get the Angle From the Buffer (cmd @ position 0 -> Short @ position 1)
		goalSpeed = getShortData(3);// Get the Angle From the Buffer (Char @ position 3 -> 1 + 2)

		motor.pwmGoalSpeed = goalSpeed;

		// Get the Speed and Runs ReachAngle Navigation Function
		if (ReachAngle((long) goalAngle, goalSpeed)) {
			// Sets the ReplyBuffer
			replyTrue();
			startNavigateToAngle = false;
		} else {
			// Sets the ReplyBuffer - Starts Navigate!
			replyFalse();
			startNavigateToAngle = true;
		}
		break;

		// Later in loop()... will keep the robot chasing the defined Distance - Only runs "one time" to configure and ACK to Master
	case REPLY_GOTO_DISTANCE:

		goalDistance = getLongData(1);		// Get the Distance From the Buffer
		goalSpeed = getShortData(5); 		// Get the Speed From Buffer

		motor.pwmGoalSpeed = goalSpeed;

		// Get the Speed and Runs ReachDistance Navigation Function
		if (ReachDistance((long) goalDistance, motor.pwmGoalSpeed)) {
			// Sets the ReplyBuffer
			replyTrue();
			startNavigateToDistance = false;
		} else {
			// Sets the ReplyBuffer
			replyFalse();
			startNavigateToDistance = true;
		}
		break;

		//
	case REPLY_TURN_TO_DEGREE:

		motor.angle = getFloatData(1);		// Get the Distance From the Buffer
		motor.angleLeftMotorPwm = getShortData(5); 		// Get the Speed From Buffer
		motor.angleRightMotorPwm = getShortData(7); 		// Get the Speed From Buffer

		// Get the Speed and Runs ReachDistance Navigation Function
		if (ReachEncoderAngle(motor.angle, motor.angleLeftMotorPwm, motor.angleRightMotorPwm)) {
			// Sets the ReplyBuffer
			replyTrue();
			startReachEncoderAngle = false;
		} else {
			// Sets the ReplyBuffer
			replyFalse();
			startReachEncoderAngle = true;
		}
		break;


	case REPLY_SET_ADC_MOTOR:
		motor.setADCTrigger(getShortData(1));		// Set the Trigger
		motor.startAdcTrigger();
		replyTrue();
		break;

	case REPLY_GET_ADC_MOTOR:
		FillShortData(motor.getADC());
		ReplyData(SHORT_DATA);
		break;

	case REPLY_STOP_ADC_MOTOR:
		motor.stopAdcTrigger();
		replyTrue();
		break;

	case REPLY_GET_OUT:
		if (GP_OUT == HIGH)
			replyTrue();
		else
			replyFalse();
		break;

	case REPLY_START_ENCODER:

		mode = (Motoruino2SMotor::EncoderMode) getCharData(1);
		DataShortAux = (unsigned short) getLongData(2);
		DataFloatAux = getFloatData(6);

		motor.startEncoders(mode, DataShortAux, DataFloatAux);

		replyTrue();
		break;


		// the shit just hit the fan
	default:
		Serial.print("Dahm! Comm:"); Serial.println(CommReplyLater);
		break;
	};
}

// BUFFER AUX FUNCTIONS
char getCharData(unsigned char index) {
	return receivedBuffer[index];
}

//
//bool Motoruino2Comm::sendDataReplyShort(enum _ReceivedOrder cmd, short *replyData)
//{
//	char * ShortData;
//	commBuff[0] = (char) cmd;
//
//	ShortData = (char*) replyData;
//
//	*replyData = 0;
//
//	// Send the Command - one byte, do not wait and receive the command plus data
//	if (!sendData(commBuff,  0, commBuff, CMD_SIZE + 2)) return false;
//
//	// Validate and process the received message - Heading
//	if ((char) cmd != commBuff[0]) return false;
//
//
//	for (int i = CMD_SIZE; i < (CMD_SIZE + 2); i++)
//	{
//		ShortData[i-CMD_SIZE] = commBuff[i];
//	}
//
//	return true;
//}

//long *Data = (long*) &reply;
//
//for (unsigned char i = index; i < index + 4; i++)
//{
//	*Data = *Data | (receivedBuffer[i]  << ((i-index)*8));
//}
//return reply;

//short getShortData(unsigned char index)
//{
//	short Data;
//
//
//	for (unsigned char i = index; i < index + 2; i++)
//	{
//		Data = Data | (receivedBuffer[i] << ((i-index)*8));
//	}
//	return Data;
//}

short getShortData(unsigned char index) {
	short Data = 0;
	char *Padding = (char*) &Data;

	for (unsigned char i = index; i < index + 2; i++) {
		Padding[i - index] = ((char) receivedBuffer[i]);

	}

	return Data;
}

long getLongData(unsigned char index) {
	long Data = 0;
	char *Padding = (char*) &Data;

	for (unsigned char i = index; i < index + 4; i++) {
		Padding[i - index] = ((char) receivedBuffer[i]);

	}
	return Data;
}

float getFloatData(unsigned char index) {
	float reply = 0;

	char *Padding = (char*) &reply;

	for (unsigned char i = index; i < index + 4; i++) {
		Padding[i - index] = ((char) receivedBuffer[i]);

	}

	return reply;
}

void FillLongData(long ReplyData) {
	for (int x = 0; x < 4; x++)			// Fills the Buffer ... MSB goes first
			{
		sendBuffer[1 + x] = (0x000000FF & ReplyData);
		ReplyData = ReplyData >> 8;
	}

}

void FillShortData(short ReplyData) {
	for (int x = 0; x < 2; x++)			// Fills the Buffer ... MSB goes first
			{
		sendBuffer[1 + x] = (0x000000FF & ReplyData);
		ReplyData = ReplyData >> 8;
	}

}

void FillFloatData(float Data) {

	char *ReplyData = (char*) &Data;

	for (int x = 0; x < 4; x++)			// Fills the Buffer ... MSB goes first
			{
		sendBuffer[1 + x] = ReplyData[x];
	}

}

