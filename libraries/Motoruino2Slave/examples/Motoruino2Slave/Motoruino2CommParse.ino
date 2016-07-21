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

// Update States for "later" reply (to do not block the Wire Interrupt driven Comm)
#define _DEBUG_COMM_ false

void StopMotors() {
	startNavigateToAngle = false;
	digitalWrite(M1_IN1, LOW);
	digitalWrite(M1_IN2, LOW);
	digitalWrite(M2_IN1, LOW);
	digitalWrite(M2_IN2, LOW);
}


void stopNavigation()
{
	startNavigateToAngle = false;
	startNavigateToDistance = false;
	startRotateToAngle = false;
	startReachEncoderAngle = false;
}

// ------------------------------------------------------------------------------
// PARSER: Received an I2C "wire" message from the master - CRC already tested - MUST BE FAST!
// ------------------------------------------------------------------------------
void CommParseCommand(unsigned short howMany) {
	unsigned char rdata = 0;
	readyFlagToReply = false;

	OrderRequest = (enum _ReceivedOrder) receivedBuffer[0];

	// Repeat command to SendBuffer
	sendBuffer[0] = OrderRequest;

	if (_DEBUG_COMM_) {
		Serial.print("CMD:");
		Serial.println((unsigned char) OrderRequest);
	}

	switch (OrderRequest) {
	case RECEIVED_NONE:
		CommReplyLater = NO_REPLY;
		break;

	case STTI:
		CommReplyLater = REPLY_START_IMU;
		break;
	case STPI:
		CommReplyLater = REPLY_STOP_IMU;
		break;

	case CALG:
		StopMotors();
		CommReplyLater = REPLY_CALIBRATE_GYRO;
		break;
	case CALA:
		StopMotors();
		CommReplyLater = REPLY_CALIBRATE_ANGLE;
		break;
	case CALM:
		StopMotors();
		CommReplyLater = REPLY_CALIBRATE_MAG;
		break;

	case RSTG:
		// Interrupts the navigation
		stopNavigation();

		rdata = (unsigned char) getCharData(1) & 0x07;

		// Reset Gyro fields = 0º . Heading, Pitch and Roll
		imuLSM9DS0.resetGyro(rdata & 0x01, rdata & 0x02, rdata & 0x04);
		replyTrue();
		break;

	case GMAH:
		CommReplyLater = REPLY_GET_MHEADING;
		break;
	case GMAP:
		CommReplyLater = REPLY_GET_MPITCH;
		break;
	case GMAR:
		CommReplyLater = REPLY_GET_MROLL;
		break;

	case GGYH:
		CommReplyLater = REPLY_GET_GHEADING;
		break;
	case GGYP:
		CommReplyLater = REPLY_GET_GPITCH;
		break;
	case GGYR:
		CommReplyLater = REPLY_GET_GROLL;
		break;

	case GAVX:
		CommReplyLater = REPLY_GET_ACC_AVERAGEX;
		break;
	case GAVY:
		CommReplyLater = REPLY_GET_ACC_AVERAGEY;
		break;
	case GAVZ:
		CommReplyLater = REPLY_GET_ACC_AVERAGEZ;
		break;

	case GAGX:
		CommReplyLater = REPLY_GET_ANGLEX;
		break;
	case GAGY:
		CommReplyLater = REPLY_GET_ANGLEY;
		break;

	case GACX:
		CommReplyLater = REPLY_GET_GX;
		break;
	case GACY:
		CommReplyLater = REPLY_GET_GY;
		break;
	case GACZ:
		CommReplyLater = REPLY_GET_GZ;
		break;

	case GGYX:
		CommReplyLater = REPLY_GET_GX;
		break;
	case GGYY:
		CommReplyLater = REPLY_GET_GY;
		break;
	case GGYZ:
		CommReplyLater = REPLY_GET_GZ;
		break;

	case GMAX:
		CommReplyLater = REPLY_GET_MX;
		break;
	case GMAY:
		CommReplyLater = REPLY_GET_MY;
		break;
	case GMAZ:
		CommReplyLater = REPLY_GET_MZ;
		break;

	case RSTD:

		stopNavigation();

		// Reset each motor distance
		motor.resetDistance(1);
		motor.resetDistance(2);

		Serial.println("RSTD");

		replyTrue();
		break;

	case GED1:
		CommReplyLater = REPLY_GET_DISTANCE_1;
		break;
	case GED2:
		CommReplyLater = REPLY_GET_DISTANCE_2;
		break;

	case GSP1:
		CommReplyLater = REPLY_GET_SPEED_1;
		break;
	case GSP2:
		CommReplyLater = REPLY_GET_SPEED_2;
		break;

	case SSPR:
		if (motor.isEncoderDefined) {
			motor.setRPMMotorSpeed(getShortData(1), getShortData(3));
			replyTrue();
			break;
		} else
			replyFalse();
		break;


	case TTDG:
		// Interrupts the navigation
		stopNavigation();

		CommReplyLater = REPLY_GOTO_DEGREES;
		break;

	case MTDI:
		// Interrupts the navigation
		stopNavigation();

		if (motor.isEncoderDefined) {
			Serial.println("MTDI");
			CommReplyLater = REPLY_GOTO_DISTANCE;
			break;
		} else
			replyFalse();
		break;

	case RTAN:
		// Interrupts the navigation
		stopNavigation();

		if (motor.isEncoderDefined) {
			Serial.println("RTAN");
			CommReplyLater = REPLY_TURN_TO_DEGREE;
			break;
		} else
			replyFalse();
		break;


	case RSTA:
		motor.angle = 0;
		replyTrue();
		break;

	case SPWM:
		// Interrupts the navigation
		stopNavigation();

		// Get the PWM Value
		motor.pwmIn1M1 = receivedBuffer[1];
		motor.pwmIn2M1 = receivedBuffer[2];
		motor.pwmIn1M2 = receivedBuffer[3];
		motor.pwmIn2M2 = receivedBuffer[4];

		// Set the Motors
		analogWrite(M1_IN1, motor.pwmIn1M1);
		analogWrite(M1_IN2, motor.pwmIn2M1);
		analogWrite(M2_IN1, motor.pwmIn1M2);
		analogWrite(M2_IN2, motor.pwmIn2M2);

		replyTrue();
		break;

	case MMOT:
		// Interrupts the navigation
		stopNavigation();

		motor.setPWMMotorSpeed(getShortData(1), getShortData(3));
		replyTrue();
		break;

	case CFGM:
		motor.config(getCharData(1), getCharData(2));
		replyTrue();
		break;

	case SOUT:
		analogWrite(GP_OUT, getCharData(1));
		replyTrue();
		break;
	case GOUT:
		CommReplyLater = REPLY_GET_OUT;
		break;

	case GBAT:
		CommReplyLater = REPLY_GET_BATTERY;
		break;

	case GENC:
		if (motor.isEncoderDefined)
			replyTrue();
		else
			replyFalse();
		break;

	case STTE:
		CommReplyLater = REPLY_START_ENCODER;
		break;
	case STPE:
		CommReplyLater = REPLY_STOP_ENCODER;
		break;

	case GWUP:
		CommReplyLater = REPLY_GLOBAL_WAKEUP;
		break;
	case GSLP:
		CommReplyLater = REPLY_GLOBAL_SLEEP;
		break;

	case GTMP:
		CommReplyLater = REPLY_GET_TEMPERATURE;
		break;

	case SBKD:
		CommReplyLater = REPLY_SET_BACKUP_DATA;
		break;
	case GBKD:
		CommReplyLater = REPLY_GET_BACKUP_DATA;
		break;

	case GADM:
		CommReplyLater = REPLY_GET_ADC_MOTOR;
		break;
	case SADM:
		CommReplyLater = REPLY_SET_ADC_MOTOR;
		break;
	case STDM:
		CommReplyLater = REPLY_STOP_ADC_MOTOR;
		break;

		// the shit just hit the fan
	default:
		break;

	}

}

