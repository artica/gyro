/*
 Artica CC - http://artica.cc

 MotoruinoSlave Protocol Commands (same as in Master - Motoruino2Protocol.h)

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


#ifndef _MOTORUINO2BPROTOCOL_H_
#define _MOTORUINO2BPROTOCOL_H_

// --------------------------------------------------------------------------------
// PROTOCOL Commands Received - I2C/TWI - Interrupt Managed (decoded on Parser function)
enum _ReceivedOrder{
	RECEIVED_NONE,											// DEFAULT....
	STTI, STPI, 											// Start Stop IMU ...
	CALG, CALA, CALM,										// Calibrate ...
	RSTG, 													// Reset Gyro
	GMAH, GMAP, GMAR, GGYH, GGYP, GGYR, GAVX, GAVY, GAVZ, GAGX, GAGY,	// Get Calculated Values
	GACX, GACY, GACZ, GGYX, GGYY, GGYZ, GMAX, GMAY, GMAZ,	// Get Raw Values
	RSTD, GED1, GED2, 										// Reset and Get Distance
	GSP1, GSP2,	SSPR, 										// Get and Set Speed
	TTDG, MTDI, SPWM,										// Turn to, Move to Distance, Set PWM
	RSTA,													// Reset Angle
	RTAN,													// Rotate to angle
	MMOT,													// Move Motors (-255 -> 255 Left and Right)
	CFGM,													// Config Motor direction
	SOUT, GOUT, GBAT,										// Set and Get Output, Get Batt
	GENC, STTE, STPE,										// GET Encoder, Start Stop encoder
	IWUP, ISLP,												// Config IMU Wake-up and Sleep
	GWUP, GSLP,												// Config Global Wake-up and Sleep
	GTMP,													// Get Temperature
	SBKD, GBKD,												// Set BackupData and Get BakupData
	GADM, SADM, STDM,									    // Set AD Motor, Get AD Motor, Stop AD Motor
	COLOR, LIGHT, DISTANCE, LINE
};


enum _ReceivedOrder OrderRequest = RECEIVED_NONE;


// --------------------------------------------------------------------------------
// Update States for "Instant" reply (simple reply messages)
enum _ReplyNowOrder{
	REPLY_NONE, TRUE, FALSE, GENERAL_FAIL
};
enum _ReplyNowOrder CommReplyNow = REPLY_NONE;


// --------------------------------------------------------------------------------
// Update States for "later" reply (to do not block the Wire Interrupt for too much time)
enum _ProcessLaterOrders {
	NO_REPLY ,
	REPLY_START_IMU, REPLY_STOP_IMU,
	REPLY_CALIBRATE_GYRO, REPLY_CALIBRATE_ANGLE, REPLY_CALIBRATE_MAG,
	REPLY_GET_MHEADING, REPLY_GET_MPITCH, REPLY_GET_MROLL,
	REPLY_GET_GHEADING, REPLY_GET_GPITCH, REPLY_GET_GROLL,
	REPLY_GET_ACC_AVERAGEX, REPLY_GET_ACC_AVERAGEY, REPLY_GET_ACC_AVERAGEZ,
	REPLY_GET_ANGLEX,REPLY_GET_ANGLEY,
	REPLY_GET_AX, REPLY_GET_AY, REPLY_GET_AZ,
	REPLY_GET_GX, REPLY_GET_GY, REPLY_GET_GZ,
	REPLY_GET_MX, REPLY_GET_MY, REPLY_GET_MZ,
	REPLY_GET_DISTANCE_1,REPLY_GET_DISTANCE_2,
	REPLY_GET_SPEED_1,REPLY_GET_SPEED_2,
	REPLY_GOTO_DEGREES, REPLY_GOTO_DISTANCE,		// ---
	REPLY_TURN_TO_DEGREE,
	REPLY_GET_OUT,
	REPLY_GET_BATTERY,
	REPLY_START_ENCODER, REPLY_STOP_ENCODER,
	REPLY_GLOBAL_WAKEUP, REPLY_GLOBAL_SLEEP,
	REPLY_GET_TEMPERATURE,
	REPLY_SET_BACKUP_DATA,	REPLY_GET_BACKUP_DATA,
	REPLY_GET_ADC_MOTOR, REPLY_SET_ADC_MOTOR, REPLY_STOP_ADC_MOTOR
};
enum _ProcessLaterOrders CommReplyLater= NO_REPLY;





#endif /* _MOTORUINO2PROTOCOL_H_ */
