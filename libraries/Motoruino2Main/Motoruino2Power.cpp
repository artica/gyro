/*
 Artica CC - http://artica.cc

 Motoruino 2 Board Power Slave Interface Lib

 V0.0 - Serras and Tarquinio - 23/11/2015
 	 Board Power Slave Interface Lib

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#include "Motoruino2Power.h"
#include "Motoruino2Comm.h"
#include "Motoruino2.h"


// Constructor
Motoruino2Power::Motoruino2Power() {


}

//Destructor
Motoruino2Power::~Motoruino2Power() {

}

bool Motoruino2Power::config_power(char config)
{
	// TODO: Configurations
	return true;
}

bool Motoruino2Power::wake(char config)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addCharData( (char) config);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(GWUP);
}
bool Motoruino2Power::sleep(char config)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addCharData( (char) config);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(GSLP);
}

// Get the Battery Voltage Value
bool Motoruino2Power::getBatteryValue(unsigned short * Value)
{

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyShort(GBAT, (short*) Value);
}
