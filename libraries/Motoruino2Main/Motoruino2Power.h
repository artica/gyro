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

#ifndef _MOTORUINO2POWER_H_
#define _MOTORUINO2POWER_H_


class Motoruino2Power {
public:
	Motoruino2Power();
	virtual ~Motoruino2Power();

	// Wake/Sleep Functions
	bool wake(char config);
	bool sleep(char config);

	bool config_power(char config = 0);

	bool getBatteryValue(unsigned short * Value);

private:


};

#endif /* _MOTORUINO2POWER_H_ */
