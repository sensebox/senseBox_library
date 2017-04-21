#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "SenseBox.h"


//-----Ultraschall Distanz Sensor HC-S04 begin----//
Ultrasonic::Ultrasonic(int rx, int tx)
{
	_rx = rx;
	_tx = tx;
}
/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::getDistance(void)
{
	pinMode(_rx, OUTPUT);
	digitalWrite(_rx, LOW);
	delayMicroseconds(2);
	digitalWrite(_rx, HIGH);
	delayMicroseconds(5);
	digitalWrite(_rx,LOW);
	pinMode(_tx,INPUT);
	long duration;
	duration = pulseIn(_tx,HIGH);
	long distance;
	distance = duration/58;
	return distance;
}
//-----Ultraschall Distanz Sensor HC-S04 End----//

//-----HDC100X Stuff begin----//

//PUBLIC:

HDC100X::HDC100X(){
	ownAddr = HDC100X_DEFAULT_ADDR;
	dataReadyPin = -1;
}
//-----------------------------------------------------------------------
HDC100X::HDC100X(uint8_t address){
	ownAddr = address;
	//dataReadyPin = pin;
}
//-----------------------------------------------------------------------
HDC100X::HDC100X(bool addr0, bool addr1){
	// set the two bits the way you set the address jumpers
	ownAddr = 0b1000000 |(addr0|(addr1<<1));
	//dataReadyPin = pin;
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

uint8_t HDC100X::begin(uint8_t mode, uint8_t tempRes, uint8_t humiRes, bool heaterState){
	/* sets the mode and resolution and the state of the heater element.  care must be taken, because it will change the temperature reading
	** in:
	** mode: HDC100X_TEMP_HUMI
	** tempRes: HDC100X_11BIT/HDC100X_14BIT
	** humiRes:  HDC100X_8BIT/HDC100X_11BIT/HDC100X_14BIT
	** heaterState: ENABLE/DISABLE
	** out:
	** high byte of the configuration register
	*/
	Wire.begin();
	HDCmode = mode;
	return writeConfigData(mode|(tempRes<<2)|humiRes|(heaterState<<5));
}
//-----------------------------------------------------------------------
uint8_t HDC100X::begin(uint8_t mode, uint8_t resulution, bool heaterState){
		/* sets the mode, resolution and heaterState. Care must be taken, because it will change the temperature reading
	** in:
	** mode: HDC100X_TEMP/HDC100X_HUMI
	** resolution: HDC100X_8BIT(just for the humidity)/HDC100X_11BIT(both)/HDC100X_14BIT(both)
	** heaterState: ENABLE/DISABLE
	** out:
	** high byte of the configuration register
	*/
	Wire.begin();
	HDCmode = mode;
	if(mode == HDC100X_HUMI) 	return writeConfigData(resulution|(heaterState<<5));
	else 						return writeConfigData((resulution<<2)|(heaterState<<5));
}

//######-----------------------------------------------------------------------

void HDC100X::setAddr(uint8_t address){
	/* sets the slave address
	** in:
	** address: slave address byte
	** out:
	** none
	*/
	ownAddr = address;
}
//-----------------------------------------------------------------------
void HDC100X::setAddr(bool addr0, bool addr1){
	/* sets the slave address
	** in:
	** addr0: true/false
	** addr1: true/false
	** out:
	** none
	*/
	ownAddr = 0b1000000 |(addr0|(addr1<<1));
}
//-----------------------------------------------------------------------
void HDC100X::setDrPin(int8_t pin){
	dataReadyPin = pin;
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

uint8_t HDC100X::setMode(uint8_t mode, uint8_t tempRes, uint8_t humiRes){
	/* sets the mode and resolution
	** in:
	** mode: HDC100X_TEMP_HUMI
	** tempRes: HDC100X_11BIT/HDC100X_14BIT
	** humiRes:  HDC100X_8BIT/HDC100X_11BIT/HDC100X_14BIT
	** out:
	** high byte of the configuration register
	*/
	uint8_t tempReg = getConfigReg() & 0xA0;
	HDCmode = mode;
	return writeConfigData(tempReg|mode|(tempRes<<2)|humiRes);
}
//-----------------------------------------------------------------------
uint8_t HDC100X::setMode(uint8_t mode, uint8_t resolution){
	/* sets the mode and resolution
	** in:
	** mode: HDC100X_TEMP/HDC100X_HUMI
	** resolution: HDC100X_8BIT(just for the humidity)/HDC100X_11BIT(both)/HDC100X_14BIT(both)
	** out:
	** high byte of the configuration register
	*/
	uint8_t tempReg = getConfigReg() & 0xA0;
	HDCmode = mode;
	if(mode == HDC100X_HUMI) 	return writeConfigData(tempReg|resolution);
	else 						return writeConfigData(tempReg|(resolution<<2));
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

uint8_t HDC100X::setHeater(bool state){
	/* turns on the heater to get rid of condensation. Care must be taken, because it will change the temperature reading
	** in:
	** state: true/false
	** out:
	** high byte of the configuration register
	*/
	uint8_t regData = getConfigReg() & 0x5F;
	if(state) return writeConfigData(regData|(state<<5));
	return writeConfigData(regData);
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

bool HDC100X::battLow(void){
	// returns a false if input voltage is higher than 2.8V and if lower a true

	if(getConfigReg() & 0x08) return true;
	return false;
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

float HDC100X::getTemp(void){
	// returns the a float number of the temperature in degrees Celsius
	if(HDCmode == HDC100X_TEMP || HDCmode == HDC100X_TEMP_HUMI)
		return ((float)getRawTemp()/65536.0*165.0-40.0);
}
//-----------------------------------------------------------------------
float HDC100X::getHumi(void){
	// returns the a float number of the humidity in percent
	if(HDCmode == HDC100X_HUMI || HDCmode == HDC100X_TEMP_HUMI)
		return ((float)getRawHumi()/65536.0*100.0);
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

uint16_t HDC100X::getRawTemp(void){
	// returns the raw 16bit data of the temperature register
	if(HDCmode == HDC100X_TEMP || HDCmode == HDC100X_TEMP_HUMI)
		return read2Byte(HDC100X_TEMP_REG);
}
//-----------------------------------------------------------------------
uint16_t HDC100X::getRawHumi(void){
	// returns the raw 16bit data of the humidity register
	if(HDCmode == HDC100X_HUMI || HDCmode == HDC100X_TEMP_HUMI)
		return read2Byte(HDC100X_HUMI_REG);
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

uint8_t HDC100X::getConfigReg(void){
	// returns the high byte of the configuration register
	return (read2Byte(HDC100X_CONFIG_REG)>>8);
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

uint16_t HDC100X::read2Byte(uint8_t reg){
	/* reads two bytes from the defined register
	** in:
	** reg: HDC100X_TEMP_REG/HDC100X_HUMI_REG/HDC100X_CONFIG_REG/HDC100X_ID1_REG/HDC100X_ID2_REG/HDC100X_ID3_REG
	** out:
	** two byte of data from the defined register
	*/
	setRegister(reg);
	uint16_t data;
	Wire.requestFrom(ownAddr, 2U);
	if(Wire.available()>=2){
		data = Wire.read()<<8;
		data += Wire.read();
	}
	return data;
}

uint8_t HDC100X::writeConfigData(uint8_t config){
	/* writes the config byte to the configuration register
	** in:
	** config: one byte
	** out:
	** one byte 0:success  1:data too long to fit in transmit buffer    2:received NACK on transmit of address    3:received NACK on transmit of data    4:other error
	*/
	Wire.beginTransmission(ownAddr);
	Wire.write(HDC100X_CONFIG_REG);
	Wire.write(config);
	Wire.write(0x00); 					//the last 8 bits are always 0
	return Wire.endTransmission();
}

//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------
//PRIVATE:
//######-----------------------------------------------------------------------
//######-----------------------------------------------------------------------

void HDC100X::setRegister(uint8_t reg){
	/* set the register for the next read or write cycle
	** in:
	** reg: HDC100X_TEMP_REG/HDC100X_HUMI_REG/HDC100X_CONFIG_REG/HDC100X_ID1_REG/HDC100X_ID2_REG/HDC100X_ID3_REG
	** out:
	** none
	*/
	Wire.beginTransmission(ownAddr);
	Wire.write(reg);
	Wire.endTransmission();
	delay(10);	// wait a little so that the sensor can set its register
}
//-----HDC100X Stuff End----//


//-----Helligkeitssensor 45315 begin----//

/**************************************************************************/
/*!
		Constructor
*/
/**************************************************************************/
TSL45315::TSL45315(uint8_t resolution)
{
	_resolution = resolution;
	_timerfactor = 0;
	if (resolution == uint8_t(TSL45315_TIME_M1)) {
		_timerfactor = 1;
	}
	if (resolution == uint8_t(TSL45315_TIME_M2)) {
		_timerfactor = 2;
	}
	if (resolution == uint8_t(TSL45315_TIME_M4)) {
		_timerfactor = 4;
	}
}


/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/
boolean TSL45315::begin(void)
{
	Wire.begin();
	Wire.beginTransmission(TSL45315_I2C_ADDR);
		Wire.write(0x80|TSL45315_REG_ID);
		Wire.endTransmission();

		Wire.requestFrom(TSL45315_I2C_ADDR, 1);
		while(Wire.available())
		{
		unsigned char c = Wire.read();
		c = c & 0xF0;
		if (c != 0xA0) {
			return false;
		}
		}

		Wire.beginTransmission(TSL45315_I2C_ADDR);
		Wire.write(0x80|TSL45315_REG_CONTROL);
		Wire.write(0x03);
		Wire.endTransmission();

		Wire.beginTransmission(TSL45315_I2C_ADDR);
		Wire.write(0x80|TSL45315_REG_CONFIG);
		Wire.write(_resolution);
		Wire.endTransmission();

	return true;
}


uint32_t TSL45315::readLux(void)
{
	uint32_t lux;

		Wire.beginTransmission(TSL45315_I2C_ADDR);
		Wire.write(0x80|TSL45315_REG_DATALOW);
		Wire.endTransmission();
		Wire.requestFrom(TSL45315_I2C_ADDR, 2);
		_low = Wire.read();
		_high = Wire.read();
		while(Wire.available()){
			Wire.read();
		}

		lux  = (_high<<8) | _low;
	lux = lux * _timerfactor;
	return lux;
}


boolean TSL45315::powerDown(void)
{
	 Wire.beginTransmission(TSL45315_I2C_ADDR);
	 Wire.write(0x80|TSL45315_REG_CONTROL);
	 Wire.write(0x00);
	 Wire.endTransmission();
 return true;
}
//-----Helligkeitssensor 45315 end----//


//-----VEML6070 UV Sensor begin ----///

boolean VEML6070::begin(void)
{
	Wire.begin();

 Wire.beginTransmission(UV_ADDR);
 Wire.write((IT_1<<2) | 0x02);
 Wire.endTransmission();
 delay(500);
}

uint16_t VEML6070::getUV(void)
{
	byte msb=0, lsb=0;
uint16_t uvValue;

Wire.requestFrom(UV_ADDR+1, 1); //MSB
delay(1);
if(Wire.available()) msb = Wire.read();

Wire.requestFrom(UV_ADDR+0, 1); //LSB
delay(1);
if(Wire.available()) lsb = Wire.read();

uvValue = (msb<<8) | lsb;

return uvValue*5;
}
