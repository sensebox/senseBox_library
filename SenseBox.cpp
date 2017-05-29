#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "SenseBox.h"
//RTC
#if defined(__AVR__)
# include <avr/io.h>
#endif
#if ARDUINO >= 100
# include "Arduino.h"
#else
# include "WProgram.h"
#endif
#include "Wire.h"
#include <math.h>




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
	ownAddr = 0x43;
	//dataReadyPin = -1;
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

uint8_t HDC100X::begin(void){		
	Wire.begin();
	uint8_t config = writeConfigData((HDC100X_14BIT<<2)|(DISABLE<<5));
	getTemp();
	return config;
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

TSL45315::TSL45315(void)
{	
	_timerfactor = 4;	
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


uint32_t TSL45315::getLux(void)
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

//-----------------RTC BEGIN-------
#define I2C_ADDR (0xD0>>1)


//-------------------- Constructor --------------------


RV8523::RV8523(void)
{
  Wire.begin();

  return;
}


//-------------------- Public --------------------


void RV8523::start(void)
{
  uint8_t val;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x00)); //control 1
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, 1);
  val = Wire.read();

  if(val & (1<<5))
  {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(byte(0x00)); //control 1
    Wire.write(val & ~(1<<5)); //clear STOP (bit 5)
    Wire.endTransmission();
  }

  return;
}


void RV8523::stop(void)
{
  uint8_t val;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x00)); //control 1
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, 1);
  val = Wire.read();

  if(!(val & (1<<5)))
  {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(byte(0x00)); //control 1
    Wire.write(val | (1<<5)); //set STOP (bit 5)
    Wire.endTransmission();
  }

  return;
}


void RV8523::set12HourMode(void) //set 12 hour mode
{
  uint8_t val;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x00)); //control 1
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, 1);
  val = Wire.read();

  if(!(val & (1<<3)))
  {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(byte(0x00)); //control 1
    Wire.write(val | (1<<3)); //set 12 hour mode (bit 3)
    Wire.endTransmission();
  }

  return;
}


void RV8523::set24HourMode(void) //set 24 hour mode
{
  uint8_t val;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x00)); //control 1
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, 1);
  val = Wire.read();

  if(val & (1<<3))
  {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(byte(0x00)); //control 1
    Wire.write(val & ~(1<<3)); //set 12 hour mode (bit 3)
    Wire.endTransmission();
  }

  return;
}


void RV8523::batterySwitchOver(int on) //activate/deactivate battery switch over mode
{   
  uint8_t val;

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x02)); //control 3
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, 1);
  val = Wire.read();
  if(val & 0xE0)
  {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(byte(0x02)); //control 3
    if(on)
    {
      Wire.write(val & ~0xE0); //battery switchover in standard mode
    }
    else
    {
      Wire.write(val | 0xE0);  //battery switchover disabled
    }
    Wire.endTransmission();
  }

  return;
}


void RV8523::get(uint8_t *sec, uint8_t *min, uint8_t *hour, uint8_t *day, uint8_t *month, uint16_t *year)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR, 7);
  *sec   = bcd2bin(Wire.read() & 0x7F);
  *min   = bcd2bin(Wire.read() & 0x7F);
  *hour  = bcd2bin(Wire.read() & 0x3F); //24 hour mode
  *day   = bcd2bin(Wire.read() & 0x3F);
           bcd2bin(Wire.read() & 0x07); //day of week
  *month = bcd2bin(Wire.read() & 0x1F);
  *year  = bcd2bin(Wire.read()) + 2000;

  return;
}


void RV8523::get(int *sec, int *min, int *hour, int *day, int *month, int *year)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR, 7);
  *sec   = bcd2bin(Wire.read() & 0x7F);
  *min   = bcd2bin(Wire.read() & 0x7F);
  *hour  = bcd2bin(Wire.read() & 0x3F); //24 hour mode
  *day   = bcd2bin(Wire.read() & 0x3F);
           bcd2bin(Wire.read() & 0x07); //day of week
  *month = bcd2bin(Wire.read() & 0x1F);
  *year  = bcd2bin(Wire.read()) + 2000;

  return;
}


void RV8523::set(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year)
{
  if(year > 2000)
  {
    year -= 2000;
  }

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.write(bin2bcd(sec));
  Wire.write(bin2bcd(min));
  Wire.write(bin2bcd(hour));
  Wire.write(bin2bcd(day));
  Wire.write(bin2bcd(0));
  Wire.write(bin2bcd(month));
  Wire.write(bin2bcd(year));
  Wire.endTransmission();

  return;
}


void RV8523::set(int sec, int min, int hour, int day, int month, int year)
{
  return set((uint8_t)sec, (uint8_t)min, (uint8_t)hour, (uint8_t)day, (uint8_t)month, (uint16_t)year);
}


//-------------------- Private --------------------


uint8_t RV8523::bin2bcd(uint8_t val)
{
  return val + 6 * (val / 10);
}


uint8_t RV8523::bcd2bin(uint8_t val)
{
  return val - 6 * (val >> 4);
}


//-----new----RTC---Functions

void RV8523::begin(void)
{
	start();
}
// A convenient constructor for using "the compiler's time":
void RV8523::setTime(const char* date, const char* time)
{
	
char buffer1[12]; 
strcpy(buffer1, date);
  uint16_t year =  (buffer1[7] -48) * 1000;
  year += (buffer1[8] -48) * 100;
  year += (buffer1[9] -48) * 10;
  year += (buffer1[10] -48);

  uint8_t month;
  switch (buffer1[0]) {
        case 'J': month = (buffer1[1] == 'a') ? 1 : ((buffer1[2] == 'n') ? 6 : 7); break;
        case 'F': month = 2; break;
        case 'A': month = buffer1[2] == 'r' ? 4 : 8; break;
        case 'M': month = buffer1[2] == 'r' ? 3 : 5; break;
        case 'S': month = 9; break;
        case 'O': month = 10; break;
        case 'N': month = 11; break;
        case 'D': month = 12; break;
    }
  
  uint8_t day = (buffer1[4] -48) * 10;
  day += (buffer1[5] -48);

  char buffer2[9];
  strcpy(buffer2, time);
  uint8_t hour = (buffer2[0]-48)*10 + (buffer2[1]-48);
  uint8_t min = (buffer2[3]-48)*10 + (buffer2[4]-48);
  uint8_t sec = (buffer2[6]-48)*10 + (buffer2[7]-48);

  set(sec,  min,  hour,  day,  month,  year);

  return;
}

uint16_t RV8523::getYear(void)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR, 7);
  uint8_t sec   = bcd2bin(Wire.read() & 0x7F);
  uint8_t min   = bcd2bin(Wire.read() & 0x7F);
  uint8_t hour  = bcd2bin(Wire.read() & 0x3F); //24 hour mode
  uint8_t day   = bcd2bin(Wire.read() & 0x3F);
           bcd2bin(Wire.read() & 0x07); //day of week
  uint8_t month = bcd2bin(Wire.read() & 0x1F);
  uint16_t year  = bcd2bin(Wire.read()) + 2000;

  
  return year;
}

uint8_t RV8523::getMonth(void)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR, 7);
  uint8_t sec   = bcd2bin(Wire.read() & 0x7F);
  uint8_t min   = bcd2bin(Wire.read() & 0x7F);
  uint8_t hour  = bcd2bin(Wire.read() & 0x3F); //24 hour mode
  uint8_t day   = bcd2bin(Wire.read() & 0x3F);
           bcd2bin(Wire.read() & 0x07); //day of week
  uint8_t month = bcd2bin(Wire.read() & 0x1F);
  uint16_t year  = bcd2bin(Wire.read()) + 2000;
  

  return month;
}

uint8_t RV8523::getDay(void)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR, 7);
  uint8_t sec   = bcd2bin(Wire.read() & 0x7F);
  uint8_t min   = bcd2bin(Wire.read() & 0x7F);
  uint8_t hour  = bcd2bin(Wire.read() & 0x3F); //24 hour mode
  uint8_t day   = bcd2bin(Wire.read() & 0x3F);
           bcd2bin(Wire.read() & 0x07); //day of week
  uint8_t month = bcd2bin(Wire.read() & 0x1F);
  uint16_t year  = bcd2bin(Wire.read()) + 2000;
  

  return day;
}

uint8_t RV8523::getHour(void)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR, 7);
  uint8_t sec   = bcd2bin(Wire.read() & 0x7F);
  uint8_t min   = bcd2bin(Wire.read() & 0x7F);
  uint8_t hour  = bcd2bin(Wire.read() & 0x3F); //24 hour mode
  uint8_t day   = bcd2bin(Wire.read() & 0x3F);
           bcd2bin(Wire.read() & 0x07); //day of week
  uint8_t month = bcd2bin(Wire.read() & 0x1F);
  uint16_t year  = bcd2bin(Wire.read()) + 2000;
  

  return hour;
}

uint8_t RV8523::getMin(void)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR, 7);
  uint8_t sec   = bcd2bin(Wire.read() & 0x7F);
  uint8_t min   = bcd2bin(Wire.read() & 0x7F);
  uint8_t hour  = bcd2bin(Wire.read() & 0x3F); //24 hour mode
  uint8_t day   = bcd2bin(Wire.read() & 0x3F);
           bcd2bin(Wire.read() & 0x07); //day of week
  uint8_t month = bcd2bin(Wire.read() & 0x1F);
  uint16_t year  = bcd2bin(Wire.read()) + 2000;

  return min;
}

uint8_t RV8523::getSec(void)
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(0x03));
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR, 7);
  uint8_t sec   = bcd2bin(Wire.read() & 0x7F);
  uint8_t min   = bcd2bin(Wire.read() & 0x7F);
  uint8_t hour  = bcd2bin(Wire.read() & 0x3F); //24 hour mode
  uint8_t day   = bcd2bin(Wire.read() & 0x3F);
           bcd2bin(Wire.read() & 0x07); //day of week
  uint8_t month = bcd2bin(Wire.read() & 0x1F);
  uint16_t year  = bcd2bin(Wire.read()) + 2000;
  

  return sec;
}

//------bmp




BMP280::BMP280()
{
	//do nothing
}
double BMP280::getPressure(void)
{
	setOversampling(4);
	double uP,uT ;
	char result = getUnPT(uP,uT);
	if(result!=0){
		// calculate the temperature
		result = calcTemperature(uT,uT);
		if(result){
			// calculate the pressure
			result = calcPressure(uP,uP);
			if(result)return (1);
			else error = 3 ;	// pressure error ;
			return (0);
		}else 
			error = 2;	// temperature error ;
	}
	else 
		error = 1;
	
	return (uP);
}
/*
*	Initialize library and coefficient for measurements
*/
char BMP280::begin() 
{
	
	// Start up the Arduino's "wire" (I2C) library:
	Wire.begin();

	// The BMP280 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking measurements.

	// Retrieve calibration data from device:
	
	if (    
		readUInt(0x88, dig_T1) &&
		readInt(0x8A, dig_T2)  &&
		readInt(0x8C, dig_T3)  &&
		readUInt(0x8E, dig_P1) &&
		readInt(0x90, dig_P2)  &&
		readInt(0x92, dig_P3)  &&
		readInt(0x94, dig_P4)  &&
		readInt(0x96, dig_P5)  &&
		readInt(0x98, dig_P6)  &&
		readInt(0x9A, dig_P7)  &&
		readInt(0x9C, dig_P8)  &&
		readInt(0x9E, dig_P9)){
		/*
		
		*/
		return (1);
	}
	else 
		return (0);
}

/*
**	Read a signed integer (two bytes) from device
**	@param : address = register to start reading (plus subsequent register)
**	@param : value   = external variable to store data (function modifies value)
*/
char BMP280::readInt(char address, int &value)

{
	unsigned char data[2];	//char is 4bit,1byte

	data[0] = address;
	if (readBytes(data,2))
	{
		value = (((int)data[1]<<8)|(int)data[0]);
		return(1);
	}
	value = 0;
	return(0);
}
/* 
**	Read an unsigned integer (two bytes) from device
**	@param : address = register to start reading (plus subsequent register)
**	@param : value 	 = external variable to store data (function modifies value)
*/

char BMP280::readUInt(char address, unsigned int &value)
{
	unsigned char data[2];	//4bit
	data[0] = address;
	if (readBytes(data,2))
	{
		value = (((unsigned int)data[1]<<8)|(unsigned int)data[0]);
		return(1);
	}
	value = 0;
	return(0);
}
/*
** Read an array of bytes from device
** @param : value  = external array to hold data. Put starting register in values[0].
** @param : length = number of bytes to read
*/

char BMP280::readBytes(unsigned char *values, char length)
{
	char x;

	Wire.beginTransmission(BMP280_ADDR);
	Wire.write(values[0]);
	error = Wire.endTransmission();
	if (error == 0)
	{
		Wire.requestFrom(BMP280_ADDR,length);
		while(Wire.available() != length) ; // wait until bytes are ready
		for(x=0;x<length;x++)
		{
			values[x] = Wire.read();
		}
		return(1);
	}
	return(0);
}
/*
** Write an array of bytes to device
** @param : values = external array of data to write. Put starting register in values[0].
** @param : length = number of bytes to write
*/
char BMP280::writeBytes(unsigned char *values, char length)
{
	Wire.beginTransmission(BMP280_ADDR);
	Wire.write(values,length);
	error = Wire.endTransmission();
	if (error == 0)
		return(1);
	else
		return(0);
}

short BMP280::getOversampling(void)
{
	return oversampling;
}

char BMP280::setOversampling(short oss)
{
	oversampling = oss;
	return (1);
}
/*
**	Begin a measurement cycle.
** Oversampling: 0 to 4, higher numbers are slower, higher-res outputs.
** @returns : delay in ms to wait, or 0 if I2C error.
*/
char BMP280::startMeasurment(void)

{
	unsigned char data[2], result, delay;
	
	data[0] = BMP280_REG_CONTROL;

	switch (oversampling)
	{
		case 0:
			data[1] = BMP280_COMMAND_PRESSURE0;     
			oversampling_t = 1;
			delay = 8;			
		break;
		case 1:
			data[1] = BMP280_COMMAND_PRESSURE1;     
			oversampling_t = 1;
			delay = 10;			
		break;
		case 2:
			data[1] = BMP280_COMMAND_PRESSURE2;		
			oversampling_t = 1;
			delay = 15;
		break;
		case 3:
			data[1] = BMP280_COMMAND_PRESSURE3;
			oversampling_t = 1;
			delay = 24;
		break;
		case 4:
			data[1] = BMP280_COMMAND_PRESSURE4;
			oversampling_t = 1;
			delay = 45;
		break;
		default:
			data[1] = BMP280_COMMAND_PRESSURE0;
			delay = 9;
		break;
	}
	result = writeBytes(data, 2);
	if (result) // good write?
		return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}

/*
**	Get the uncalibrated pressure and temperature value.
**  @param : uP = stores the uncalibrated pressure value.(20bit)
**  @param : uT = stores the uncalibrated temperature value.(20bit)
*/
char BMP280::getUnPT(double &uP, double &uT)
{
	unsigned char data[6];
	char result;
	
	data[0] = BMP280_REG_RESULT_PRESSURE; //0xF7 

	result = readBytes(data, 6); // 0xF7; xF8, 0xF9, 0xFA, 0xFB, 0xFC
	if (result) // good read
	{
		double factor = pow(2, 4);
		uP = (( (data[0] *256.0) + data[1] + (data[2]/256.0))) * factor ;	//20bit UP
		uT = (( (data[3] *256.0) + data[4] + (data[5]/256.0))) * factor ;	//20bit UT
		
	}
	return(result);
}
/*
** Retrieve temperature and pressure.
** @param : T = stores the temperature value in degC.
** @param : P = stores the pressure value in mBar.
*/
char BMP280::getTemperatureAndPressure(double &T,double &P)
{
	double uP,uT ;
	char result = getUnPT(uP,uT);
	if(result!=0){
		// calculate the temperature
		result = calcTemperature(T,uT);
		if(result){
			// calculate the pressure
			result = calcPressure(P,uP);
			if(result)return (1);
			else error = 3 ;	// pressure error ;
			return (0);
		}else 
			error = 2;	// temperature error ;
	}
	else 
		error = 1;
	
	return (0);
}
/*
** temperature calculation
** @param : T  = stores the temperature value after calculation.
** @param : uT = the uncalibrated temperature value.
*/
char BMP280::calcTemperature(double &T, double &uT)
//
{
	double adc_T = uT ;
	//Serial.print("adc_T = "); Serial.println(adc_T,DEC);
		
	double var1 = (((double)adc_T)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
	double var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0)*(((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);
	t_fine = (long signed int)(var1+var2);
		
	T = (var1+var2)/5120.0;
	
	if(T>100.0 || T <-100.0)return 0;
	
	return (1);
}
/*
**	Pressure calculation from uncalibrated pressure value.
**  @param : P  = stores the pressure value.
**  @param : uP = uncalibrated pressure value. 
*/
char BMP280::calcPressure(double &P,double uP)
{
	//char result;
	double var1 , var2 ;
	
	var1 = ((double)t_fine/2.0) - 64000.0;
	//Serial.print("var1 = ");Serial.println(var1,2);
	var2 = var1 * (var1 * ((double)dig_P6)/32768.0);	//not overflow
	//Serial.print("var2 = ");Serial.println(var2,2);
	var2 = var2 + (var1 * ((double)dig_P5)*2.0);	//overflow
	//Serial.print("var2 = ");Serial.println(var2,2);
		
	var2 = (var2/4.0)+(((double)dig_P4)*65536.0);
	//Serial.print("var2 = ");Serial.println(var2,2);
		
	var1 = (((double)dig_P3) * var1 * var1/524288.0 + ((double)dig_P2) * var1) / 524288.0;
	//Serial.print("var1 = ");Serial.println(var1,2);
		
		
	//Serial.print("(32768.0 + var1) = ");Serial.println((32768.0 + var1),5);
		
	double t_var = (32768.0 + var1)/32768.0;
	//Serial.print("((32768.0 + var1)/32768.0) = "); Serial.println(t_var,5);
	//Serial.print("dig_P1 = ");Serial.println(dig_P1);
	//Serial.print("dig_P1 = ");Serial.println((double)dig_P1,5);
	double tt_var = t_var * (double)dig_P1;
		
	//Serial.print("mulipication = "); Serial.println(tt_var,5);
	
	var1 = ((32768.0 + var1)/32768.0)*((double)dig_P1);
	//Serial.print("var1 = ");Serial.println(var1,2);
		
	double p = 1048576.0- (double)uP;
	//Serial.print("p = ");Serial.println(p,2);
		
	p = (p-(var2/4096.0))*6250.0/var1 ;	//overflow
	//Serial.print("p = ");Serial.println(p,2);	
		
	var1 = ((double)dig_P9)*p*p/2147483648.0;	//overflow
		
	var2 = p*((double)dig_P8)/32768.0;
	//Serial.print("var1 = ");Serial.println(var1,2);
	p = p + (var1+var2+((double)dig_P7))/16.0;
	//Serial.print("p = ");Serial.println(p,2);
		
	P = p/100.0 ;
	
	if(P>1200.0 || P < 800.0)return (0);
	return (1);
}




double BMP280::sealevel(double P, double A)
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}


double BMP280::altitude(double P, double P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}


char BMP280::getError(void)
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error
{
	return(error);
}


