/*
  SenseBox.h Arduino
*/
#ifndef SenseBox_h
#define SenseBox_h

#include "Arduino.h"
#include <inttypes.h>
#include "Wire.h"

#include "OpenSenseMap.h"

//-----HDC100X Stuff begin----//
#define HDC100X_DEFAULT_ADDR		0x40

#define HDC100X_TEMP_REG			0x00
#define HDC100X_HUMI_REG			0x01
#define	HDC100X_CONFIG_REG			0x02
#define HDC100X_ID1_REG				0xFB
#define HDC100X_ID2_REG				0xFC
#define HDC100X_ID3_REG				0xFD


#define HDC100X_RST					0x80
#define	HDC100X_TEMP_HUMI			0x16
#define	HDC100X_HUMI				1
#define	HDC100X_TEMP				0

#define HDC100X_14BIT				0x00
#define HDC100X_11BIT				0x01
#define HDC100X_8BIT				0x02

#define DISABLE						0
#define ENABLE						1
//-----HDC100X Stuff End----//
//-----TSL45315 Stuff Begin-//
#define TSL45315_I2C_ADDR 		(0x29)

#define TSL45315_REG_CONTROL  	(0x00)
#define TSL45315_REG_CONFIG   	(0x01)
#define TSL45315_REG_DATALOW  	(0x04)
#define TSL45315_REG_DATAHIGH 	(0x05)
#define TSL45315_REG_ID       	(0x0A)

// Sensing time for one measurement
#define TSL45315_TIME_M1		(0x00)	// M=1 T=400ms
#define TSL45315_TIME_M2		(0x01)	// M=2 T=200ms
#define TSL45315_TIME_M4		(0x02)	// M=4 T=100ms
//-----TSL45315 Stuff End-//

//----VEML6040 Stuff Begin-//
#define UV_ADDR 0x38
// Integrationszeiten
#define IT_0_5 0x0 // 1/2T
#define IT_1   0x1 // 1T
#define IT_2   0x2 // 2T
#define IT_4   0x3 // 4T
// Referenzwert: 0,01 W/m^2 ist äquivalent zu 0.4 als UV-Index
//----VEML6070 Stuff End--/

//---RTC Stuff Begin-//
#define RV8523_h
//---RTC Stuff End--//

//---BMP_STUFF----/
#define BMP280_ADDR 0x76 // 7-bit address

#define	BMP280_REG_CONTROL 0xF4
#define	BMP280_REG_RESULT_PRESSURE 0xF7			// 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
#define BMP280_REG_RESULT_TEMPRERATURE 0xFA		// 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.

#define	BMP280_COMMAND_TEMPERATURE 0x2E
#define	BMP280_COMMAND_PRESSURE0 0x25
#define	BMP280_COMMAND_PRESSURE1 0x29
#define	BMP280_COMMAND_PRESSURE2 0x2D
#define	BMP280_COMMAND_PRESSURE3 0x31
#define	BMP280_COMMAND_PRESSURE4 0x5D

//-----Ultraschall Distanz Sensor HC-S04----//
class Ultrasonic
{
  public:
    Ultrasonic(int rx, int tx);
		long getDistance(void);
  private:
    int _rx;//pin of rx pin
    int _tx;//pin of tx pin
};
//----------------------------------------//

//-----Temperatur und Luftfeuchtigkeit Sensor HDC100X----//
class HDC100X{
	public:
		HDC100X();
		HDC100X(uint8_t address);
		HDC100X(bool addr0, bool addr1);

		uint8_t begin(uint8_t mode, uint8_t tempRes, uint8_t humiRes, bool heaterState);
		uint8_t begin(uint8_t mode, uint8_t resulution, bool heaterState);
		uint8_t begin(void);

		void setAddr(bool addr0, bool addr1);
		void setAddr(uint8_t address);
		void setDrPin(int8_t pin);

		uint8_t setMode(uint8_t mode, uint8_t tempRes, uint8_t humiRes);
		uint8_t setMode(uint8_t mode, uint8_t resolution);

		uint8_t setHeater(bool state);
		bool battLow(void);

		float getTemp(void);
		float getHumi(void);

		uint16_t getRawTemp(void);
		uint16_t getRawHumi(void);

		uint8_t getConfigReg(void);
		uint16_t read2Byte(uint8_t reg);

		uint8_t writeConfigData(uint8_t config);

	private:
  		uint8_t ownAddr;
		uint8_t dataReadyPin;
		uint8_t HDCmode;
  		void setRegister(uint8_t reg);

};
//----------------------------------------//

//-----Helligkeitssensor 45315----//

class TSL45315 {
public:
	TSL45315(uint8_t resolution);
	TSL45315(void);
	boolean begin(void);
	uint32_t getLux(void);
	boolean powerDown(void);


private:
	uint16_t _low, _high, _timerfactor;
	uint8_t _resolution;
};
//----------------------------------------//

//------VEML6070 -------//

class VEML6070{
public:
  boolean begin(void);
  uint16_t getUV(void);


private:
float refVal = 0.4;

};

//----------------------rtc-------------------------------

class RV8523
{
  public:
    RV8523();

    void start(void);
    void stop(void);
    void get(uint8_t *sec, uint8_t *min, uint8_t *hour, uint8_t *day, uint8_t *month, uint16_t *year);
    void get(int *sec, int *min, int *hour, int *day, int *month, int *year);
    void set(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year);
    void set(int sec, int min, int hour, int day, int month, int year);
    void set12HourMode(void);
    void set24HourMode(void);
    void batterySwitchOver(int on);

    
    //new funtions
    void begin(void);
    void setTime(const char* date, const char* time);
    uint16_t getYear(void);
    uint8_t getMonth(void);
    uint8_t getDay(void);
    uint8_t getHour(void);
    uint8_t getMin(void);
    uint8_t getSec(void);
    
    
  private:
    uint8_t bin2bcd(uint8_t val);
    uint8_t bcd2bin(uint8_t val);
};
    
   
   
 


//----------------------bmp----------------------------
class BMP280
{
	public:
		BMP280(); // base type

		double getPressure(void);
		char begin();
			// call pressure.begin() to initialize BMP280 before use
			// returns 1 if success, 0 if failure (i2C connection problem.)

		short getOversampling(void);
		char  setOversampling(short oss);

		char startMeasurment(void);
			// command BMP280 to start a pressure measurement
			// oversampling: 0 - 3 for oversampling value
			// returns (number of ms to wait) for success, 0 for fail

		char calcTemperature(double &T, double &uT);
			// calculation the true temperature from the given uncalibrated Temperature

		char calcPressure(double &P, double uP);
			//calculation for measuring pressure.

		double sealevel(double P, double A);
			// convert absolute pressure to sea-level pressure
			// P: absolute pressure (mbar)
			// A: current altitude (meters)
			// returns sealevel pressure in mbar

		double altitude(double P, double P0);
			// convert absolute pressure to altitude (given baseline pressure; sea-level, runway, etc.)
			// P: absolute pressure (mbar)
			// P0: fixed baseline pressure (mbar)
			// returns signed altitude in meters

		char getError(void);
			// If any library command fails, you can retrieve an extended
			// error code using this command. Errors are from the wire library:
			// 0 = Success
			// 1 = Data too long to fit in transmit buffer
			// 2 = Received NACK on transmit of address
			// 3 = Received NACK on transmit of data
			// 4 = Other error

		char getTemperatureAndPressure(double& T,double& P);

	private:

		char readInt(char address, int &value);
			// read an signed int (16 bits) from a BMP280 register
			// address: BMP280 register address
			// value: external signed int for returned value (16 bits)
			// returns 1 for success, 0 for fail, with result in value

		char readUInt(char address, unsigned int &value);
			// read an unsigned int (16 bits) from a BMP280 register
			// address: BMP280 register address
			// value: external unsigned int for returned value (16 bits)
			// returns 1 for success, 0 for fail, with result in value

		char readBytes(unsigned char *values, char length);
			// read a number of bytes from a BMP280 register
			// values: array of char with register address in first location [0]
			// length: number of bytes to read back
			// returns 1 for success, 0 for fail, with read bytes in values[] array

		char writeBytes(unsigned char *values, char length);
			// write a number of bytes to a BMP280 register (and consecutive subsequent registers)
			// values: array of char with register address in first location [0]
			// length: number of bytes to write
			// returns 1 for success, 0 for fail

		char getUnPT(double &uP, double &uT);
			//get uncalibrated UP and UT value.


		int dig_T2 , dig_T3 , dig_T4 , dig_P2 , dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
		unsigned int dig_P1,dig_T1 ;
		short oversampling, oversampling_t;
		long signed int t_fine;
		char error;
};

// BMP uses 0x76 or 0x77 depending on voltage on SDO. (see: https://github.com/watterott/BMP280-Breakout)
// change default to 0x76 for easier hardware setup

//----------------------sd-------------------------------

#endif
