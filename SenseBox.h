/*
  SenseBox.h Arduino
*/
#ifndef SenseBox_h
#define SenseBox_h

#include "Arduino.h"
#include <inttypes.h>
#include "Wire.h"

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

    void begin(void);
    void stop(void);
    void get(uint8_t *sec, uint8_t *min, uint8_t *hour, uint8_t *day, uint8_t *month, uint16_t *year);
    void get(int *sec, int *min, int *hour, int *day, int *month, int *year);
    void set(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year);
    void set(int sec, int min, int hour, int day, int month, int year);
    void set12HourMode(void);
    void set24HourMode(void);
    void batterySwitchOver(int on);
    //new
    //void	setTime(void);
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



//----------------------rgb----------------------------
//----------------------sd-------------------------------

#endif
