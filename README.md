# SenseBox Library

Use Library for the following sensors:

- HDC100X
- VEML 6070
- TSL45315
- BMP 280
- HC SR04 Ultrasonic Sensor
- RV8523 RTC
- RGB LED

# Usage

Include library with `#include <SenseBox.h>`. Before the setup routine define the classes and classnames:

- `HDC100X HDC1(0,0);` define the I2C-Adress within the Brackets
- `Ultrasonic Ultrasonic(3,4);` set RX and TX Pins
- `TSL45315 luxsensor = TSL45315(TSL45315_TIME_M4);`
- `VEML6070 uvsensor;`
- `RV8523 rtc;`

In the `setup()`-Routine initialise the sensor with `classname.begin()`. For example `HDC1.begin()`.
Now you can use `classname.getValue`. For example `HDC1.getTemp()`. See also the example folder.
