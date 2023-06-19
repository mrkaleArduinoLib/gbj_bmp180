# gbjBMP180

Library for the barometric sensors _BMP180_ with `two-wire` (also known as <abbr title='Inter-Integrated Circuit'>I2C</abbr>) bus interface.

* It is compatible with sensor `BMP085`.
* Sensor address is `0x77` hardcoded and cannot be changed by any library method.
* The library provides measured temperature in degrees of Celsius and air pressure in Pascal.
* For conversion among various temperature unit scales and for calculating dew point temperature use library `gbjAppHelpers`.
* The sensor has temperature resolution `0.1 °C` and pressure resolution `10 Pa` (0.01 hPa).


#### Particle hardware configuration
* Connect microcontroller's pin `D0` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).

#### Arduino UNO hardware configuration
* Connect microcontroller's pin `A4` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `A5` to sensor's pin **SCL** (Serial Clock).

#### Espressif - ESP8266, ESP32 default hardware configuration
* Connect microcontroller's pin `D2` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).


<a id="dependency"></a>

## Dependency

#### Particle platform
* **Particle.h**: Includes alternative (C++) data type definitions.

#### Arduino platform
* **Arduino.h**: Main include file for the Arduino SDK version greater or equal to 100.
* **inttypes.h**: Integer type conversions. This header file includes the exact-width integer definitions and extends them with additional facilities provided by the implementation.
* **TwoWire**: I2C system library loaded from the file `Wire.h`.

#### Custom Libraries
* **gbjTwoWire**: I2C custom library loaded from the file `gbj_twowire.h`, which provides common bus functionality.


<a id="constants"></a>

## Constants
* **GBJ\_BMP180\_TEST**: If this preprocessor macro is defined before including the library header file in a sketch, the library utilizes all calibration and temprature and pressure uncompensated values from the datasheet instead of real received values from the sensor. That mode serves for testing compensating algorithm implemented in the library.

The library does not have specific error codes. Error codes as well as result code are inherited from the parent library [gbjTwoWire](#dependency) only. The result code and error codes can be tested in the operational code with its method `getLastResult()`, `isError()` or `isSuccess()`.


### Referencing constants
In a sketch the constants can be referenced in following forms:
* **Static constant** in the form `gbj_bmp180::<enumeration>::<constant>` or shortly `gbj_bmp180::<constant>`, e.g., _gbj_bmp180::ClockSpeeds::CLOCK\_400KHZ_ or _gbj_bmp180::CLOCK\_400KHZ_.
* **Instance constant** in the form `<object>.<constant>`, e.g., _sensor.CLOCK_400KHZ_.
```cpp
gbj_bmp180 sensor = gbj_bmp180(sensor.CLOCK_400KHZ)
```


<a id="interface"></a>

## Interface

#### Main
* [gbj_bmp180()](#gbj_bmp180)
* [begin()](#begin)
* [reset()](#reset)
* [measureTemperature()](#measureTemperature)
* [measurePressure()](#measurePressure)

#### Setters
* [setOversamplingLow()](#setOversampling)
* [setOversamplingStandard()](#setOversampling)
* [setOversamplingHigh()](#setOversampling)
* [setOversamplingHighUltra()](#setOversampling)
* [setOversampling()](#setOversampling)

#### Getters
* [getPressureSea()](#getPressureSea)
* [getAltitude()](#getAltitude)
* [getOversampling()](#getOversampling)
* [getErrorValue()](#getErrorValue)

Other possible setters and getters are inherited from the parent library [gbjTwoWire](#dependency) and described there.


<a id="gbj_bmp180"></a>

## gbj_bmp180()

#### Description
The library does not need special constructor and destructor, so that the inherited ones are good enough and there is no need to define them in the library, just use it with default or specific parameters as defined at constructor of parent library [gbjTwoWire](#dependency).
* Constructor sets parameters specific to the two-wire bus in general.
* All the constructor parameters can be changed dynamically with corresponding setters later in a sketch.

#### Syntax
    gbj_bmp180(uint32_t clockSpeed, uint8_t pinSDA, uint8_t pinSCL)

#### Parameters
* **clockSpeed**: Two-wire bus clock frequency in Hertz.
  * *Valid values*:ClockSpeeds::CLOCK\_100KHZ, ClockSpeeds::CLOCK\_400KHZ
  * *Default value*: ClockSpeeds::CLOCK\_100KHZ

* **pinSDA**: Microcontroller's pin for serial data. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms for communication on the bus. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  * *Valid values*: positive integer
  * *Default value*: 4 (GPIO4, D2)

* **pinSCL**: Microcontroller's pin for serial clock. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  * *Valid values*: positive integer
  * *Default value*: 5 (GPIO5, D1)

#### Returns
Object performing the sensor management.
The constructor cannot return [a result or error code](#constants) directly, however, it stores them in the instance object.

#### Example
The method has all arguments defaulted and calling without any parameters is equivalent to the calling with all arguments set by corresponding constant with default value:

```cpp
  gbj_bmp180 sensor = gbj_bmp180(); // It is equivalent to
  gbj_bmp180 sensor = gbj_bmp180(sensor.CLOCK_100KHZ, D2, D1)
```

[Back to interface](#interface)


<a id="begin"></a>

## begin()

#### Description
The method creates class instance object and initiates two-wire bus and sensor.

#### Syntax
    ResultCodes begin()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

[Back to interface](#interface)


<a id="reset"></a>

## reset()

#### Description
The method resets the sensor to power-up state without need of power outage.

#### Syntax
    ResultCodes reset()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

[Back to interface](#interface)


<a id="measureTemperature"></a>

## measureTemperature()

#### Description
The method measures temperature only.
* The method is useful when barometric pressure is not needed.
* The final temperature value is compensated with calibration data hardcoded in the sensor.

#### Syntax
    float measureTemperature()

#### Parameters
None

#### Returns
Temperature in centigrade or erroneous value returned by [getErrorValue()](#getErrorValue). The error code can be tested in the operational code with the method [getLastResult()](#getLastResult), [isError()](#isError), or [isSuccess()](#isSuccess).

#### See also
[measurePressure()](#measurePressure)

[Back to interface](#interface)


<a id="measurePressure"></a>

## measurePressure()

#### Description
The method measures barometric pressure alongside with temperature at once, because the temperature is needed for pressure compensation calculation.
* The temperature is returned through referenced input parameter.
* The final presure value is compensated with calibration data hardcoded in the sensor.

#### Syntax
    float measurePressure(float &temperature)

#### Parameters
* **temperature**: Referenced variable for placing a temperature value in centigrade.
  * *Valid values*: sensor specific
  * *Default value*: none

#### Returns
Relative humidity truncated and compensated to range 0 - 100 °C or erroneous value returned by [getErrorRHT()](#getErrorRHT).

#### Example
``` cpp
gbj_bmp180 sensor = gbj_bmp180();
float tempValue, pressValue;
setup()
{
  if (sensor.isSuccess(sensor.begin()))
  {
    pressValue = sensor.measurePressure(tempValue);
  }
}
```

#### See also
[measureTemperature()](#measureTemperature)

[Back to interface](#interface)


<a id="setOversampling"></a>

## setOversamplingLow(), setOversamplingStandard(), setOversamplingHigh(), setOversamplingHighUltra(), setOversampling()

#### Description
The particular method sets corresponding oversampling rate and related conversion time period for subsequent measurement according to the data sheet.
* The oversampling rate is determined either by the suffix in a method's name or in the argument.
* The method with input argument is suitable for setting an oversampling from previous obtained value by the corresponding getter.

#### Syntax
    void setOversamplingLow()
    void setOversamplingStandard()
    void setOversamplingHigh()
    void setOversamplingHighUltra()
    void setOversampling(uint8_t oss)

#### Parameters
* **oss**: Oversampling set of pressure measurement rate. It is sanitized to the valid range.
  * *Valid values*: non-negative integer 0 ~ 3
  * *Default value*: none

#### Returns
None

#### See also
[getOversampling()](#getOversampling)

[Back to interface](#interface)


<a id="getPressureSea"></a>

## getPressureSea()

#### Description
The method calculates barometric pressure at sea level from provided pressure and altitude.
* The input pressure is usually just measured one by the method [measurePressure()](#measurePressure), but the method calculates in general.
* The measurement unit of the local pressure can be arbitrary. However the methods returns the sea level pressure in the same unit.

#### Syntax
    float getPressureSea(float pressure, float altitude)

#### Parameters
* **pressure**: Local barometric pressure in arbitrary measurement unit, usually Pascal or hectoPascal.
  * *Valid values*: decimal number
  * *Default value*: none

* **altitude**: Local altitude in meters for which the equivalent sea level pressure should be calculated.
  * *Valid values*: decimal number
  * *Default value*: none

#### Returns
Sea level barometric pressure.

#### See also
[getAltitude()](#getAltitude)

[Back to interface](#interface)


<a id="getAltitude"></a>

## getAltitude()

#### Description
The method calculates local altitude from provided local barometric pressure and corresponding sea level pressure.
* The input local pressure is usually just measured one by the method [measurePressure()](#measurePressure), but the method calculates in general.
* Both input pressures should be in the same measurement unit. However that unit can be arbitrary, usually Pascal or hectoPascal.

#### Syntax
    float getAltitude(float pressure, float pressureSea)

#### Parameters
* **pressure**: Local barometric pressure in arbitrary measurement unit, usually Pascal or hectoPascal.
  * *Valid values*: decimal number
  * *Default value*: none

* **pressureSea**: Sea level barometric pressure in arbitrary measurement unit, but the same as the first argument has.
  * *Valid values*: decimal number
  * *Default value*: none

#### Returns
Altitude in meters.

#### See also
[getPressureSea()](#getPressureSea)

[Back to interface](#interface)


<a id="getOversampling"></a>

## getOversampling()

#### Description
The method provides code of currently set oversampling rate for pressure measuring.

#### Syntax
    uint8_t getOversampling()

#### Parameters
None

#### Returns
Code of oversampling rate in the range 0 ~ 3.

#### See also
[setOversampling()](#setOversampling)

[Back to interface](#interface)


<a id="getErrorValue"></a>

## getErrorValue()

#### Description
The method returns virtually wrong temperature or barometric pressure value at erroneous measurement usually at failure of two-wire bus.

#### Syntax
    float getErrorValue()

#### Parameters
None

#### Returns
Erroneous temperature and/or barometric pressure value.
The error code can be tested in the operational code with the method [getLastResult()](#getLastResult), [isError()](#isError), or [isSuccess()](#isSuccess).

#### See also
[measureTemperature()](#measureTemperature)

[measurePressure()](#measurePressure)

[Back to interface](#interface)
