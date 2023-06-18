/*
  NAME:
  gbjBMP180

  DESCRIPTION:
  Library for temperature and pressure sensor BMP180 on two-wire (I2C) bus.
  - Sensor contains individual calibration data in its EEPROM, that should be
  read at every start after power-up.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the license GNU GPL v3
  http://www.gnu.org/licenses/gpl-3.0.html (related to original code) and MIT
  License (MIT) for added code.

  CREDENTIALS:
  Author: Libor Gabaj
  GitHub: https://github.com/mrkaleArduinoLib/gbj_bmp180.git
*/
#ifndef GBJ_BMP180_H
#define GBJ_BMP180_H

#include "gbj_twowire.h"

class gbj_bmp180 : public gbj_twowire
{
public:
  gbj_bmp180(ClockSpeeds clockSpeed = ClockSpeeds::CLOCK_100KHZ,
             uint8_t pinSDA = 4,
             uint8_t pinSCL = 5)
    : gbj_twowire(clockSpeed, pinSDA, pinSCL){};

  /*
    Initialize sensor

    DESCRIPTION:
    The method check communicatioin and functionality of the sensor and reads
    calibration data from EEPROM.

    PARAMETERS: None

    RETURN: Result code
  */
  inline ResultCodes begin()
  {
    if (isError(gbj_twowire::begin()))
    {
      return getLastResult();
    }
    if (isError(setAddress(Addresses::ADDRESS)))
    {
      return getLastResult();
    }
    if (isError(checkChipId()))
    {
      return getLastResult();
    }
#ifndef GBJ_BMP180_TEST
    if (isError(readCalibration()))
    {
      return getLastResult();
    }
#endif
    return getLastResult();
  }

  /*
    Reset sensor.

    DESCRIPTION:
    The method resets sensor to power-up state.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes reset()
  {
    return busSend(Commands::CMD_REG_RESET, Params::PARAM_RESET);
  }

  /*
    Measure temperature.

    DESCRIPTION:
    The method initiate conversion, reads the uncompensated temperature, and
    calculates its value temperature with help the calibration table.

    PARAMETERS: none

    RETURN: Temperature in centigrades or bad measure value
  */
  inline float measureTemperature()
  {
#ifndef GBJ_BMP180_TEST
    if (isError(readTemperature()))
    {
      return getErrorPT();
    }
#endif
    int32_t b5, t;
    b5 = getB5();
    t = (b5 + 8) >> 4;
#ifdef GBJ_BMP180_TEST
    Serial.println();
    Serial.println("Temperature");
    Serial.println("b5: " + String(b5));
    Serial.println("t: " + String(t));
#endif
    return t * 0.1;
  }

  /*
    Measure pressure and temperature.

    DESCRIPTION:
    The method initiate temperature and pressure conversion in this order, reads
    the their uncompensated values, and calculates them to real values with help
    the calibration table.

    PARAMETERS:
    temperature - Referenced variable for placing a temperature value.
      - Data type: float
      - Default value: none
      - Limited range: sensor specific

    RETURN: Pressure in pascals or bad measure value
  */
  inline float measurePressure(float &temperature)
  {
    int32_t x1, x2, x3, b5, b6, b3, p;
    uint32_t b4, b7;
    // Temperature measurement
    temperature = measureTemperature();
    if (isError())
    {
      return getErrorPT();
    }
    b5 = getB5();
    // Pressure measurement
#ifdef GBJ_BMP180_TEST
    setOversamplingLow();
#else
    if (isError(readPressure()))
    {
      return getErrorPT();
    }
#endif
    b6 = b5 - 4000;
    x1 = b6 * b6;
    x1 >>= 12;
    x1 *= status_.calibration[CalibrationIndexes::CALIB_IDX_B2];
    x1 >>= 11;
    x2 = b6 * status_.calibration[CalibrationIndexes::CALIB_IDX_AC2];
    x2 >>= 11;
    x3 = x1 + x2;
    b3 = 4 * static_cast<int32_t>(
               status_.calibration[CalibrationIndexes::CALIB_IDX_AC1]);
    b3 += x3;
    b3 <<= status_.oss;
    b3 += 2;
    b3 >>= 2;
#ifdef GBJ_BMP180_TEST
    Serial.println();
    Serial.println("Pressure");
    Serial.println("UP: " + String(status_.pressure));
    Serial.println("b6: " + String(b6));
    Serial.println("x1: " + String(x1));
    Serial.println("x2: " + String(x2));
    Serial.println("x3: " + String(x3));
    Serial.println("b3: " + String(b3));
#endif
    x1 = b6 * static_cast<int32_t>(
                status_.calibration[CalibrationIndexes::CALIB_IDX_AC3]);
    x1 >>= 13;
    x2 = b6 * b6;
    x2 >>= 12;
    x2 *= status_.calibration[CalibrationIndexes::CALIB_IDX_B1];
    x2 >>= 16;
    x3 = x1 + x2 + 2;
    x3 >>= 2;
    b4 = static_cast<uint16_t>(
      status_.calibration[CalibrationIndexes::CALIB_IDX_AC4]);
    b4 *= static_cast<uint32_t>(x3 + 32768);
    b4 >>= 15;
    b7 = static_cast<uint32_t>(status_.pressure) - b3;
    b7 *= 50000UL >> status_.oss;
    if (b7 < 0x80000000UL)
    {
      p = (b7 << 1) / b4;
    }
    else
    {
      p = (b7 / b4) << 1;
    }
#ifdef GBJ_BMP180_TEST
    Serial.println("x1: " + String(x1));
    Serial.println("x2: " + String(x2));
    Serial.println("x3: " + String(x3));
    Serial.println("b4: " + String(b4));
    Serial.println("b7: " + String(b7));
    Serial.println("p: " + String(p));
#endif
    x1 = (p >> 8);
    x1 *= x1;
#ifdef GBJ_BMP180_TEST
    Serial.println("x1: " + String(x1));
#endif
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;
#ifdef GBJ_BMP180_TEST
    Serial.println("x1: " + String(x1));
    Serial.println("x2: " + String(x2));
    Serial.println("p: " + String(p));
#endif
    return p;
  }

  // Setters
  inline void setOversamplingLow() { status_.oss = Oversampling::OSS_LOW; }
  inline void setOversamplingStandard()
  {
    status_.oss = Oversampling::OSS_STANDARD;
  }
  inline void setOversamplingHigh() { status_.oss = Oversampling::OSS_HIGH; }
  inline void setOversamplingHighUltra()
  {
    status_.oss = Oversampling::OSS_HIGH_ULTRA;
  }

  // Getters
  inline float getPressureSea(float pressure, float altitude)
  {
    return pressure / pow(1.0 - altitude / 44330.0, 5.255);
  }
  inline float getAltitude(float pressure, float pressureSea)
  {
    return 44330.0 * (1.0 - pow(pressure / pressureSea, (1.0 / 5.255)));
  }
  inline int16_t *getCalibration(uint8_t &items)
  {
    items = Params::PARAM_CALIB_CNT;
    return status_.calibration;
  }
  inline float getErrorPT() { return -999; }

private:
  enum Addresses
  {
    // Hardware address
    ADDRESS = 0x77,
  };
  enum Commands : uint8_t
  {
    // Register with chip identifier
    CMD_REG_CHIPID = 0xD0,
    // Register with the first calibration byte
    CMD_REG_CALIB_LSB = 0xAA,
    // Register with the last calibration byte
    CMD_REG_CALIB_MSB = 0xBF,
    // Register for software reset
    CMD_REG_RESET = 0xE0,
    // Register for measurement control
    CMD_REG_CONTROL = 0xF4,
    // Register with MSB of ADC output value
    CMD_REG_OUT_MSB = 0xF6,
    // Register with LSB of ADC output value
    CMD_REG_OUT_LSB = 0xF7,
    // Register with XLSB of ADC output value
    CMD_REG_OUT_XLSB = 0xF8,
  };
  // Maximal times in milliseconds
  enum Timing : uint8_t
  {
    // Temperature (4.5 ms by datasheet)
    TIMING_CONVERSION_TEMPERATURE = 5,
    // Pressure - ultra low power (4.5 ms by datasheet)
    TIMING_CONVERSION_PRESSURE_LOW = 5,
    // Pressure - standard (7.5 ms by datasheet)
    TIMING_CONVERSION_PRESSURE_STANDARD = 8,
    // Pressure - high resolution (13.5 ms by datasheet)
    TIMING_CONVERSION_PRESSURE_HIGH = 14,
    // Pressure - high resolution (25.5 ms by datasheet)
    TIMING_CONVERSION_PRESSURE_HIGH_ULTRA = 26,
  };
  enum Oversampling : uint8_t
  {
    OSS_LOW,
    OSS_STANDARD,
    OSS_HIGH,
    OSS_HIGH_ULTRA,
  };
  enum ControlValues : uint8_t
  {
    // Temperature
    CONTROL_VALUE_TEMPERATURE = 0x2E,
    // Pressure
    CONTROL_VALUE_PRESSURE = 0x34,
  };
  enum ControlBits : uint8_t
  {
    // Start of conversion
    CONTROL_BIT_SCO = 5,
    // Oversampling setting
    CONTROL_BIT_OSS = 6,
  };
  enum Params : uint8_t
  {
    // Hardcoded identification of the sensor
    PARAM_ID = 0x55,
    // Number of calibration parameters
    PARAM_CALIB_CNT = 11,
    // Software reset value
    PARAM_RESET = 0xB6,
  };
  enum CalibrationIndexes : uint8_t
  {
    CALIB_IDX_MD,
    CALIB_IDX_MC,
    CALIB_IDX_MB,
    CALIB_IDX_B2,
    CALIB_IDX_B1,
    CALIB_IDX_AC6,
    CALIB_IDX_AC5,
    CALIB_IDX_AC4,
    CALIB_IDX_AC3,
    CALIB_IDX_AC2,
    CALIB_IDX_AC1,
  };
  struct Status
  {
    // Calibration table
#ifdef GBJ_BMP180_TEST
    int16_t calibration[Params::PARAM_CALIB_CNT] = {
      2868, -8711, -32768, 4, 6190, 23153, 32757, 32741, -14383, -72, 408,
    };
#else
    int16_t calibration[Params::PARAM_CALIB_CNT];
#endif
    // Oversampling mode
    Oversampling oss;
    // Uncompensated temperature
#ifdef GBJ_BMP180_TEST
    int32_t temperature = 27898;
#else
    int32_t temperature;
#endif
    // Uncompensated pressure
#ifdef GBJ_BMP180_TEST
    int32_t pressure = 23843;
#else
    int32_t pressure;
#endif
  } status_;
  inline uint8_t getConversionTemperatureTime()
  {
    return Timing::TIMING_CONVERSION_TEMPERATURE;
  }
  inline uint8_t getConversionPressureTime()
  {
    uint8_t ossTime[] = {
      Timing::TIMING_CONVERSION_PRESSURE_LOW,
      Timing::TIMING_CONVERSION_PRESSURE_STANDARD,
      Timing::TIMING_CONVERSION_PRESSURE_HIGH,
      Timing::TIMING_CONVERSION_PRESSURE_HIGH_ULTRA,
    };
    return ossTime[status_.oss];
  }
  inline int32_t getB5()
  {
    int32_t x1, x2;
    x1 = status_.temperature -
         static_cast<uint16_t>(
           status_.calibration[CalibrationIndexes::CALIB_IDX_AC6]);
    x1 *= static_cast<uint16_t>(
      status_.calibration[CalibrationIndexes::CALIB_IDX_AC5]);
    x1 >>= 15;
    x2 = static_cast<int32_t>(
      status_.calibration[CalibrationIndexes::CALIB_IDX_MC]);
    x2 <<= 11;
    x2 /= x1 + status_.calibration[CalibrationIndexes::CALIB_IDX_MD];
#ifdef GBJ_BMP180_TEST
    Serial.println();
    Serial.println("B5");
    Serial.println("UT: " + String(status_.temperature));
    Serial.println("x1: " + String(x1));
    Serial.println("x2: " + String(x2));
#endif
    return x1 + x2;
  }

  /*
    Read chip id and compare it to the predefined value.

    DESCRIPTION:
    The checks communication with the sensor.

    PARAMETERS: None

    RETURN: Result code
  */
  inline ResultCodes checkChipId()
  {
    uint8_t chipid;
    if (isError(busSend(Commands::CMD_REG_CHIPID)))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    if (isError(busReceive(&chipid, sizeof(chipid))))
    {
      return setLastResult(ResultCodes::ERROR_RCV_DATA);
    }
    if (chipid != Params::PARAM_ID)
    {
      return setLastResult(ResultCodes::ERROR_SN);
    }
    return getLastResult();
  }
  /*
    Read calibration table from EEPROM.

    DESCRIPTION:
    The method checks each calibration word for valid range value.

    PARAMETERS: None

    RETURN: Result code
  */
  inline ResultCodes readCalibration()
  {
    setLastResult();
    // Read table in reverse order due to MSB first
    if (isError(busSend(Commands::CMD_REG_CALIB_LSB)))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    if (isError(busReceive(reinterpret_cast<uint8_t *>(status_.calibration),
                           Params::PARAM_CALIB_CNT * 2,
                           REVERSE)))
    {
      return setLastResult(ResultCodes::ERROR_RCV_DATA);
    }
    // Check table
    for (uint8_t i = 0; i < Params::PARAM_CALIB_CNT; i++)
    {
      if (status_.calibration[i] == 0 ||
          status_.calibration[i] == static_cast<int16_t>(0xFFFF))
      {
        return setLastResult(ResultCodes::ERROR_BUFFER);
      }
    }
    return getLastResult();
  }
  /*
    Wait for conversion.

    DESCRIPTION:
    The method waits for input milliseconds and then checks the conversion SCO
    bit in the control register to be released.

    PARAMETERS:
    delay - Waiting time period in milliseconds.
      - Data type: non-negative integer
      - Default value: none
      - Limited range: 0 ~ 2^32 - 1

    RETURN: Result code
  */
  inline ResultCodes waitConversion(unsigned long delay)
  {
    uint8_t state, attemps = 50;
    wait(delay);
    // Check end of conversion for sure
    while (attemps)
    {
      // Read control register
      if (isError(busReceive(Commands::CMD_REG_CONTROL, &state, sizeof(state))))
      {
        return setLastResult(ResultCodes::ERROR_REGISTER);
      }
      // Check conversion bit on '0'
      if ((state & (1 << ControlBits::CONTROL_BIT_SCO)) == 0)
      {
        break;
      }
      wait(1);
      attemps--;
    }
    return attemps ? getLastResult()
                   : setLastResult(ResultCodes::ERROR_MEASURE);
  }
  /*
    Read uncompensated temperature.

    PARAMETERS: None

    RETURN: Result code
  */
  inline ResultCodes readTemperature()
  {
    uint16_t buffer16;
    if (isError(busSend(Commands::CMD_REG_CONTROL,
                        ControlValues::CONTROL_VALUE_TEMPERATURE)))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    if (isError(waitConversion(getConversionTemperatureTime())))
    {
      return getLastResult();
    }
    if (isError(busReceive(Commands::CMD_REG_OUT_MSB,
                           reinterpret_cast<uint8_t *>(&buffer16),
                           sizeof(buffer16),
                           REVERSE)))
    {
      return setLastResult(ResultCodes::ERROR_RCV_DATA);
    }
    status_.temperature = buffer16;
    return getLastResult();
  }
  /*
    Read uncompensated pressure.

    PARAMETERS: None

    RETURN: Result code
  */
  inline ResultCodes readPressure()
  {
    uint8_t reg = ControlValues::CONTROL_VALUE_PRESSURE +
                  (status_.oss << ControlBits::CONTROL_BIT_OSS);
    uint8_t buffer8;
    uint16_t buffer16;
    if (isError(busSend(Commands::CMD_REG_CONTROL, reg)))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    if (isError(waitConversion(getConversionPressureTime())))
    {
      return getLastResult();
    }
    if (isError(busReceive(Commands::CMD_REG_OUT_MSB,
                           reinterpret_cast<uint8_t *>(&buffer16),
                           sizeof(buffer16),
                           REVERSE)))
    {
      return setLastResult(ResultCodes::ERROR_RCV_DATA);
    }
    if (isError(
          busReceive(Commands::CMD_REG_OUT_XLSB, &buffer8, sizeof(buffer8))))
    {
      return setLastResult(ResultCodes::ERROR_RCV_DATA);
    }
    status_.pressure = static_cast<uint32_t>(buffer16) << 8;
    status_.pressure += buffer8;
    status_.pressure >>= (8 - status_.oss);
    return getLastResult();
  }
};

#endif
