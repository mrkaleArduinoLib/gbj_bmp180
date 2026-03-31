/**
 * @file gbj_bmp180.h
 * @brief Library for temperature and pressure sensor BMP180 on two-wire (I2C)
 * bus.
 * @details Sensor contains individual calibration data in its EEPROM, that
 * should be read at every start after power-up.
 *
 * @copyright This program is free software; you can redistribute it and/or
 * modify it under the terms of the license GNU GPL v3
 * http://www.gnu.org/licenses/gpl-3.0.html (related to original code) and MIT
 * License (MIT) for added code.
 *
 * @author Libor Gabaj
 * @see https://github.com/mrkaleArduinoLib/gbj_bmp180.git
 */
#ifndef GBJ_BMP180_H
#define GBJ_BMP180_H

#include "gbj_twowire.h"

/**
 * @class gbj_bmp180
 * @brief BMP180 temperature and pressure sensor driver.
 * @details Driver for Bosch BMP180 barometric pressure and temperature sensor
 * connected via two-wire (I2C) bus. Provides measurement methods with internal
 * calibration compensation and altitude calculation utilities.
 */
class gbj_bmp180 : public gbj_twowire
{
public:
  /**
   * @brief Oversampling settings for pressure measurement.
   */
  enum class Oversamplings : uint8_t
  {
    ULTRA_LOW_POWER_1X = B00,
    STANDARD_RESOLUTION_2X = B01,
    HIGH_RESOLUTION_4X = B10,
    ULTRA_HIGH_RESOLUTION_8X = B11,
  };
  gbj_bmp180(ClockSpeeds clockSpeed = ClockSpeeds::CLOCK_100KHZ,
             uint8_t pinSDA = 4,
             uint8_t pinSCL = 5)
    : gbj_twowire(clockSpeed, pinSDA, pinSCL) {};

  /**
   * @brief Initialize sensor.
   * @details Checks communication and functionality of the sensor and reads
   * calibration data from EEPROM.
   * @return Result code
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
    if (isError(reset()))
    {
      return getLastResult();
    }
    return getLastResult();
  }

  /**
   * @brief Reset sensor to power-up state.
   * @return Result code
   */
  inline ResultCodes reset()
  {
    if (isError(busSend(Commands::CMD_REG_RESET, Params::PARAM_RESET)))
    {
      return getLastResult();
    }
    wait(Timing::TIMING_STARTUP);
    setOversampling(Oversamplings::STANDARD_RESOLUTION_2X);
    return getLastResult();
  }

  /**
   * @brief Measure temperature.
   * @details Initiates conversion, reads the uncompensated temperature, and
   * calculates its value with help of the calibration table.
   * @details The separate usage of this method is useful for very frequent
   * pressure measurements, where the temperature is used only for compensation,
   * e.g., every 1 second, and does not need to be measured every time.
   * @return Temperature in centigrades or bad measure value.
   */
  inline float measureTemperature()
  {
#ifndef GBJ_BMP180_TEST
    if (isError(readTemperature()))
    {
      return getErrorValue();
    }
#endif
    calculateTemperatureFine();
    return compensateTemperature();
  }

  /**
   * @brief Measure pressure without temperature compensation.
   * @details Initiates pressure conversion only, reads the uncompensated value,
   * and calculates it to real value with help of the calibration table.
   * @details This method is useful for very frequent pressure measurements,
   * where the temperature has been measured previously.
   * @return Pressure in pascals or bad measure value.
   */
  inline float measurePressureOnly()
  {
#ifndef GBJ_BMP180_TEST
    if (isError(readPressure()))
    {
      return getErrorValue();
    }
#endif
    return compensatePressure();
  }

  /**
   * @brief Measure pressure.
   * @details Initiates temperature and pressure conversion in this order, reads
   * their uncompensated values, and calculates them to real values with help
   * of the calibration table.
   * @return Pressure in pascals or bad measure value.
   */
  inline float measurePressure()
  {
#ifndef GBJ_BMP180_TEST
    if (isError(readTemperature()))
    {
      return getErrorValue();
    }
    if (isError(readPressure()))
    {
      return getErrorValue();
    }
#endif
    calculateTemperatureFine();
    return compensatePressure();
  }

  /**
   * @brief Measure pressure and temperature.
   * @details Initiates temperature and pressure conversion in this order, reads
   * their uncompensated values, and calculates them to real values with help
   * of the calibration table.
   * @param[out] temperature Temperature in centigrades.
   * @return Pressure in pascals or bad measure value.
   */
  inline float measurePressure(float &temperature)
  {
    temperature = measureTemperature();
    return measurePressure();
  }

  /** @name Setters */
  ///@{

  /**
   * @brief Set oversampling to ultra low power (1x).
   */
  inline void setOversamplingLow()
  {
    setOversampling(Oversamplings::ULTRA_LOW_POWER_1X);
  }

  /**
   * @brief Set oversampling to standard resolution (2x).
   */
  inline void setOversamplingStandard()
  {
    setOversampling(Oversamplings::STANDARD_RESOLUTION_2X);
  }

  /**
   * @brief Set oversampling to high resolution (4x).
   */
  inline void setOversamplingHigh()
  {
    setOversampling(Oversamplings::HIGH_RESOLUTION_4X);
  }

  /**
   * @brief Set oversampling to ultra high resolution (8x).
   */
  inline void setOversamplingHighUltra()
  {
    setOversampling(Oversamplings::ULTRA_HIGH_RESOLUTION_8X);
  }

  /**
   * @brief Set oversampling mode.
   * @details Constrains the oversampling value to valid range. Higher
   * oversampling values provide higher resolution but longer measurement times.
   * @param oss Oversampling setting value.
   */
  inline void setOversampling(Oversamplings oss)
  {
    status_.oss = static_cast<Oversamplings>(
      constrain(oss,
                Oversamplings::ULTRA_LOW_POWER_1X,
                Oversamplings::ULTRA_HIGH_RESOLUTION_8X));
  }

  ///@}

  /** @name Getters */
  ///@{

  /**
   * @brief Get error return value.
   * @return Error value marker (-999).
   */
  inline float getErrorValue() { return -999; }

  /**
   * @brief Calculate pressure at sea level.
   * @details Calculates equivalent sea level pressure from measured pressure
   * and altitude using barometric formula.
   * @param pressure Measured pressure in Pa.
   * @param altitude Altitude in meters.
   * @return Pressure at sea level in Pa.
   */
  inline float getPressureSea(float pressure, float altitude)
  {
    return pressure / pow(1.0 - altitude / 44330.0, 5.255);
  }

  /**
   * @brief Calculate altitude.
   * @details Calculates altitude from measured pressure and sea level pressure
   * using barometric formula.
   * @param pressure Measured pressure in Pa.
   * @param pressureSea Sea level reference pressure in Pa.
   * @return Altitude in meters.
   */
  inline float getAltitude(float pressure, float pressureSea)
  {
    return 44330.0 * (1.0 - pow(pressure / pressureSea, (1.0 / 5.255)));
  }

  /**
   * @brief Get calibration table.
   * @details Retrieves pointer to the calibration coefficients array.
   * @param[out] items Number of calibration parameters.
   * @return Pointer to calibration coefficients.
   */
  inline uint16_t *getCalibration(uint8_t &items)
  {
    items = Params::PARAM_CALIB_CNT;
    return reinterpret_cast<uint16_t *>(&calibration_);
  }

  /**
   * @brief Get current oversampling mode.
   * @return Oversampling setting.
   */
  inline Oversamplings getOversampling() { return status_.oss; }

  /**
   * @brief Get text representation of oversampling mode.
   * @param oss Oversampling setting value.
   * @return Human-readable string label for the oversampling mode.
   */
  inline String txtOversampling(Oversamplings oss)
  {
    String labels[] = {
      "UltraLowPower(1x)",
      "Standard(2x)",
      "HighResolution(4x)",
      "UltraHighResolution(8x)",
    };
    return labels[static_cast<uint8_t>(
      constrain(oss,
                Oversamplings::ULTRA_LOW_POWER_1X,
                Oversamplings::ULTRA_HIGH_RESOLUTION_8X))];
  }

  ///@}

private:
  enum Addresses
  {
    /// Hardware address (hexadecimal: 0x77, binary: 0b1110111)
    ADDRESS = 0x77,
  };

  enum Commands : uint8_t
  {
    /// Register with chip identifier
    CMD_REG_CHIPID = 0xD0,
    /// Register with the first calibration byte
    CMD_REG_CALIB_LSB = 0xAA,
    /// Register with the last calibration byte
    CMD_REG_CALIB_MSB = 0xBF,
    /// Register for software reset
    CMD_REG_RESET = 0xE0,
    /// Register for measurement control
    CMD_REG_CONTROL = 0xF4,
    /// Register with MSB of ADC output value
    CMD_REG_OUT_MSB = 0xF6,
    /// Register with LSB of ADC output value
    CMD_REG_OUT_LSB = 0xF7,
    /// Register with XLSB of ADC output value
    CMD_REG_OUT_XLSB = 0xF8,
  };

  /// Maximal times in milliseconds
  enum Timing : uint8_t
  {
    /// Startup time after power-up
    TIMING_STARTUP = 10,
    /// Temperature (4.5 ms by datasheet)
    TIMING_CONVERSION_TEMPERATURE = 5,
    /// Pressure - ultra low power (4.5 ms by datasheet)
    TIMING_CONVERSION_PRESSURE_LOW = 5,
    /// Pressure - standard (7.5 ms by datasheet)
    TIMING_CONVERSION_PRESSURE_STANDARD = 8,
    /// Pressure - high resolution (13.5 ms by datasheet)
    TIMING_CONVERSION_PRESSURE_HIGH = 14,
    /// Pressure - high resolution (25.5 ms by datasheet)
    TIMING_CONVERSION_PRESSURE_HIGH_ULTRA = 26,
  };

  /// Pressure conversion times by oversampling mode.
  const uint8_t ossTime_[4] = {
    Timing::TIMING_CONVERSION_PRESSURE_LOW,
    Timing::TIMING_CONVERSION_PRESSURE_STANDARD,
    Timing::TIMING_CONVERSION_PRESSURE_HIGH,
    Timing::TIMING_CONVERSION_PRESSURE_HIGH_ULTRA,
  };

  enum ControlValues : uint8_t
  {
    /// Temperature
    CONTROL_VALUE_TEMPERATURE = 0x2E,
    /// Pressure
    CONTROL_VALUE_PRESSURE = 0x34,
  };

  enum ControlBits : uint8_t
  {
    /// Start of conversion bit in F4 register position and mask
    CONTROL_BIT_SCO_POS = 5,
    CONTROL_BIT_SCO_MSK = B1,
    /// Oversampling setting bits in F4 register position and mask
    CONTROL_BIT_OSS_POS = 6,
    CONTROL_BIT_OSS_MSK = B11,
  };

  enum Params : uint8_t
  {
    /// Hardcoded identification of the sensor
    PARAM_ID = 0x55,
    /// Number of calibration parameters
    PARAM_CALIB_CNT = 11,
    /// Software reset value
    PARAM_RESET = 0xB6,
  };

  struct Status
  {
    /// Oversampling mode
    Oversamplings oss;
    /// Uncompensated temperature (default from datasheet example)
    int32_t temperature = 27898;
    /// Uncompensated pressure (default from datasheet example)
    int32_t pressure = 23843;
    /// Calculated calibration coefficient
    int32_t C_B5;
  } status_;

  /// Calibration coefficients (defaults from datasheet example)
  /// The order is reversed due to MSB first
  struct Calibration
  {
    int16_t C_MD = 2868;
    int16_t C_MC = -8711;
    int16_t C_MB = -32768;
    int16_t C_B2 = 4;
    int16_t C_B1 = 6190;
    uint16_t C_AC6 = 23153;
    uint16_t C_AC5 = 32757;
    uint16_t C_AC4 = 32741;
    int16_t C_AC3 = -14383;
    int16_t C_AC2 = -72;
    int16_t C_AC1 = 408;
  } calibration_;

  /**
   * @brief Get conversion time for temperature measurement.
   * @return Conversion time in milliseconds.
   */
  inline uint8_t getConversionTemperatureTime()
  {
    return Timing::TIMING_CONVERSION_TEMPERATURE;
  }

  /**
   * @brief Get conversion time for pressure measurement.
   * @details Conversion time depends on the current oversampling mode.
   * @return Conversion time in milliseconds.
   */
  inline uint8_t getConversionPressureTime()
  {
    return ossTime_[static_cast<uint8_t>(status_.oss)];
  }

  /**
   * @brief Read chip ID and compare it to the predefined value.
   * @details Checks communication with the sensor.
   * @return Result code
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

  /**
   * @brief Read calibration table from EEPROM.
   * @details Checks each calibration word for valid range value.
   * @return Result code
   */
  inline ResultCodes readCalibration()
  {
    setLastResult();
    // Read table in reverse order due to MSB first
    if (isError(busSend(Commands::CMD_REG_CALIB_LSB)))
    {
      return setLastResult(ResultCodes::ERROR_REGISTER);
    }
    if (isError(busReceive(reinterpret_cast<uint8_t *>(&calibration_),
                           Params::PARAM_CALIB_CNT * 2,
                           REVERSE)))
    {
      return setLastResult(ResultCodes::ERROR_RCV_DATA);
    }
    // Check table
    byte calibCnt;
    uint16_t *calibTable = getCalibration(calibCnt);
    for (uint8_t i = 0; i < Params::PARAM_CALIB_CNT; i++)
    {
      if (calibTable[i] == 0 || calibTable[i] == static_cast<uint16_t>(0xFFFF))
      {
        return setLastResult(ResultCodes::ERROR_BUFFER);
      }
    }
    return getLastResult();
  }

  /**
   * @brief Wait for ADC conversion to complete.
   * @details Waits for the given milliseconds and then checks the conversion
   * SCO bit in the control register to be released.
   * @param delay Waiting time period in milliseconds. Range: 0 ~ 2^32 - 1.
   * @return Result code
   */
  inline ResultCodes waitConversion(unsigned long delay)
  {
    uint8_t state, attempts = 10;
    // Check end of conversion
    while (attempts)
    {
      wait(delay);
      // Read control register
      if (isError(busReceive(Commands::CMD_REG_CONTROL, &state, sizeof(state))))
      {
        return getLastResult();
      }
      // Check conversion bit
      if (((state >> ControlBits::CONTROL_BIT_SCO_POS) &
           ControlBits::CONTROL_BIT_SCO_MSK))
      {
        attempts--;
        continue;
      }
      break;
    }
    return attempts ? getLastResult()
                    : setLastResult(ResultCodes::ERROR_MEASURE);
  }

  /**
   * @brief Read uncompensated temperature from sensor.
   * @return Result code
   */
  inline ResultCodes readTemperature()
  {
    uint8_t buffer[2];
    setBusRepeat();
    if (isError(busSend(Commands::CMD_REG_CONTROL,
                        ControlValues::CONTROL_VALUE_TEMPERATURE)))
    {
      return getLastResult();
    }
    if (isError(waitConversion(getConversionTemperatureTime())))
    {
      return getLastResult();
    }
    if (isError(busReceive(Commands::CMD_REG_OUT_MSB,
                           buffer,
                           sizeof(buffer) / sizeof(buffer[0]))))
    {
      return getLastResult();
    }
    setBusStop();
    // MSB (F6)
    status_.temperature = buffer[0];
    status_.temperature <<= 8;
    // LSB (F7)
    status_.temperature |= buffer[1];
    return getLastResult();
  }

  /**
   * @brief Read uncompensated pressure from sensor.
   * @return Result code
   */
  inline ResultCodes readPressure()
  {
    uint8_t buffer[3];
    uint8_t reg =
      ControlValues::CONTROL_VALUE_PRESSURE +
      (static_cast<uint8_t>(status_.oss) << ControlBits::CONTROL_BIT_OSS_POS);
    setBusRepeat();
    if (isError(busSend(Commands::CMD_REG_CONTROL, reg)))
    {
      return getLastResult();
    }
    if (isError(waitConversion(getConversionPressureTime())))
    {
      return getLastResult();
    }
    if (isError(busReceive(Commands::CMD_REG_OUT_MSB,
                           buffer,
                           sizeof(buffer) / sizeof(buffer[0]))))
    {
      return getLastResult();
    }
    setBusStop();
    // MSB (F6)
    status_.pressure = buffer[0];
    status_.pressure <<= 8;
    // LSB (F7)
    status_.pressure |= buffer[1];
    status_.pressure <<= 8;
    // XLSB (F8)
    status_.pressure |= buffer[2];
    status_.pressure >>= (8 - static_cast<uint8_t>(status_.oss));
    return getLastResult();
  }

  /**
   * @brief Calculate fine temperature value.
   * @details This value is used for pressure compensation.
   */
  inline void calculateTemperatureFine()
  {
    int32_t x1, x2;
    x1 = status_.temperature - calibration_.C_AC6;
    x1 *= calibration_.C_AC5;
    x1 >>= 15;
    x2 = static_cast<int32_t>(calibration_.C_MC);
    x2 <<= 11;
    x2 /= x1 + calibration_.C_MD;
    status_.C_B5 = x1 + x2;
#ifdef GBJ_BMP180_TEST
    Serial.println();
    Serial.println("Tfine");
    Serial.println("UT: " + String(status_.temperature));
    Serial.println("x1: " + String(x1));
    Serial.println("x2: " + String(x2));
    Serial.println("B5: " + String(status_.C_B5));
#endif
  }

  /**
   * @brief Compensate temperature value.
   * @return Compensated temperature in degrees Celsius.
   */
  inline float compensateTemperature()
  {
    int32_t t;
    t = status_.C_B5 + 8;
    t >>= 4;
#ifdef GBJ_BMP180_TEST
    Serial.println();
    Serial.println("Temperature");
    Serial.println("t: " + String(t));
#endif
    return t * 0.1;
  }

  /**
   * @brief Compensate pressure value.
   * @return Compensated pressure in Pa.
   */
  inline float compensatePressure()
  {
    int32_t x1, x2, x3, b6, b3, p;
    uint32_t b4, b7;
#ifdef GBJ_BMP180_TEST
    setOversampling(Oversamplings::ULTRA_LOW_POWER_1X);
#endif
    b6 = status_.C_B5 - 4000;
    x1 = b6 * b6;
    x1 >>= 12;
    x1 *= calibration_.C_B2;
    x1 >>= 11;
    x2 = b6 * calibration_.C_AC2;
    x2 >>= 11;
    x3 = x1 + x2;
    b3 = 4 * static_cast<int32_t>(calibration_.C_AC1);
    b3 += x3;
    b3 <<= static_cast<uint8_t>(status_.oss);
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
    x1 = b6 * static_cast<int32_t>(calibration_.C_AC3);
    x1 >>= 13;
    x2 = b6 * b6;
    x2 >>= 12;
    x2 *= calibration_.C_B1;
    x2 >>= 16;
    x3 = x1 + x2 + 2;
    x3 >>= 2;
    b4 = calibration_.C_AC4;
    b4 *= static_cast<uint32_t>(x3 + 32768);
    b4 >>= 15;
    b7 = static_cast<uint32_t>(status_.pressure) - b3;
    b7 *= 50000UL >> static_cast<uint8_t>(status_.oss);
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
};

#endif
