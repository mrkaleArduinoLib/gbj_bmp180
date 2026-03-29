/**
 * @name gbj_bmp180_measure
 *
 * @brief BMP180 measuring barometric pressure and temperature at once.
 *
 * @note Temperature is in centigrades.
 * @note Pressure is in hectopascals.
 *
 * @copyright This program is free software; you can redistribute it and/or
 * modify it under the terms of the MIT License (MIT).
 *
 * @author Libor Gabaj
 */
#include "gbj_bmp180.h"

const unsigned int PERIOD_MEASURE = 3000;
float tempValue, pressValue;
bool flErrorBegin = false;

// Software configuration
gbj_bmp180 sensor = gbj_bmp180();
// gbj_bmp180 sensor = gbj_bmp180(sensor.CLOCK_100KHZ, D2, D1);
// gbj_bmp180 sensor = gbj_bmp180(sensor.CLOCK_400KHZ);

/**
 * @brief Print the sensor's last error text with operation location.
 *
 * @param location Label of operation related to the last error.
 */
void errorHandler(String location)
{
  Serial.println(sensor.getLastErrorTxt(location));
  Serial.println("---");
  return;
}

/**
 * @brief Initialize serial communication and the BMP180 sensor.
 */
void setup()
{
  Serial.begin(115200);
  Serial.println("---");

  // Initialize sensor
  if (sensor.isError(sensor.begin()))
  {
    errorHandler("Begin");
    flErrorBegin = true;
    return;
  }

  // Print sensor address
  Serial.print("Address: 0x");
  Serial.println(sensor.getAddress(), HEX);
  Serial.println("---");
  Serial.println("Oversampling pressure: " +
                 sensor.txtOversampling(sensor.getOversampling()));

  // Print table header
  Serial.println("Temperature (°C) / Pressure (hPa)");
}

/**
 * @brief Measure and print temperature and pressure periodically.
 */
void loop()
{
  // Finish program if sensor initialization failed
  if (flErrorBegin)
  {
    return;
  }

  // Measure pressure and the related temperature
  pressValue = sensor.measurePressure(tempValue);
  if (sensor.isError())
  {
    errorHandler("Measure");
  }
  else
  {
    Serial.print(tempValue, 1);
    Serial.print(" / ");
    Serial.println(pressValue / 100, 2);
  }
  delay(PERIOD_MEASURE);
}
