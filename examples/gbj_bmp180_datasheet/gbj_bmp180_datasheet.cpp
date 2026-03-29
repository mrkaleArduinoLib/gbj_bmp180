/**
 * @name gbj_bmp180_datasheet
 *
 * @brief Testing BMP180's calculation algorithms with datasheet constants.
 *
 * @note The preprocessor GBJ_BMP180_TEST macro should be defined before
 * including library header file.
 * @note In testing mode no data reading is provided.
 * @note In testing mode the library writes to serial port internally.
 *
 * @copyright This program is free software; you can redistribute it and/or
 * modify it under the terms of the MIT License (MIT).
 *
 * @author Libor Gabaj
 */
#define GBJ_BMP180_TEST
#include "gbj_bmp180.h"

float tempValue, pressValue;

// Software configuration
gbj_bmp180 sensor = gbj_bmp180();
// gbj_bmp180 sensor = gbj_bmp180(sensor.CLOCK_100KHZ, D2, D1);
// gbj_bmp180 sensor = gbj_bmp180(sensor.CLOCK_400KHZ);

void errorHandler(String location)
{
  Serial.println(sensor.getLastErrorTxt(location));
  Serial.println("---");
  return;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("===");

  // Initialize sensor
  if (sensor.isError(sensor.begin()))
  {
    errorHandler("Begin");
    return;
  }
  // Address
  Serial.print("Address: 0x");
  Serial.println(sensor.getAddress(), HEX);
  Serial.println("---");

  pressValue = sensor.measurePressure(tempValue);
  if (sensor.isError())
  {
    errorHandler("Measure");
  }
  else
  {
    Serial.println("Temperature (°C) / Pressure (hPa)");
    Serial.print(tempValue, 1);
    Serial.print(" / ");
    Serial.println(pressValue / 100, 2);
    Serial.println("===");
  }
}

void loop() {}
