/*
  NAME:
  Testing calculation algorithms with datasheet constants.

  DESCRIPTION:
  - The preprocessor GBJ_BMP180_TEST macro should be defined before including
  library header file.
  - In testing mode no data reading is provided.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#define GBJ_BMP180_TEST
#include "gbj_bmp180.h"

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
  Serial.begin(9600);
  Serial.println("---");

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

  sensor.measureTemperature();
  if (sensor.isError())
  {
    errorHandler("Temperature");
  }
  sensor.measurePressure();
  if (sensor.isError())
  {
    errorHandler("Pressure");
  }
}

void loop() {}
