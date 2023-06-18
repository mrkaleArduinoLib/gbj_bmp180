/*
  NAME:
  Measurement the barometric pressure and temperature at once.

  DESCRIPTION:
  - Pressure is in hectopascals.
  - Temperature is in centigrades.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_bmp180.h"

const unsigned int PERIOD_MEASURE = 3000;
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

  // Print header
  Serial.println("Temperature (°C) / Pressure (hPa)");
}

void loop()
{
  // Measure
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
