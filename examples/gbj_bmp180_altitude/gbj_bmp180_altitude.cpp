/*
  NAME:
  Measurement the barometric preasure and recalculating it to altitude.

  DESCRIPTION:
  - Sea level pressure input in hectopascals.
  - Altitude output in meters.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_bmp180.h"

const float PRESSURE_SEALEVEL = 1015.0; // in hPa; Bratislava, Slovakia
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
  // Measure
  pressValue = sensor.measurePressure(tempValue);
  if (sensor.isError())
  {
    errorHandler("Measure");
  }
  else
  {
    Serial.println("Temperature: " + String(tempValue, 1) + " °C");
    Serial.println("Pressure: " + String(pressValue / 100, 2) + " hPa");
    Serial.println(
      "Altitude: " +
      String(sensor.getAltitude(pressValue, PRESSURE_SEALEVEL * 100), 1) +
      " m");
    Serial.println();
  }
}

void loop() {}
