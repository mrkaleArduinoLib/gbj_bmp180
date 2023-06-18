/*
  NAME:
  Measurement the barometric preasure and recalculating it to sea level.

  DESCRIPTION:
  - Altitude input in meters.
  - Sea level pressure output in hectopascals.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_bmp180.h"

const float ALTITUDE = 134.0; // Bratislava, Slovakia
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
    Serial.println("Altitude: " + String(ALTITUDE, 1) + " m");
    Serial.println("Pressure: " + String(pressValue / 100, 2) + " hPa");
    Serial.println(
      "PressureSea: " +
      String(sensor.getPressureSea(pressValue, ALTITUDE) / 100, 2) + " hPa");
    Serial.println();
  }
}

void loop() {}
