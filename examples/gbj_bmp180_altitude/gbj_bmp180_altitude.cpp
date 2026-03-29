/**
 * @name gbj_bmp180_altitude
 *
 * @brief One-time barometric pressure measuring and recalculating it to
 * altitude.
 *
 * @note Sea level pressure input in hectopascals.
 * @note Altitude output in meters.
 *
 * @copyright This program is free software; you can redistribute it and/or
 * modify it under the terms of the MIT License (MIT).
 *
 * @author Libor Gabaj
 */
#include "gbj_bmp180.h"

const float PRESSURE_SEALEVEL = 1015.0; // in hPa; Bratislava, Slovakia
float pressValue;

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
  // Measure
  pressValue = sensor.measurePressure();
  if (sensor.isError())
  {
    errorHandler("Measure");
  }
  else
  {
    Serial.println("PressureSea: " + String(PRESSURE_SEALEVEL, 2) + " hPa");
    Serial.println("Pressure: " + String(pressValue / 100, 2) + " hPa");
    Serial.println(
      "Altitude: " +
      String(sensor.getAltitude(pressValue, PRESSURE_SEALEVEL * 100), 1) +
      " m");
    Serial.println("===");
  }
}

void loop() {}
