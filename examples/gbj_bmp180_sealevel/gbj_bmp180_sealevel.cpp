/**
 * @name gbj_bmp180_sealevel
 *
 * @brief One-time barometric pressure measuring and recalculating it to sea
 * level.
 *
 * @note Altitude input in meters.
 * @note Sea level pressure output in hectopascals.
 *
 * @copyright This program is free software; you can redistribute it and/or
 * modify it under the terms of the MIT License (MIT).
 *
 * @author Libor Gabaj
 */
#include "gbj_bmp180.h"

const float ALTITUDE = 134.0; // Bratislava, Slovakia
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
    Serial.println("Altitude: " + String(ALTITUDE, 1) + " m");
    Serial.println("Pressure: " + String(pressValue / 100, 2) + " hPa");
    Serial.println(
      "PressureSea: " +
      String(sensor.getPressureSea(pressValue, ALTITUDE) / 100, 2) + " hPa");
    Serial.println("===");
  }
}

void loop() {}
