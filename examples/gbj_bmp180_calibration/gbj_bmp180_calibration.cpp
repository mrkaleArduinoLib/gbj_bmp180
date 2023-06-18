/*
  NAME:
  Reading calibration data from EEPROM.

  DESCRIPTION:

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_bmp180.h"

const unsigned int PERIOD_MEASURE = 3000;
byte calibCnt;
int *calibTable;
String calibItems[] = {
  "MD", "MC", "MB", "B2", "B1", "AC6", "AC5", "AC4", "AC3", "AC2", "AC1",
};

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

  calibTable = reinterpret_cast<int *>(sensor.getCalibration(calibCnt));
  for (int i = calibCnt - 1; i >= 0; i--)
  {
    Serial.println(calibItems[i] + ": " + String(calibTable[i]));
  }
  Serial.println("---");
}

void loop() {}
