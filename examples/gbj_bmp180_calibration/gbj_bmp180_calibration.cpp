/**
 * @name gbj_bmp180_calibration
 *
 * @brief Reading BMP180's calibration data from EEPROM.
 *
 * @note If the macro GBJ_BMP180_TEST is defined, the library provides datasheet
 * sample calibration coefficients instead of reading them from the sensor's
 * EEPROM.
 *
 * @copyright This program is free software; you can redistribute it and/or
 * modify it under the terms of the MIT License (MIT).
 *
 * @author Libor Gabaj
 */
// #define GBJ_BMP180_TEST
#include "gbj_bmp180.h"

uint8_t calibCnt;
uint16_t *calibTable;
// The order is reversed due to MSB first
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

  calibTable = sensor.getCalibration(calibCnt);
  for (byte i = 0; i < calibCnt; i++)
  {
    Serial.print(calibItems[i] + ": ");
    if (calibItems[i] == "AC4" || calibItems[i] == "AC5" ||
        calibItems[i] == "AC6")
    {
      Serial.println(calibTable[i]);
    }
    else
    {
      Serial.println(static_cast<int16_t>(calibTable[i]));
    }
  }
  Serial.println("===");
}

void loop() {}
