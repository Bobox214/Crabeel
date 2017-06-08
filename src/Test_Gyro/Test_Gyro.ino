#include "MeMegaPi.h"
#include <Wire.h>

MeGyro gyro;
void setup()
{
  Serial.begin(115200);
  gyro.begin();
}

void loop()
{
  gyro.update();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" GY:");
  Serial.println(gyro.getGyroY() );
  delay(10);
}

