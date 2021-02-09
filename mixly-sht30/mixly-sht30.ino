#include <Wire.h>

#define sht30_address 0x44

float sht30_get_data(int data_type)
{
  unsigned int data[6];
  Wire.beginTransmission(sht30_address);
  Wire.write(0x2C);
  Wire.write(0x06);
  Wire.endTransmission();
  Wire.requestFrom(sht30_address, 6);
  if (Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
  }
  switch (data_type)
  {
    case 0:
      return ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
    case 1:
      return ((((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45) * 1.8) + 32;
    case 2:
      return ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);
    default:
      return -1;
  }
}

void setup(){
  Serial.begin(9600);
  Wire.begin();
}

void loop(){
  Serial.println(sht30_get_data(0));
  Serial.println(sht30_get_data(2));
  delay(5000);

}
