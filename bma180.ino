#include <Kalman.h>

#include <Wire.h>

#define address 0x40
double measurement, filteredMeasurement;
/*q: process noise covariance, r = measurement noise covariance, p = estimation error covariance*/
Kalman myFilter(0.125,32,1023,0); //suggested initial values for high noise filtering

void setup()
{
  pinMode(A0, INPUT);
  Wire.begin();
  Serial.begin(115200);
  initBMA180();
  delay(2000);
}

void loop()
{
  readEMG();
  readAccel();

  delay(200);
}

int x;

void readAccel()
{
  int temp, result;

  temp = 0;

  while (temp != 1)
  {
    Wire.beginTransmission(address);
    Wire.write(0x03);
    Wire.requestFrom(address, 1);
    while (Wire.available())
    {
      temp = Wire.read() & 0x01;
    }
  }

  Wire.beginTransmission(address);
  Wire.write(0x02);
  Wire.requestFrom(address, 1);
  while (Wire.available())
  {
    temp |= Wire.read();
    temp = temp >> 2;
  }

  /*set kalman params for accelerometer*/
  setParameters(0.125,32,1023);
  
  measurement = (double) temp;
  filteredMeasurement = myFilter.getFilteredValue(measurement);
  // Serial.print("X = ");
//  Serial.print(temp);
  Serial.print(temp);
  Serial.print(" ");
  result = Wire.endTransmission();

  /*Added by RX*/
  /*For y axis*/
  Wire.beginTransmission(address);
  Wire.write(0x05);
  Wire.requestFrom(address, 1);
  while (Wire.available())
  {
    temp = Wire.read() & 0x01;
  }

  Wire.beginTransmission(address);
  Wire.write(0x04);
  Wire.requestFrom(address, 1);
  while (Wire.available())
  {
    temp |= Wire.read();
    temp = temp >> 2;
  }

  measurement = (double) temp;
  filteredMeasurement = myFilter.getFilteredValue(measurement);
  // Serial.print("Y = ");
//  Serial.print(temp);
  Serial.print(temp);
  Serial.print(" ");
  result = Wire.endTransmission();

  /*For Z axis*/
  Wire.beginTransmission(address);
  Wire.write(0x07);
  Wire.requestFrom(address, 1);
  while (Wire.available())
  {
    temp = Wire.read() & 0x01;
  }

  Wire.beginTransmission(address);
  Wire.write(0x06);
  Wire.requestFrom(address, 1);
  while (Wire.available())
  {
    temp |= Wire.read();
    temp = temp >> 2;
  }

  measurement = (double) temp;
  filteredMeasurement = myFilter.getFilteredValue(measurement);
  // Serial.print("Z = ");
//  Serial.print(temp);
  Serial.print(temp);
  Serial.print(" ");
  result = Wire.endTransmission();

  
  // Serial.println();
  /*End of code added by RX*/
}

void initBMA180()
{
  int temp, result, error;

  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.requestFrom(address, 1);
  while (Wire.available())
  {
    temp = Wire.read();
  }
  Serial.print("Id = ");
  Serial.println(temp);
  result = Wire.endTransmission();
  checkResult(result);
  if (result > 0)
  {
    error = 1;
  }
  delay(10);
  if (temp == 3)
  {
    // Connect to the ctrl_reg1 register and set the ee_w bit to enable writing.
    Wire.beginTransmission(address);
    Wire.write(0x0D);
    Wire.write(B0001);
    result = Wire.endTransmission();
    checkResult(result);
    if (result > 0)
    {
      error = 1;
    }
    delay(10);
    // Connect to the bw_tcs register and set the filtering level to 10hz.
    Wire.beginTransmission(address);
    Wire.write(0x20);
    Wire.write(B00001000);
    result = Wire.endTransmission();
    checkResult(result);
    if (result > 0)
    {
      error = 1;
    }
    delay(10);
    // Connect to the offset_lsb1 register and set the range to +- 2.
    Wire.beginTransmission(address);
    Wire.write(0x35);
    Wire.write(B0100);
    result = Wire.endTransmission();
    checkResult(result);
    if (result > 0)
    {
      error = 1;
    }
    delay(10);
  }

  if (error == 0)
  {
    //   Serial.print("BMA180 Init Successful");
  }
}

void checkResult(int result)
{
  if (result >= 1)
  {
    //   Serial.print("PROBLEM..... Result code is ");
    //   Serial.println(result);
  }
  else
  {
    //   Serial.println("Read/Write success");
  }
}

void readId()
{
  int temp, result;

  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.requestFrom(address, 1);
  while (Wire.available())
  {
    temp = Wire.read();
  }
  // Serial.print("Id = ");
  // Serial.println(temp);
  result = Wire.endTransmission();
  checkResult(result);
  delay(10);
}

void readEMG(){
  /*set kalman params for EMG sensors*/
  setParameters(0.125,32,1023); //default values
  measurement = (double) analogRead(A0);
  filteredMeasurement = myFilter.getFilteredValue(measurement);
  Serial.println(filteredMeasurement);
}

