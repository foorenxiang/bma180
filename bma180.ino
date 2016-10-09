#include <Kalman.h>

#include <Wire.h>

#define address 0x40
double measurement, filteredMeasurement;
Kalman myFilter(0.125,30,1023,0); //suggested initial values for high noise filtering

#define samplesize 3
float samplex[samplesize] = {0}; //accelerometer x axis
float sampley[samplesize] = {0}; 
float samplez[samplesize] = {0};
float sampleemg[samplesize] = {0};

float changex = 0;
float changey = 0;
float changez = 0;
float changeemg = 0;

float changex2nd = 0;
float changey2nd = 0;
float changez2nd = 0;
float changeemg2nd = 0;

int t = 0; //tracker
int flag = 0; //first sample set tracker

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  initBMA180();
  delay(2000);
}

void loop()
{
  myFilter.setParameters(0.125,30,1023);
  readAccel();
//  myFilter.setParameters(0.125,32,1023);
//  readEMG();
//
  if(flag == 1 && t == 0){
    firstorder(samplex, &changex);
//    firstorder(sampley, &changey);
//    firstorder(samplez, &changez);
//    firstorder(sampleemg, &changeemg);
//
    secondorder(samplex, &changex2nd);
//    secondorder(sampley, &changey2nd);
//    secondorder(samplez, &changez2nd);
//    secondorder(sampleemg, &changeemg2nd);
  }
  Serial.print(changex);
  Serial.print(" ");
  Serial.print(changex2nd);
  Serial.println();

  t++;
  if(t == samplesize) t = 0;
  flag = 1;

  delay(50);
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

  measurement = (double) temp;
  filteredMeasurement = myFilter.getFilteredValue(measurement);
  samplex[t] = filteredMeasurement;
  // Serial.print("X = ");
  Serial.print(filteredMeasurement);
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
  sampley[t] = filteredMeasurement;
  Serial.print(filteredMeasurement);
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
  samplez[t] = filteredMeasurement;
  // Serial.print("Z = ");
  Serial.print(filteredMeasurement);
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
  myFilter.setParameters(0.125,32,1023); //default values
  measurement = (double) analogRead(A0);
  filteredMeasurement = myFilter.getFilteredValue(measurement);
  Serial.print(filteredMeasurement);
  Serial.println("");
  sampleemg[t] = filteredMeasurement;
}

void firstorder(float* sample, float* change){
  int i;
  for(i = 0, *change = 0; i<samplesize-1; i++){
    *change +=  (sample[i+1] - sample[i]);
  }
  
    *change /= samplesize-1;
}

void secondorder(float* sample, float* change){
  int i;

  for(i = 0, *change = 0; i<samplesize-2; i++){
    *change += (int) (sample[i+2] - sample[i+1])-(sample[i+1] - sample[i]);
  }
  
  *change /= samplesize-2;
}
