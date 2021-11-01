/*
Function definitions is taken from POLOLU VL53L1X st-api
*/
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <slave_wire.h>

TwoWire Wire1 (PB7, PB6); // I2C_2 (PB11-SDA, PB10-SCL )
VL53L1X_Result_t resultArr[8];
bool isRobotArr[8] = {false};
bool filteredIsRobotArr[8] = {false};
float prevIsRobotArr[8] = {0.0};
uint16_t distanceArr[8] = {0};
uint16_t prevDistanceArr[8] = {0};
uint16_t filteredDistanceArr[8] = {0};
uint8_t statusArr[8] = {0};
uint16_t ambientArr[8] = {0};
uint8_t numDev = 0;
uint8_t error = 0;
uint8_t sensorErrId = 0;
uint8_t processId;

uint8_t ambientThresh = 60;

int8_t WriteMulti(uint8_t addr, uint16_t index, uint8_t *pdata, uint32_t count)
{
  while (count > 0)
  {
    Wire1.beginTransmission(addr >> 1);
    Wire1.write((index >> 8) & 0xff);
    Wire1.write(index & 0xff);

    uint8_t writing = 0;

    while (count > 0 && Wire1.write(*pdata) != 0)
    {
      pdata++;
      writing++;
      count--;
    }

    if (writing == 0 || Wire1.endTransmission() != 0) { return 1; }
    index += writing;
  }

  return 0;
}

int8_t ReadMulti(uint8_t addr, uint16_t index, uint8_t *pdata, uint32_t count)
{
  Wire1.beginTransmission(addr >> 1);
  Wire1.write((index >> 8) & 0xff);
  Wire1.write(index & 0xff);
  if (Wire1.endTransmission() != 0) { return 1; }

  while (count > 0)
  {
    uint8_t reading = Wire1.requestFrom(addr >> 1, count);

    if (reading == 0) { return 1; }
    count -= reading;

    while (reading-- > 0)
    {
      *pdata++ = Wire1.read();
    }
  }

  return 0;
}
void WrResult(VL53L1X_Result_t tofResult)
{
  byte msgArray[9];
  msgArray[0] = (tofResult.Status);
  msgArray[1] = (tofResult.Distance >> 8) & 0xFF;
  msgArray[2] = (tofResult.Distance) & 0xFF;
  msgArray[3] = (tofResult.Ambient >> 8) & 0xFF;
  msgArray[4] = (tofResult.Ambient) & 0xFF;
  msgArray[5] = (tofResult.SigPerSPAD >> 8) & 0xFF;
  msgArray[6] = (tofResult.SigPerSPAD) & 0xFF;
  msgArray[7] = (tofResult.NumSPADs >> 8) & 0xFF;
  msgArray[8] = (tofResult.NumSPADs) & 0xFF;
  Wire1.write(msgArray, 9);
}
void WrResults(VL53L1X_Result_t tofResultArr[])
{
  byte msgArray[numDev*2];
  for (uint8_t i = 0; i < numDev; i++)
  {
    VL53L1X_Result_t tofResult = tofResultArr[i];
    msgArray[2*i+0] = (tofResult.Distance >> 8) & 0xFF;
    msgArray[2*i+1] = (tofResult.Distance) & 0xFF;
    // msgArray[4*i+2] = (tofResult.Ambient >> 8) & 0xFF;
    // msgArray[4*i+3] = (tofResult.Ambient) & 0xFF;
  //   msgArray[4*i+4] = (tofResult.SigPerSPAD >> 8) & 0xFF;
  //   msgArray[4*i+5] = (tofResult.SigPerSPAD) & 0xFF;
  }
  Wire1.write(msgArray, 2*numDev);
}

void WrRandB()
{
  byte msgArray[numDev*2];
  for (uint8_t i = 0; i < numDev; i++)
  {
    uint16_t Distance = filteredDistanceArr[i];
    bool isRobot = filteredIsRobotArr[i];
    // Distance data is 12 -bit at max 4000 < 4096
    // MSB
    msgArray[2*i] = (Distance >> 8) & 0xFF;
    // LSB
    msgArray[2*i+1] = (Distance) & 0xFF;
    if (isRobot)
    {
      // turn on the 13'th bit
      msgArray[2*i] = msgArray[2*i] | 0x10;
    }
  }
  Wire1.write(msgArray, 2*numDev);
}
void WrAmbient()
{
  byte msgArray[numDev];
  for (uint8_t i = 0; i < numDev; i++)
  {
    uint16_t Ambient = ambientArr[i]/2;
    // Distance data is 12 -bit at max 4000 < 4096
    // LSB
    msgArray[i] = (Ambient) & 0xFF;
  }
  Wire1.write(msgArray, numDev);
}

void WrResultSigPerSPAD(VL53L1X_Result_t tofResultArr[])
{
  byte msgArray[numDev*2];
  for (uint8_t i = 0; i < numDev; i++)
  {
    VL53L1X_Result_t tofResult = tofResultArr[i];
    msgArray[2*i+0] = (tofResult.SigPerSPAD >> 8) & 0xFF;
    msgArray[2*i+1] = (tofResult.SigPerSPAD) & 0xFF;
  //   msgArray[4*i+4] = (tofResult.SigPerSPAD >> 8) & 0xFF;
  //   msgArray[4*i+5] = (tofResult.SigPerSPAD) & 0xFF;
  }
  Wire1.write(msgArray, 2*numDev);
}

void WrByte(uint8_t byteVal)
{
  Wire1.write(byteVal);
}

void WrWord(uint16_t longVal)
{
  byte msgArray[2];
  // seperate 32 bit value to 4 bytes
  msgArray[0] = (longVal >> 8) & 0xFF;
  msgArray[1] = (longVal) & 0xFF;
  Wire1.write(msgArray, 2);
}
void WrDWord(uint32_t longVal)
{
  byte msgArray[4];
  // seperate 32 bit value to 4 bytes
  msgArray[0] = (longVal >> 24) & 0xFF;
  msgArray[1] = (longVal >> 16) & 0xFF;
  msgArray[2] = (longVal >> 8) & 0xFF;
  msgArray[3] = (longVal) & 0xFF;
  Wire1.write(msgArray, 4);
}

uint8_t RdByte()
{
  return Wire1.read();
}

uint16_t RdWord()
{
  // as first item number of bytes are sent
  // read that value
  uint8_t howMany = Wire1.read();
  byte data[2];
  // collect all the bytes
  for (uint8_t i = 0; i < howMany; i++)
  {
    data[i] = Wire1.read();
  }
  /*   
  A union is an "overlay" of variables, 
  sharing the same memory cells.
  In this case it is used as a re-interpret-cast. 
  */
  union int16_tTag
  {
    byte b[2];
    int16_t intVal;
  } it;

  it.b[0] = data[0];
  it.b[1] = data[1];

  return it.intVal;
}

uint32_t RdDWord()
{
  // as first item number of bytes are sent
  // read that value
  uint8_t howMany = Wire1.read();
  byte data[4];
  
  // collect all the bytes
  for (uint8_t i = 0; i < howMany; i++)
  {
    data[i] = Wire1.read();
  }
  
  /*   
  A union is an "overlay" of variables,
  sharing the same memory cells.
  In this case it is used as a re-interpret-cast. 
  */
  union longTag
  {
    byte b[4];
    uint32_t longVal;
  } lt;

  lt.b[0] = data[0];
  lt.b[1] = data[1];
  lt.b[2] = data[2];
  lt.b[3] = data[3];

  return lt.longVal;
}

void receiveEvent(int howMany)
{
  // what to do with the incoming msg is determined
  // by its first byte

  processId = Wire1.read(); 
  // set motor controller params from the master
  switch (processId)
  {
    case 0: // refVal is set by master
    {
      int8_t readVal = Wire1.read();
      if (readVal == 3)
      {
        pinMode(PC13, OUTPUT);
        digitalWrite(PC13,HIGH);
      }
      break;
    }
    case 1: // kp is set by master
    {
      uint8_t readVal = Wire1.read();
      ambientThresh = readVal;
      break;
    }
    case 2: // ki is set by master
    {
      break;
    }
    case 3:  // kd is set by master
    {
      break;
    }
    case 4:
    {
      break;
    }
    default:
    {
      break;
    }
  }
  // flush the i2c input buffer
  while (Wire1.available() > 0)
  {
    Wire1.read();
  }
}

void requestEvent()
{
  switch (processId)
  {
    case 255:
    {
      WrByte(numDev);
      break; 
    }
    case 254:
    {
      WrByte(error);
      break;
    }
    case 253:
    {
      WrByte(sensorErrId);
      break;
    }
    case 252:
    {
      WrResults(resultArr);
      break;
    }
    case 251:
    {
      WrResultSigPerSPAD(resultArr);
      break;
    }
    case 250:
    {
      WrRandB();
      break;
    }
    case 249:
    {
      WrAmbient();
      break;
    }
    default:
    {
      break;
    }
  }
}  // end of requestEvent

void slaveBegin(uint8_t slaveAddr)
{
  // Join to main I2C bus as a slave
  Wire1.begin(slaveAddr);
  Wire1.onReceive(receiveEvent);
  Wire1.onRequest(requestEvent);
}

