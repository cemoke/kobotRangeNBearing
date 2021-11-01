#include <Arduino.h>
#include <Wire.h>
#include <vl53l1_api.h>
#include <slave_wire.h>

/*
we have connections for 8 sensors max.
if not all the 8 sensors are connected
or not desired to work, 
then configure sensorArray[] to map 
which sensors are going to work and
connected to the which shutPin.
*/
//
// const uint8_t sensorArr[] = {PA8,PB14,PB12,PA4,PA3,PB5,PB3,PA15};  # True Config
//
const uint8_t sensorArr[] = {PA8,PB14,PB12,PA4,PA3,PB5,PB3,PA15};

// default i2cAddr for the TOF sensors
const uint8_t baseAddr = 0x52;
// number of Tof is determined from sensorArr[]
uint8_t numTOF;
uint8_t counter=0;
// Struct Array for multiple TOF Device 
VL53L1_Dev_t tof[8];
// Array of Pointers to the structs
VL53L1_DEV tofPtr[8];

/** @brief bootTOF() definition.\n
 * Checks whether sensor boots up 
 * correctly or not and inits device
 * /w default params.
 * @param tofPtr -> pointer to TOF struct 
 * @param state -> 1:booted, 0:boot-error
 */
void bootTOF(VL53L1_DEV tofPtr)
{
  uint8_t state = 0;
  while (1)
  {
    VL53L1X_BootState(tofPtr, &state);
    if (state)
    {
      // init TOF w/ default params.
      VL53L1X_SensorInit(tofPtr);
      // give time to write all params.
       break;
    }
    else // not booted yet try again
    {
      delay(50);
      uint8_t shutPin;
      shutPin = tofPtr->shutPin;
      sensorErrId = shutPin;
      // not booted yet try again
    }
  }
}

/** @brief createSensorArray() definition.\n
 * constructs struct array and array of pointers
 * to structs by using sensorArr[]
 * @param tof[] -> tof sensor struct array
 * @param tofPtr[] -> array of pointers to
 * structs (tof[])
 * @param numTOF -> number of active TOF sensors
 */
void createSensorArray()
{
  numTOF = sizeof(sensorArr)/sizeof(uint8_t);
  for (uint8_t i = 0; i < numTOF; i++)
  {
    // increment addr by 2 since addr is 
    // 7-bit instead of 8-bit
    tof[i].i2cAddr = baseAddr;
    tof[i].shutPin = sensorArr[i];
    tof[i].sensorIndx = i;
    tofPtr[i] = &tof[i];
  }
}

/** @brief setMultipleTOF() definition.\n
 * All sensors are assigned unique ID and 
 * corresponding params are set 
 * @param tofPtr[] -> array of pointers to tof 
 * structs 
 */
void setMultipleTOF(VL53L1_DEV tofPtr[])
{
  // configure shutPins and shut down all the TOFs
  for (uint8_t i = 0; i < numTOF; i++)
  {
    pinMode(tofPtr[i]->shutPin, OUTPUT);
    // // Shut TOF
    digitalWrite(tofPtr[i]->shutPin, LOW);
  }
  // wake TOF one by one and set new i2cAddr
  for (uint8_t i = 0; i < numTOF; i++)
  {
    // Wake TOF
    digitalWrite(tofPtr[i]->shutPin, HIGH);
    delay(50);
    // Loop until TOF boots up properly
    bootTOF(tofPtr[i]);
    // (i+1) because none of the sensor IDs should be the default addr 
    uint8_t error = VL53L1X_SetI2CAddress(tofPtr[i], baseAddr + 2*(i+1));
    if (error)
    {
      delay(50);
      // try again from start
      setMultipleTOF(tofPtr);
    }
    // now address is changed, change it in the struct as well
    // for upcoming i2c operations
    tofPtr[i]->i2cAddr = baseAddr + 2*(i+1);
    tofPtr[i]->sensorIndx = i;
    // without calling SensorInit() again it is not working
    VL53L1X_SensorInit(tofPtr[i]);
    // configure timing budget and distance mode 
    // choose 15 ms -> 66Hz for max. sampling resolution is enough
    // short mode is always more than enough
    VL53L1X_SetTimingBudgetInMs(tofPtr[i], 100);
    // Short Distance : 1, Long : 2
    VL53L1X_SetDistanceMode(tofPtr[i], 1);
    // start continous ranging
    VL53L1X_StartRanging(tofPtr[i]);
  }
}

/** @brief rangingLoop() definition.\n
 * To be implemented by the developer
 * @param tofPtr -> pointer to tof struct 
 * @return when data is read, continous loop otherwise
 */
VL53L1X_Result_t rangingLoop(VL53L1_DEV tofPtr)
{
  uint8_t isDataReady = 0;
  VL53L1X_Result_t measurementResult;
  // poll for data ready
  while(1)
  {
    VL53L1X_CheckForDataReady(tofPtr, &isDataReady);
    if (isDataReady)
    {
      VL53L1X_GetResult(tofPtr, &measurementResult);
      // change ambient to ambientPerSpad it is more consistent
      measurementResult.Ambient = measurementResult.Ambient / float(measurementResult.NumSPADs);
      break;
    }
    else
    {
      // wait for data ready
      delay(5);
    }
  }
  return measurementResult;
}

bool robotDetection(uint16_t ambient)
{
  bool isRobot;
  if (ambient > ambientThresh)
  {
    isRobot = true;
  }
  else
  {
    isRobot = false;
  }
  return isRobot;
}

uint16_t movingAverageFilter(uint16_t currentVal, uint16_t prevVal, float zeta)
{
  return prevVal * (1 - zeta) + zeta * currentVal;
}

uint16_t filterDistance(uint16_t readDistance, uint16_t prevDistance, uint8_t status, bool isRobot)
{
  uint16_t filteredDistance;
  float zeta = 1.0;
  if (readDistance < 30)
    readDistance = 4000;
  switch (status)
  {
    case 0:  // range is valid
    {
      filteredDistance = movingAverageFilter(readDistance, prevDistance, zeta);
      break;
    }
    default:
    {
      if (isRobot)
      {
      readDistance = prevDistance;
      filteredDistance = movingAverageFilter(readDistance, prevDistance, zeta); 
      }
      else
      {
      readDistance = 4000;
      filteredDistance = movingAverageFilter(readDistance, prevDistance, zeta); 
      }
      break;
    }
    // case 1:  // WARNING sigma failure
    // case 2:  // WARNING signal failure
    // {
    //   if (isRobot)
    //   {
    //     // we can assume distance is the last valid distance
    //     filteredDistance = prevDistance;
    //   }
    //   else
    //   {
    //     // most probably object is out of sight
    //     readDistance = 4000;
    //     filteredDistance = movingAverageFilter(readDistance, prevDistance, zeta);
    //   }
    //   break;
    // }
    // case 4:  // ERROR out of bounds
    // case 7:  // ERROR wraparound
    // {
    //   if (isRobot)
    //   {
    //     // we can assume distance is the last valid distance
    //     filteredDistance = prevDistance;
    //   }
    //   else
    //   {
    //     // most probably object is out of sight
    //     readDistance = 4000;
    //     filteredDistance = movingAverageFilter(readDistance, prevDistance, zeta);
    //   }
    //   break;
    // }
  }
  return filteredDistance;
}

bool filterIsRobot(uint8_t tofIndx, bool isRobot)
{
  uint8_t bufferSize;
  bufferSize = tof[tofIndx].bufferSize;
  // move older values in the buffer by one
  // and add the newest reading
  for (uint8_t j=0; j<bufferSize;j++)
  {
    if (j == (bufferSize - 1))
    {
      // add the newest reading
      tof[tofIndx].isRobot[j] = isRobot;
    }
    else
    {
      // shift old readings by one
      tof[tofIndx].isRobot[j] = tof[tofIndx].isRobot[j+1];
    }
  }
  // check the buffer for isRobot values
  bool filteredIsRobot = false;
  for (uint8_t j=0; j<bufferSize;j++)
  {
    if (tof[tofIndx].isRobot[j])
    {
      // even we have one isRobot in the buffer
      // assume we saw a robot
      filteredIsRobot = true;
      break;
    }
  }
  return filteredIsRobot;
}

void setup() 
{
  delay(2000);
  // Initialize DEBUG_LED3
  pinMode(PC13, OUTPUT);
  // Join to I2C-1 bus as a slave
  slaveBegin(10);
  // Join to I2C-2 bus as the master
  VL53L1_begin();
  createSensorArray();
  setMultipleTOF(tofPtr);
  numDev = VL53L1_scanI2CBus();
  if (numDev == numTOF)
  {
    // all sensors are ready
    // inform the user
    digitalWrite(PC13, HIGH);
  }
  else
  {
    digitalWrite(PC13, LOW);
    error = 1;
  }
}

void loop() 
{
  // long startTime = millis();
  counter++;

  // collect and process readings from all tofs
  for (uint8_t i = 0; i < numTOF; i++)
  {
    resultArr[i] = rangingLoop(tofPtr[i]);
    distanceArr[i] = resultArr[i].Distance;
    statusArr[i] = resultArr[i].Status;
    ambientArr[i] = resultArr[i].Ambient;

    bool isRobot;
    isRobot = robotDetection(ambientArr[i]);
    isRobotArr[i] = isRobot;

    bool filteredIsRobot;
    filteredIsRobot = filterIsRobot(i, isRobotArr[i]);
    filteredIsRobotArr[i] = filteredIsRobot;

    uint16_t filteredDistance;
    filteredDistance = filterDistance(distanceArr[i], prevDistanceArr[i],
                                      statusArr[i], isRobotArr[i]);
    filteredDistanceArr[i] = filteredDistance;
    prevDistanceArr[i] = filteredDistanceArr[i];
    // long timeDiff = millis() - startTime;
  }

  if (random(0,2))
  {
    delay(20);
  }
}
