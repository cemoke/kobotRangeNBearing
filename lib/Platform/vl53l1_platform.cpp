
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

/*
Function definitions is taken from POLOLU VL53L1X st-api
*/

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <vl53l1_platform.h>


TwoWire Wire2 (PB11, PB10); // I2C_2 (PB11-SDA, PB10-SCL )
void VL53L1_begin()
{
  Wire2.begin();
  Wire2.setClock(400000);
}

/** @brief scanI2CBus() definition.\n
 * scans I2C Bus and reports found devices
 * through serial port
 */
uint8_t VL53L1_scanI2CBus()
{
  byte count = 0;
  for (byte i = 1; i < 120; i++)
  {
    Wire2.beginTransmission (i);
    if (Wire2.endTransmission () == 0)
    {
      count++;
    } // end of good response
    delay (5);  // give devices time to recover
  } // end of for loop
  return count;
}

int8_t VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  while (count > 0)
  {
    Wire2.beginTransmission(Dev->i2cAddr >> 1);
    Wire2.write((index >> 8) & 0xff);
    Wire2.write(index & 0xff);

    uint8_t writing = 0;

    while (count > 0 && Wire2.write(*pdata) != 0)
    {
      pdata++;
      writing++;
      count--;
    }

    if (writing == 0 || Wire2.endTransmission() != 0) { return 1; }
    index += writing;
  }

  return 0;
}


int8_t VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  Wire2.beginTransmission(Dev->i2cAddr >> 1);
  Wire2.write((index >> 8) & 0xff);
  Wire2.write(index & 0xff);
  if (Wire2.endTransmission() != 0) { return 1; }

  while (count > 0)
  {
    uint8_t reading = Wire2.requestFrom(Dev->i2cAddr >> 1, count);

    if (reading == 0) { return 1; }
    count -= reading;

    while (reading-- > 0)
    {
      *pdata++ = Wire2.read();
    }
  }

  return 0;
}


int8_t VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
  Wire2.beginTransmission(Dev->i2cAddr >> 1);
  Wire2.write((index >> 8) & 0xff);
  Wire2.write(index & 0xff);
  Wire2.write(data);
  return (Wire2.endTransmission() == 0 ? 0 : 1);
}

int8_t VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
{
  Wire2.beginTransmission(Dev->i2cAddr >> 1);
  Wire2.write((index >> 8) & 0xff);
  Wire2.write(index & 0xff);
  Wire2.write((data >> 8) & 0xff);
  Wire2.write(data & 0xff);
  return (Wire2.endTransmission() == 0 ? 0 : 1);
}

int8_t VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
{
  Wire2.beginTransmission(Dev->i2cAddr >> 1);
  Wire2.write((index >> 8) & 0xff);
  Wire2.write(index & 0xff);
  // Cast to uint8_t to solve overload ambiguity
  Wire2.write(uint8_t((data >> 24) & 0xff));
  Wire2.write(uint8_t((data >> 16) & 0xff));
  Wire2.write(uint8_t((data >> 8) & 0xff));
  Wire2.write(uint8_t(data & 0xff));
  return (Wire2.endTransmission() == 0 ? 0 : 1);
}


int8_t VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data)
{
  Wire2.beginTransmission(Dev->i2cAddr >> 1);
  Wire2.write((index >> 8) & 0xff);
  Wire2.write(index & 0xff);
  if (Wire2.endTransmission() != 0) { return 1; }
  if (Wire2.requestFrom(Dev->i2cAddr >> 1, 1) != 1) { return 1; }
  *data = Wire2.read();
  return 0;
}

int8_t VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data)
{
  Wire2.beginTransmission(Dev->i2cAddr >> 1);
  Wire2.write((index >> 8) & 0xff);
  Wire2.write(index & 0xff);
  if (Wire2.endTransmission() != 0) { return 1; }
  if (Wire2.requestFrom(Dev->i2cAddr >> 1, 2) != 2) { return 1; }
  *data = (uint16_t)Wire2.read() << 8;
  *data |= Wire2.read();
  return 0;
}

int8_t VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data)
{
  Wire2.beginTransmission(Dev->i2cAddr >> 1);
  Wire2.write((index >> 8) & 0xff);
  Wire2.write(index & 0xff);
  if (Wire2.endTransmission() != 0) { return 1; }
  if (Wire2.requestFrom(Dev->i2cAddr >> 1, 4) != 4) { return 1; }
  *data = (uint32_t)Wire2.read() << 24;
  *data |= (uint32_t)Wire2.read() << 16;
  *data |= (uint16_t)Wire2.read() << 8;
  *data |= Wire2.read();
  return 0;
}

int8_t VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
  delay(wait_ms);
  return 0;
}
