/**
 * @file  master_wire.h
 * @brief Wire implementation for multiple bytes comm. with RPi
 */
#ifndef _SLAVE_WIRE_H_
#define _SLAVE_WIRE_H_
#include <Arduino.h>
#include <vl53l1_api.h>

#ifdef __cplusplus
extern "C"
{
#endif


// extern means declared here implemented externally
extern VL53L1X_Result_t resultArr[8];
extern bool isRobotArr[8];
extern bool filteredIsRobotArr[8];
extern float prevIsRobotArr[8];
extern uint16_t prevDistanceArr[8];
extern uint16_t filteredDistanceArr[8];
extern uint16_t distanceArr[8];
extern uint16_t ambientArr[8];
extern uint8_t statusArr[8];
extern uint8_t ambientThresh;

extern uint8_t numDev;
extern uint8_t error;
extern uint8_t sensorErrId;
void slaveBegin(uint8_t slaveAddr);
void requestEvent(int howMany);
void receiveEvent();
uint8_t scanI2CBus();
/** @brief WriteMulti() definition.\n
 * To be implemented by the addreloper
 */
int8_t WriteMulti(
		uint8_t 			addr,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief ReadMulti() definition.\n
 * To be implemented by the addreloper
 */
int8_t ReadMulti(
		uint8_t 			addr,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief WrByte() definition.\n
 * To be implemented by the addreloper
 */
void WrByte(
		uint8_t byteVal);
/** @brief WrWord() definition.\n
 * To be implemented by the addreloper
 */
void WrWord(uint16_t shortVal);
/** @brief WrDWord() definition.\n
 * To be implemented by the addreloper
 */
void WrDWord(uint32_t longVal);
/** @brief RdByte() definition.\n
 * To be implemented by the addreloper
 */

uint8_t RdByte();
/** @brief RdWord() definition.\n
 * To be implemented by the addreloper
 */
uint16_t RdWord();
/** @brief RdDWord() definition.\n
 * To be implemented by the addreloper
 */
uint32_t RdDWord();
#ifdef __cplusplus
}
#endif

#endif
