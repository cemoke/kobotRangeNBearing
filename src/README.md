# VL53L1x ULD API port for Arduino Platform used in Relative Positioning System with STM32F103C8T6 #
## Cemoke ##
* ULD API by ST was used
* PlatformIO extension of VSCode was used
* 
## Notes ##
* vl53l1_api & vl53l1_types are from ST
* vl53l1_platform is implemented for Arduino framework by using Pololu library for VL53L1x
* multiple vl53l1x operation is implemented in the main.cpp
* I2C_2 of STM32F103 was used for communicating with sensors
* I2C_1 of STM32F103 was used for communicating with external HW
* Supply and IO voltages are regulated 3.3v
* shutPin is used on vl53l1, for setting I2C address of the device after boot by resetting devices
* ranging result is collected by polling
* Up to 12 sensors can be interfaced 