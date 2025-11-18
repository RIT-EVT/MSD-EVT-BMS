/**
@file BQ34.hpp
@brief C++ header for interfacing with the BQ34Z100-R2 battery fuel gauge.
This header defines a class for communicating with the BQ34Z100-R2 over I2C
using STM32 HAL libraries.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: November 2025
*/

#ifndef BQ34_HPP
#define BQ34_HPP

#include "stm32f4xx_hal.h"
#include <cstdint>

// BQ34Z100-R2 commands
#define CONTROL             0x00 | 0x01
// control sub-commands
#define C_STATUS         0x0000
#define C_RESET_DATA     0x0005
#define C_PREV_MACRO     0x0007
#define C_BOARD_OFFSET   0x0009
#define C_CC_OFFSET      0x000A
#define C_CAL_ENABLE     0x002D
#define C_RESET          0x0041
#define C_EXIT_CAL       0x0080
#define C_ENTER_CAL      0x0081
#define C_SEALED         0x0020

// other commands - see datasheet
#define SOC                 0x02
#define MAXERROR            0x03 // SOH error in %
#define REMAININGCAPACITY   0x04 | 0x05 // in mAh
#define FULLCHARGECAPACITY  0x06 | 0x07 // in mAh
#define VOLTAGE             0x08 | 0x09 // in mV
#define AVERAGECURRENT      0x0A | 0x0B // in mA
#define CURRENT             0x10 | 0x11 // in mA
#define TEMPERATURE         0x0C | 0x0D // in 0.1K
#define FLAGS               0x0E | 0x0F 
#define FLAGSB              0x12 | 0x13

class BQ34 {
public:
    BQ34(I2C_HandleTypeDef* i2c);

    bool readWord(uint8_t command, uint16_t& value);

    bool getVoltage(uint16_t& mv);
    bool getTemperature(uint16_t& t);
    bool getCurrent(int16_t& mA);
    bool getSOC(uint16_t& soc);
    bool getSOH(uint16_t& soh);

private:
    I2C_HandleTypeDef* i2cHandle;
    static constexpr uint8_t ADDRESS = 0x55 << 1;  // in STM form
};

#endif
