/** 
@file BQ34.cpp
@brief Implementation of the BQ34 class for interfacing with the BQ34Z100-R2 battery fuel gauge.
This file contains the definitions of the methods declared in BQ34.hpp.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: November 2025
*/

#include "dev/BQ34.hpp"

BQ34::BQ34(I2C_HandleTypeDef* i2c) : i2cHandle(i2c) {}

// Read a 16-bit word from the BQ34Z100-R2
// params:
//   command - the command/register to read from
//   value - reference to store the read value
// returns:
//   true on success, false on failure
// read value stored in 'value'
bool BQ34::readWord(uint8_t command, uint16_t& value) {
    uint8_t buffer[2];

    if (HAL_I2C_Mem_Read(i2cHandle, ADDRESS, command,
                         I2C_MEMADD_SIZE_8BIT, buffer, 2, 100) != HAL_OK) {
        return false;
    }

    value = (buffer[1] << 8) | buffer[0];
    return true;
}

// getters for various parameters

bool BQ34::getVoltage(uint16_t& mv) { 
    if (readWord(VOLTAGE, mv) ){
        std::cout << "Voltage: " << mv << " mV" << std::endl;
        return true;
    }
    return false;
}

bool BQ34::getTemperature(uint16_t& t) { 
    if (readWord(TEMPERATURE, t) ){
        std::cout << "Temperature: " << (t / 10.0 - 273.15) << " °C" << std::endl;
        return true;
    }
    return false;
}

bool BQ34::getCurrent(int16_t& mA) { 
    if (readWord(CURRENT, (uint16_t&)mA) ){
        std::cout << "Current: " << mA << " mA" << std::endl;
        return true;
    }
    return false;
}

bool BQ34::getSOC(uint16_t& soc) { 
    if (readWord(SOC, soc) ){
        std::cout << "State of Charge: " << soc << " %" << std::endl;
        return true;
    }
    return false;
}

bool BQ34::getSOH(uint16_t& soh) { 
    if (readWord(MAXERROR, soh) ){
        std::cout << "State of Health: " << soh << " %" << std::endl;
        return true;
    }
    return false;
}