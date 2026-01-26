/** 
@file BQ34.cpp
@brief Implementation of the BQ34 class for interfacing with the BQ34Z100-R2 battery fuel gauge.
This file contains the definitions of the methods declared in BQ34.hpp.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: November 2025
*/

#include "dev/BQ34.hpp"


BQ34::BQ34(core::io::I2C* i2c) : i2cHandle(i2c) {}

// Read a 16-bit word from the BQ34Z100-R2
// params:
//   command - the command/register to read from
//   value - reference to store the read value
// returns:
//   true on success, false on failure
// read value stored in 'value'
bool BQ34::readWord(uint8_t command, uint16_t& value) {
    uint8_t buffer[2];

    // Uses 1-byte memory address (standard for BQ34Z100)
    auto status = i2cHandle->readMemReg(
        ADDRESS,         // 7-bit address
        static_cast<uint32_t>(command),               // register/command
        buffer,                // output buffer
        2,                     // number of bytes to read
        1                      // memory address length
    );

    if (status != core::io::I2C::I2CStatus::OK)
        return false;

    // Convert little-endian from BQ34Z100
    value = (uint16_t(buffer[1]) << 8) | buffer[0];

    return true;
}

// getters for various parameters

bool BQ34::getVoltage(uint16_t& mv) {
    if (readWord(VOLTAGE, mv) ){
        return true;
    }
    return false;
}

bool BQ34::getTemperature(uint16_t& t) {
    if (readWord(TEMPERATURE, t) ){
        return true;
    }
    return false;
}

bool BQ34::getCurrent(int16_t& mA) {
    if (readWord(CURRENT, (uint16_t&)mA) ){
        return true;
    }
    return false;
}

bool BQ34::getSOC(uint16_t& soc) {
    if (readWord(SOC, soc) ){
        return true;
    }
    return false;
}

bool BQ34::getMaxError(uint16_t& error) {
    if (readWord(MAXERROR, error) ){
        return true;
    }
    return false;
}

bool BQ34::getFlags(uint16_t& flags) {
    if (readWord(FLAGSH, flags)) {
        return true;
    }
    return false;
}

bool BQ34::getVoltageRaw(uint16_t& mv) {
    if (readWord(VOLTAGE, mv)) {
        return true;
    }
    return false;
}