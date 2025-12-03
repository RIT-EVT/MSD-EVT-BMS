/** 
@file BQ34.cpp
@brief Implementation of the BQ34 class for interfacing with the BQ34Z100-R2 battery fuel gauge.
This file contains the definitions of the methods declared in BQ34.hpp.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: November 2025
*/

#include "dev/BQ34.hpp"
#include <iostream>


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

bool BQ34::getFlags(uint16_t& flags) {
    if (readWord(FLAGS, flags)) {
        std::cout << "FLAGS: 0x" << std::hex << flags << std::dec << std::endl;

        // Decode common flag bits
        if (flags & 0x0001) std::cout << "  - DSG (Discharging detected)" << std::endl;
        if (flags & 0x0002) std::cout << "  - SOCF (SOC threshold final)" << std::endl;
        if (flags & 0x0004) std::cout << "  - SOC1 (SOC threshold 1)" << std::endl;
        if (flags & 0x0008) std::cout << "  - BAT_DET (Battery detected)" << std::endl;
        if (flags & 0x0010) std::cout << "  - WAIT_ID (Waiting for ID)" << std::endl;
        if (flags & 0x0020) std::cout << "  - OCV_TAKEN (OCV measurement taken)" << std::endl;
        if (flags & 0x0100) std::cout << "  - CHG (Charging detected)" << std::endl;
        if (flags & 0x0200) std::cout << "  - FC (Fully charged)" << std::endl;
        if (flags & 0x0400) std::cout << "  - OTD (Over temperature discharge)" << std::endl;
        if (flags & 0x0800) std::cout << "  - OTC (Over temperature charge)" << std::endl;

        return true;
    }
    return false;
}

bool BQ34::getVoltageRaw(uint16_t& mv) {
    if (readWord(VOLTAGE, mv)) {
        std::cout << "Voltage (raw): " << mv << " mV" << std::endl;
        return true;
    }
    return false;
}