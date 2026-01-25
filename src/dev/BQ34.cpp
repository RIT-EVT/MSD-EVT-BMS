/** 
@file BQ34.cpp
@brief Implementation of the BQ34 class for interfacing with the BQ34Z100-R2 battery fuel gauge.
This file contains the definitions of the methods declared in BQ34.hpp.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: November 2025
*/

#include "dev/BQ34.hpp"


BQ34::BQ34(core::io::I2C* i2c, core::io::UART* uart) : i2cHandle(i2c), uartHandle(uart) {}

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
        // debug print statement
        uartHandle->printf("Voltage: %d mV%\r\n", mv);
        return true;
    }
    return false;
}

bool BQ34::getTemperature(uint16_t& t) {
    if (readWord(TEMPERATURE, t) ){
        // debug print statement
        t = t/10 - 273.15f;
        uartHandle->printf("Temperature: %d C%\r\n", t);
        return true;
    }
    return false;
}

bool BQ34::getCurrent(int16_t& mA) {
    if (readWord(CURRENT, (uint16_t&)mA) ){
        // debug print statement
        uartHandle->printf("Current: %d mA%\r\n", mA);
        return true;
    }
    return false;
}

bool BQ34::getSOC(uint16_t& soc) {
    if (readWord(SOC, soc) ){
        // debug print statement
        uartHandle->printf("SOC: %d mA%\r\n", soc);
        return true;
    }
    return false;
}

bool BQ34::getSOH(uint16_t& soh) {
    if (readWord(MAXERROR, soh) ){
        // debug print statement
        uartHandle->printf("SOH: %d mA%\r\n", soh);
        return true;
    }
    return false;
}

bool BQ34::getFlags(uint16_t& flags) {
    if (readWord(FLAGS, flags)) {
        // debug print statements
        uartHandle->printf("Flags: %d mA:%\r\n", flags);

        // Decode common flag bits

        if (flags & 0x0001) uartHandle->printf("%\tDischarging detected%\r\n");
        if (flags & 0x0002) uartHandle->printf("%\tSOC Final Threshold%\r\n");
        if (flags & 0x0004) uartHandle->printf("%\tSOC Threshold 1%\r\n");
        if (flags & 0x0008) uartHandle->printf("%\tBattery Detected%\r\n");
        if (flags & 0x0010) uartHandle->printf("%\tWaiting for ID%\r\n");
        if (flags & 0x0020) uartHandle->printf("%\tOCV Measurement Taken%\r\n");
        if (flags & 0x0100) uartHandle->printf("%\tCharging Detected%\r\n");
        if (flags & 0x0200) uartHandle->printf("%\tFully Charged%\r\n");
        if (flags & 0x0400) uartHandle->printf("%\tOver Temperature Discharge%\r\n");
        if (flags & 0x0800) uartHandle->printf("%\tOver Temperature Charge%\r\n");

        return true;
    }
    return false;
}

bool BQ34::getVoltageRaw(uint16_t& mv) {
    if (readWord(VOLTAGE, mv)) {
        // debug print statement
        uartHandle->printf("Voltage (raw): %d mV%\r\n", mv);
        return true;
    }
    return false;
}