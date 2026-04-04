/** 
@file BQ34.cpp
@brief Implementation of the BQ34 class for interfacing with the BQ34Z100-R2 battery fuel gauge.
This file contains the definitions of the methods declared in BQ34.hpp.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: November 2025
*/

#include "dev/BQ34.hpp"

/**
 * @brief Construct a new BQ34 object
 *
 * Initializes the BQ34 interface with a provided I2C handle.
 *
 * @param i2c Pointer to initialized I2C interface used for communication
 */
BQ34::BQ34(core::io::I2C* i2c) : i2cHandle(i2c) {}

/**
 * @brief Read a 16-bit word from the BQ34Z100-R2
 *
 * Performs an I2C memory read using a 1-byte command/register address and
 * retrieves a 2-byte (16-bit) value from the device.
 *
 * @param command Register/command address to read from
 * @param value Reference to variable where the result will be stored
 * @return true if read operation was successful
 * @return false if I2C transaction failed
 *
 * @note Data is returned in little-endian format and converted to host format
 */
bool BQ34::readWord(uint8_t command, uint16_t& value) {
    uint8_t buffer[2]; // Temporary buffer for raw I2C data (LSB first)

    // Uses 1-byte memory address (standard for BQ34Z100)
    auto status = i2cHandle->readMemReg(
        ADDRESS,                         // 7-bit address
        static_cast<uint32_t>(command),  // register/command address
        buffer,                          // output buffer
2,                                       // number of bytes to read
        1                                // memory address size (bytes)
    );

    if (status != core::io::I2C::I2CStatus::OK)
        return false;

    // Convert little-endian from BQ34Z100
    value = (uint16_t(buffer[1]) << 8) | buffer[0];

    return true;
}

/**
 * @brief Get battery voltage
 *
 * Reads the measured battery voltage from the fuel gauge.
 *
 * @param mv Reference to store voltage in millivolts
 * @return true on success, false on failure
 */
bool BQ34::getVoltage(uint16_t& mv) {
    return readWord(VOLTAGE, mv);
}

/**
 * @brief Get battery temperature
 *
 * Reads the temperature value reported by the fuel gauge.
 *
 * @param t Reference to store temperature (units per device datasheet)
 * @return true on success, false on failure
 */
bool BQ34::getTemperature(uint16_t& t) {
    return readWord(TEMPERATURE, t);
}

/**
 * @brief Get battery current
 *
 * Reads the signed current value from the fuel gauge.
 * Positive/negative direction is defined by the device configuration.
 *
 * @param mA Reference to store current in milliamps
 * @return true on success, false on failure
 *
 * @note Internally casts to uint16_t for raw read, then interpreted as signed
 */
bool BQ34::getCurrent(int16_t& mA) {
    return readWord(CURRENT, reinterpret_cast<uint16_t&>(mA));
}

/**
 * @brief Get state of charge (SOC)
 *
 * Reads the battery state of charge as a percentage.
 *
 * @param soc Reference to store SOC (%)
 * @return true on success, false on failure
 */
bool BQ34::getSOC(uint16_t& soc) {
    return readWord(SOC, soc);
}

/**
 * @brief Get maximum error estimate
 *
 * Reads the maximum error percentage reported by the fuel gauge.
 *
 * @param error Reference to store error (%)
 * @return true on success, false on failure
 */
bool BQ34::getMaxError(uint16_t& error) {
    return readWord(MAXERROR, error);
}

/**
 * @brief Get status flags
 *
 * Reads the status flag register from the device.
 * Flags indicate battery and system conditions.
 *
 * @param flags Reference to store flag bitfield
 * @return true on success, false on failure
 */
bool BQ34::getFlags(uint16_t& flags) {
    return readWord(FLAGSH, flags);
}

/**
 * @brief Get raw voltage register value
 *
 * Reads the raw voltage register without additional interpretation.
 * Useful for debugging or validation.
 *
 * @param mv Reference to store raw voltage value
 * @return true on success, false on failure
 */
bool BQ34::getVoltageRaw(uint16_t& mv) {
    return readWord(VOLTAGE, mv);
}