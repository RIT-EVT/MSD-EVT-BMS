/**
@file BQ79631.hpp
@brief C++ header for interfacing with the BQ79631 high voltage monitor.
This header defines a class for communicating with the BQ79631 over SPI
using STM32 HAL libraries.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: January 2026
*/

#ifndef MSD_EVT_BMS_BQ79631_H
#define MSD_EVT_BMS_BQ79631_H

#include "core/io/SPI.hpp"
#include "core/io/UART.hpp"
#include <cstdint>

namespace core::dev {

class BQ79631 {
public:
    explicit BQ79631(core::io::SPI& spi, uint8_t spi_device, core::io::UART& uart);

    bool writeReg16(uint8_t dev, uint16_t reg, uint8_t val);

    bool readReg16(uint8_t dev, uint16_t reg, uint8_t& val);

    /** Wake the device (send any I2C activity to wake) */
    bool wake();

    bool ping();

    /** Read DEVICE_ID (should return 0x7963) */
    bool readDeviceID(uint16_t& id);

    /** Read DEVICE REVISION (should return 0x7963) */
    bool readRevision(uint8_t& id);

    /** Read pack voltage (mV) */
    bool getPackVoltage_mV(uint16_t& mv);

    /** Read die temperature (centi-Celsius) */
    bool getDieTemperature_cC(uint16_t& t);

    /** Read cell count */
    bool getCellCount(uint16_t& count);

    /** Read fault flags */
    bool getFaultFlags(uint16_t& flags);

private:
    core::io::SPI& spi_;
    uint8_t device_;
    core::io::UART& uart_;

    /** Command helpers */
    bool readRegister(uint16_t reg, uint8_t* data, uint8_t len) const;
};

} // namespace core::dev

#endif // MSD_EVT_BMS_BQ79631_H

