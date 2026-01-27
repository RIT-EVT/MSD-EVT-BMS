/**
@file BQ79631.hpp
@brief C++ header for interfacing with the BQ79631 high voltage monitor.
This header defines a class for communicating with the BQ79631 over I2C
using STM32 HAL libraries.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: January 2026
*/

#ifndef MSD_EVT_BMS_BQ79631_H
#define MSD_EVT_BMS_BQ79631_H

#include <cstdint>
#include "core/io/I2C.hpp"

namespace core::dev {

class BQ79631 {
public:
    explicit BQ79631(core::io::I2C& i2c, uint8_t addr = 0x18);

    /** Wake the device (send any I2C activity to wake) */
    bool wake();

    /** Read DEVICE_ID (should return 0x7963) */
    bool readDeviceID(uint16_t& id);

    /** Read pack voltage (mV) */
    bool getPackVoltage_mV(uint16_t& mv);

    /** Read die temperature (centi-Celsius) */
    bool getDieTemperature_cC(uint16_t& t);

    /** Read cell count */
    bool getCellCount(uint16_t& count);

    /** Read fault flags */
    bool getFaultFlags(uint16_t& flags);

private:
    core::io::I2C& i2c_;
    uint8_t addr_;

    /** Send raw command frame */
    bool sendCommand(uint8_t cmd, const uint8_t* payload = nullptr, uint8_t len = 0);

    /** Read response frame */
    bool readResponse(uint8_t* buffer, uint8_t len);
};

} // namespace core::dev

#endif // MSD_EVT_BMS_BQ79631_H

