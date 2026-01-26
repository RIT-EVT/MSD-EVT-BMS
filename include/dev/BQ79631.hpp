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
    /**
     * @brief Construct a BQ79631 device
     * @param i2c Reference to initialized I2C bus
     * @param addr I2C address of the BQ79631 (default 0x08 typical)
     */
    explicit BQ79631(core::io::I2C& i2c, uint8_t addr = 0x08);

    /**
     * @brief Initialize the device
     * @return true if device responds correctly
     */
    bool init();

    /**
     * @brief Poll device measurements and status
     * @return true if successful
     */
    bool update();

    /* ---- Measurements ---- */

    uint16_t getPackVoltage_mV() const;
    int16_t  getDieTemperature_cC() const;   // centi-Celsius
    uint16_t getCellCount() const;

    /* ---- Fault / Status ---- */

    uint16_t getFaultFlags() const;
    bool     hasFault() const;

    /* ---- Control ---- */

    bool enableBalancing(uint16_t cellMask);
    bool disableBalancing();

private:
    core::io::I2C& i2c_;
    uint8_t addr_;

    /* Cached values */
    uint16_t pack_voltage_mV_;
    int16_t  die_temp_cC_;
    uint16_t fault_flags_;
    uint16_t cell_count_;

    /* ---- Low-level helpers ---- */

    bool writeWord(uint16_t reg, uint16_t value);
    bool readWord(uint16_t reg, uint16_t& value);
};

} // namespace core::dev

#endif//MSD_EVT_BMS_BQ79631_H

