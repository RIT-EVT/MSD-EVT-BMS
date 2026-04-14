/**
@file BQ34.hpp
@brief C++ header for interfacing with the BQ34Z100-R2 battery fuel gauge.

This header defines the BQ34 class, which provides an abstraction layer for
communicating with the BQ34Z100-R2 over I2C. It exposes high-level functions
for retrieving battery parameters such as voltage, current, temperature, and
state of charge.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: November 2025
*/

#ifndef BQ34_HPP
#define BQ34_HPP

#include <core/io/I2C.hpp>


/* =========================
 * BQ34Z100-R2 Command Set
 * ========================= */

// Control register and subcommands

#define CONTROL             0x00

// Control sub-commands (used with CONTROL register)
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

// Standard command registers (see datasheet for full list)
#define SOC                 0x02    // State of Charge (%)
#define MAXERROR            0x03    // Maximum error (%)
#define REMAININGCAPACITY   0x04    // Remaining capacity (mAh)
#define FULLCHARGECAPACITY  0x06    // Full charge capacity (mAh)
#define VOLTAGE             0x08    // Battery voltage (mV)
#define TEMPERATURE         0x0C    // Temperature (0.1 K units)
#define FLAGSH              0x0E    // Status flags (high byte)
#define FLAGSL              0x0F    // Status flags (low byte)
#define AVERAGECURRENT      0x0A    // Average current (mA)
#define CURRENT             0x10    // Instantaneous current (mA)


/**
 * @class BQ34
 * @brief Interface class for the BQ34Z100-R2 fuel gauge
 *
 * Provides methods to read key battery parameters over I2C.
 * All reads are performed using 16-bit register accesses.
 *
 * @note Assumes I2C peripheral is initialized prior to use
 */
class BQ34 {
public:


    BQ34(core::io::I2C* i2c);

    bool readWord(uint8_t command, uint16_t& value);

    bool getVoltage(uint16_t& mv);
    bool getTemperature(uint16_t& t);
    bool getCurrent(uint16_t& mA);
    bool getSOC(uint16_t& soc);
    bool getMaxError(uint16_t& soh);
    bool getFlags(uint16_t& flags);
    bool getVoltageRaw(uint16_t& mv);

private:
    core::io::I2C* i2cHandle;   // Pointer to I2C interface used for communication
    static constexpr uint8_t ADDRESS = 0x55;  // 7-bit I2C device address
};

#endif
