/**
@file BQ79616.hpp
@brief Declaration of the BQ79616 class for the 16-channel cell monitor slave

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: April 2026
*/

#pragma once

#include "dev/BQ79600.hpp" // Include your Bridge driver
#include <cstdint>

namespace core::dev {

class BQ79616 {
public:
    /**
     * @brief Construct a new BQ79616 Slave Device
     * * @param bridge Reference to the BQ79600 Bridge handling the physical bus
     * @param stack_address The daisy-chain address assigned to this slave (e.g., 0, 1, 2)
     */
    BQ79616(BQ79600& bridge, uint8_t stack_address);

    // Device Info
    bool readDeviceID(uint16_t& id);
    bool readRevision(uint8_t& rev);

    // Core Measurements
    bool getCellVoltages_mV(uint16_t* voltages, uint8_t num_cells = 16);
    bool getThermistors_cC(uint16_t* temps, uint8_t num_thermistors);

    // Safety & Diagnostics
    bool getDieTemperature_cC(uint16_t& t);
    bool getFaultFlags(uint16_t& flags);

    // Balancing
    bool setBalancingMask(uint16_t cell_mask);
    bool stopBalancing();

private:
    BQ79600& bridge_;       // The communication bridge
    uint8_t stack_address_; // This device's specific address
};

} // namespace core::dev