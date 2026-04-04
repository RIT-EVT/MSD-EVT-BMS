/**
@file BQ79631.hpp
@brief C++ header for interfacing with the BQ79631 high voltage monitor.
This header defines a class for communicating with the BQ79631 over the
daisy-chain bus via the BQ79600 bridge.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: January 2026
*/

#ifndef MSD_EVT_BMS_BQ79631_H
#define MSD_EVT_BMS_BQ79631_H

#include "dev/BQ79600.hpp"
#include <cstdint>

namespace core::dev {

class BQ79631 {
public:
    /**
     * @brief Construct a new BQ79631 HVM Device
     * @param bridge Reference to the BQ79600 Bridge handling the physical bus
     * @param stack_address The daisy-chain address assigned to this device
     */
    BQ79631(BQ79600& bridge, uint8_t stack_address);

    /** Read DEVICE_ID */
    bool readDeviceID(uint16_t& id);

    /** Read DEVICE REVISION */
    bool readRevision(uint8_t& rev);

    /** Read pack voltage (mV) */
    bool getPackVoltage_mV(uint16_t& mv);

    /** Read die temperature (centi-Celsius) */
    bool getDieTemperature_cC(uint16_t& t);

    /** Read cell count */
    bool getCellCount(uint16_t& count);

    /** Read fault flags */
    bool getFaultFlags(uint16_t& flags);

private:
    BQ79600& bridge_;       // The communication bridge
    uint8_t stack_address_; // This device's specific daisy-chain address
};

} // namespace core::dev

#endif // MSD_EVT_BMS_BQ79631_H