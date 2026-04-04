/**
@file BQ79616.cpp
@brief Implementation of the BQ79616 class using the BQ79600 bridge.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: April 2026
*/

#include "dev/BQ79616.hpp"

#include "core/manager.hpp"
#include "core/utils/time.hpp"

#define BMS_DEBUG

namespace core::dev {

namespace {
    constexpr uint32_t WAKE_DELAY_MS = 5;
}

// BQ79616 Register Map (Check datasheet for exact addresses)
namespace {
    constexpr uint16_t PARTID         = 0x0500;
    constexpr uint16_t REVID          = 0x0E00;
    constexpr uint16_t FAULT_SUMMARY  = 0x052D;
    constexpr uint16_t DIE1_TEMP_HI   = 0x05AE;
    constexpr uint16_t DIE1_TEMP_LO   = 0x05AF;
    constexpr uint16_t DIE2_TEMP_HI   = 0x05B0;
    constexpr uint16_t DIE2_TEMP_LO   = 0x05B1;

    // need to verify these registers
    constexpr uint16_t VCELL1_HI      = 0x0041; // Cell 1 High Byte
    constexpr uint16_t CB_CELL_CTRL   = 0x0061; // Cell balancing control
}

// Constructor passing the bridge by reference
BQ79616::BQ79616(BQ79600& bridge, uint8_t stack_address)
    : bridge_(bridge), stack_address_(stack_address) {}

/* ---- Device commands ---- */

bool BQ79616::readDeviceID(uint16_t& id) {
    uint8_t hi = 0, lo = 0;

    // Assuming PARTID spans two bytes
    if (!bridge_.singleRead(stack_address_, PARTID, hi)) return false;
    if (!bridge_.singleRead(stack_address_, PARTID + 1, lo)) return false;

    id = (uint16_t(hi) << 8) | lo;
    return true;
}

bool BQ79616::readRevision(uint8_t &rev) {
    uint8_t rx = 0;

    if (!bridge_.singleRead(stack_address_, REVID, rx)) return false;

    rev = rx;
    return true;
}

bool BQ79616::getCellVoltages_mV(uint16_t* voltages, uint8_t num_cells) {

    // Fallback implementation: Read bytes sequentially since block read isn't available
    for (uint8_t i = 0; i < num_cells; i++) {
        uint8_t hi = 0, lo = 0;

        // Calculate register addresses: VCELL1_HI, VCELL1_LO, VCELL2_HI, etc.
        uint16_t reg_hi = VCELL1_HI + (i * 2);
        uint16_t reg_lo = VCELL1_HI + (i * 2) + 1;

        if (!bridge_.singleRead(stack_address_, reg_hi, hi)) return false;
        if (!bridge_.singleRead(stack_address_, reg_lo, lo)) return false;

        voltages[i] = (uint16_t(hi) << 8) | lo;
    }

    return true;
}

bool BQ79616::getThermistors_cC(uint16_t* temps, uint8_t num_thermistors) {
    // Implement similar loop logic here for GPIO/AUX ADC registers
    return true;
}

bool BQ79616::getDieTemperature_cC(uint16_t& t) {
    uint8_t hi = 0, lo = 0;

    if (!bridge_.singleRead(stack_address_, DIE1_TEMP_HI, hi)) return false;
    if (!bridge_.singleRead(stack_address_, DIE1_TEMP_LO, lo)) return false;

    t = (uint16_t(hi) << 8) | lo;
    return true;
}

bool BQ79616::getFaultFlags(uint16_t& flags) {
    uint8_t hi = 0, lo = 0;

    if (!bridge_.singleRead(stack_address_, FAULT_SUMMARY, hi)) return false;
    if (!bridge_.singleRead(stack_address_, FAULT_SUMMARY + 1, lo)) return false;

    flags = (uint16_t(hi) << 8) | lo;
    return true;
}

/* ---- Balancing Commands ---- */

bool BQ79616::setBalancingMask(uint16_t cell_mask) {
    uint8_t hi = static_cast<uint8_t>(cell_mask >> 8);
    uint8_t lo = static_cast<uint8_t>(cell_mask & 0xFF);

    // Assuming your bridge has a singleWrite function
    // if (!bridge_.singleWrite(stack_address_, CB_CELL_CTRL, hi)) return false;
    // if (!bridge_.singleWrite(stack_address_, CB_CELL_CTRL + 1, lo)) return false;

    return true;
}

bool BQ79616::stopBalancing() {
    return setBalancingMask(0x0000);
}

} // namespace core::dev