/**
@file BQ79631.cpp
@brief Implementation of the BQ79631 class for interfacing with the BQ79631 High voltage monitor.
This file contains the definitions of the methods declared in BQ79631.hpp.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: January 2026
*/

#include "dev/BQ79631.hpp"
#include "core/manager.hpp"
#include "core/utils/time.hpp"

#define BMS_DEBUG

namespace core::dev {

namespace {
    constexpr uint32_t WAKE_DELAY_MS = 5;
}

// BQ79631 Register Map Base Addresses
namespace {
    constexpr uint16_t PARTID       = 0x0500;
    constexpr uint16_t REVID        = 0x0E00;
    constexpr uint16_t DIE1_TEMP_HI   = 0x05AE;
    constexpr uint16_t DIE1_TEMP_LO   = 0x05AF;
    constexpr uint16_t DIE2_TEMP_HI   = 0x05B0;
    constexpr uint16_t DIE2_TEMP_LO   = 0x05B1;
    // need to verify these registers
    constexpr uint16_t PACK_VOLTAGE = 0x0004;
    constexpr uint16_t CELL_COUNT   = 0x0006;
    constexpr uint16_t FAULT_FLAGS  = 0x0010;
}

// HVM instance
BQ79631::BQ79631(BQ79600& bridge, uint8_t stack_address)
    : bridge_(bridge), stack_address_(stack_address) {}

/* ---- Device commands ---- */

bool BQ79631::readDeviceID(uint16_t& id) {
    uint8_t hi = 0, lo = 0;

    if (!bridge_.singleRead(stack_address_, PARTID, hi)) return false;
    if (!bridge_.singleRead(stack_address_, PARTID + 1, lo)) return false;

    id = (uint16_t(hi) << 8) | lo;
    return true;
}

bool BQ79631::readRevision(uint8_t &rev) {
    uint8_t rx = 0;

    if (!bridge_.singleRead(stack_address_, REVID, rx)) return false;

    rev = rx;
    return true;
}

bool BQ79631::getPackVoltage_mV(uint16_t& mv) {
    uint8_t hi = 0, lo = 0;

    if (!bridge_.singleRead(stack_address_, PACK_VOLTAGE, hi)) return false;
    if (!bridge_.singleRead(stack_address_, PACK_VOLTAGE + 1, lo)) return false;

    mv = (uint16_t(hi) << 8) | lo;
    return true;
}

bool BQ79631::getDieTemperature_cC(uint16_t& t) {
    uint8_t hi = 0, lo = 0;

    if (!bridge_.singleRead(stack_address_, DIE1_TEMP_HI, hi)) return false;
    if (!bridge_.singleRead(stack_address_, DIE1_TEMP_LO, lo)) return false;

    t = (uint16_t(hi) << 8) | lo;
    return true;
}

bool BQ79631::getCellCount(uint16_t& count) {
    uint8_t hi = 0, lo = 0;

    if (!bridge_.singleRead(stack_address_, CELL_COUNT, hi)) return false;
    if (!bridge_.singleRead(stack_address_, CELL_COUNT + 1, lo)) return false;

    count = (uint16_t(hi) << 8) | lo;
    return true;
}

bool BQ79631::getFaultFlags(uint16_t& flags) {
    uint8_t hi = 0, lo = 0;

    if (!bridge_.singleRead(stack_address_, FAULT_FLAGS, hi)) return false;
    if (!bridge_.singleRead(stack_address_, FAULT_FLAGS + 1, lo)) return false;

    flags = (uint16_t(hi) << 8) | lo;
    return true;
}

} // namespace core::dev