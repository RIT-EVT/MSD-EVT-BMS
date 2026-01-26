/**
@file BQ34.cpp
@brief Implementation of the BQ79631 class for interfacing with the BQ79631 High voltage monitor
This file contains the definitions of the methods declared in BQ79631.hpp.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: January 2026
*/

#include "dev/BQ79631.hpp"

namespace core::dev {

/* ---- Register map (partial, expandable) ---- */
namespace {
constexpr uint16_t REG_DEVICE_ID      = 0x0000;
constexpr uint16_t REG_FAULT_STATUS   = 0x0010;
constexpr uint16_t REG_PACK_VOLTAGE   = 0x0020;
constexpr uint16_t REG_DIE_TEMP       = 0x0022;
constexpr uint16_t REG_CELL_COUNT     = 0x0024;
constexpr uint16_t REG_BALANCE_CTRL   = 0x0030;

constexpr uint16_t DEVICE_ID_EXPECTED = 0x7963;
}

BQ79631::BQ79631(core::io::I2C& i2c, uint8_t addr)
    : i2c_(i2c),
      addr_(addr),
      pack_voltage_mV_(0),
      die_temp_cC_(0),
      fault_flags_(0),
      cell_count_(0) {}

bool BQ79631::init() {
    uint16_t device_id = 0;
    if (!readWord(REG_DEVICE_ID, device_id)) {
        return false;
    }
    return (device_id == DEVICE_ID_EXPECTED);
}

bool BQ79631::update() {
    bool ok = true;

    ok &= readWord(REG_PACK_VOLTAGE, pack_voltage_mV_);
    ok &= readWord(REG_DIE_TEMP, reinterpret_cast<uint16_t&>(die_temp_cC_));
    ok &= readWord(REG_FAULT_STATUS, fault_flags_);
    ok &= readWord(REG_CELL_COUNT, cell_count_);

    return ok;
}

/* ---- Getters ---- */

uint16_t BQ79631::getPackVoltage_mV() const {
    return pack_voltage_mV_;
}

int16_t BQ79631::getDieTemperature_cC() const {
    return die_temp_cC_;
}

uint16_t BQ79631::getCellCount() const {
    return cell_count_;
}

uint16_t BQ79631::getFaultFlags() const {
    return fault_flags_;
}

bool BQ79631::hasFault() const {
    return fault_flags_ != 0;
}

/* ---- Control ---- */

bool BQ79631::enableBalancing(uint16_t cellMask) {
    return writeWord(REG_BALANCE_CTRL, cellMask);
}

bool BQ79631::disableBalancing() {
    return writeWord(REG_BALANCE_CTRL, 0x0000);
}

/* ---- Low-level I2C ---- */

bool BQ79631::writeWord(uint16_t reg, uint16_t value) {
    uint8_t buffer[2] = {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>(value >> 8)
    };

    auto status = i2c_.writeMemReg(
        addr_,                          // 7-bit device address
        static_cast<uint32_t>(reg),     // register address
        *buffer,                         // data buffer
        2,                              // write 2 bytes
        2                               // 16-bit register address
    );

    return (status == core::io::I2C::I2CStatus::OK);
}

bool BQ79631::readWord(uint16_t reg, uint16_t& value) {
    uint8_t buffer[2];

    auto status = i2c_.readMemReg(
        addr_,                          // 7-bit device address
        static_cast<uint32_t>(reg),     // register address
        buffer,                         // output buffer
        2,                              // read 2 bytes
        2                               // 16-bit register address
    );

    if (status != core::io::I2C::I2CStatus::OK) {
        return false;
    }

    // Little-endian conversion (TI standard)
    value = (uint16_t(buffer[1]) << 8) | buffer[0];
    return true;
}

} // namespace core::dev
