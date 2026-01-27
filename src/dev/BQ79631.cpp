/**
@file BQ34.cpp
@brief Implementation of the BQ79631 class for interfacing with the BQ79631 High voltage monitor
This file contains the definitions of the methods declared in BQ79631.hpp.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: January 2026
*/

#include "dev/BQ79631.hpp"
#include "core/utils/time.hpp"

namespace core::dev {

BQ79631::BQ79631(core::io::I2C& i2c, uint8_t addr)
    : i2c_(i2c), addr_(addr) {}

/* ---- Wake device ---- */
bool BQ79631::wake() {
    uint8_t dummy = 0x00;
    // Any write wakes the device
    return i2c_.write(addr_, &dummy, 1) == io::I2C::I2CStatus::OK;
}

/* ---- Send raw command ---- */
bool BQ79631::sendCommand(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    uint8_t frame[1 + len];
    frame[0] = cmd;
    if (payload && len > 0) {
        for (uint8_t i = 0; i < len; i++) {
            frame[i + 1] = payload[i];
        }
    }
    return i2c_.write(addr_, frame, 1 + len) == io::I2C::I2CStatus::OK;
}

/* ---- Read response ---- */
bool BQ79631::readResponse(uint8_t* buffer, uint8_t len) {
    return i2c_.read(addr_, buffer, len) == io::I2C::I2CStatus::OK;
}

/* ---- Device commands ---- */

constexpr uint8_t CRC8_POLY = 0x07;

uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLY;
            else
                crc <<= 1;
        }
    }
    return crc;
}

bool BQ79631::readDeviceID(uint16_t& id) {
    // Command frame: CMD=0x01, rest=0x00,0x00 (dummy), CRC
    uint8_t cmd[4] = {0x01, 0x00, 0x00, 0x00};
    cmd[3] = crc8(cmd, 3);

    if (i2c_.write(addr_, cmd, 4) != io::I2C::I2CStatus::OK)
        return false;

    // Wait for device to process
    core::time::wait(2);

    // Response frame: [MSB][LSB][CRC]
    uint8_t rx[3] = {0};
    if (i2c_.read(addr_, rx, 3) != io::I2C::I2CStatus::OK)
        return false;

    // Validate CRC
    if (rx[2] != crc8(rx, 2)) return false;

    id = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getPackVoltage_mV(uint16_t& mv) {
    if (!wake()) return false;
    core::time::wait(2);

    if (!sendCommand(0x04)) return false; // PACK_VOLTAGE command
    core::time::wait(2);

    uint8_t rx[2] = {0};
    if (!readResponse(rx, 2)) return false;

    mv = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getDieTemperature_cC(uint16_t& t) {
    if (!wake()) return false;
    core::time::wait(2);

    if (!sendCommand(0x05)) return false; // DIE_TEMP command
    core::time::wait(2);

    uint8_t rx[2] = {0};
    if (!readResponse(rx, 2)) return false;

    t = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getCellCount(uint16_t& count) {
    if (!wake()) return false;
    core::time::wait(2);

    if (!sendCommand(0x06)) return false; // CELL_COUNT command
    core::time::wait(2);

    uint8_t rx[2] = {0};
    if (!readResponse(rx, 2)) return false;

    count = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getFaultFlags(uint16_t& flags) {
    if (!wake()) return false;
    core::time::wait(2);

    if (!sendCommand(0x10)) return false; // FAULT_FLAGS command
    core::time::wait(2);

    uint8_t rx[2] = {0};
    if (!readResponse(rx, 2)) return false;

    flags = (rx[0] << 8) | rx[1];
    return true;
}

} // namespace core::dev