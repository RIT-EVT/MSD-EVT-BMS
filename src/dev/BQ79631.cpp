/**
@file BQ79631.cpp
@brief Implementation of the BQ79631 class for interfacing with the BQ79631 High voltage monitor
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
// registers
namespace {
constexpr uint16_t PARTID     = 0x0500;
constexpr uint16_t REVID      = 0xE00;


}

// HVM instance
BQ79631::BQ79631(core::io::SPI& spi, uint8_t spi_device, core::io::UART& uart)
    : spi_(spi), device_(spi_device), uart_(uart){}

// // bridge instance
//  BQ79600::BQ79600(core::io::SPI& spi, uint8_t spi_device, core::io::UART& uart)
//     : spi_(spi), device_(spi_device), uart_(uart){}




/* ---- Device commands ---- */


bool BQ79631::readDeviceID(uint16_t& id) {
    uint8_t rx[2] = {0};

    // if (!wake()) return false;
    // if (!BQ79600::singleRead(0x0, PARTID, rx[0])) return false;

    id = (uint16_t(rx[0]) << 8) | rx[1];
    return true;
}

bool BQ79631::readRevision(uint8_t &rev) {
    uint8_t rx = 0;

    // if (!wake()) return false;

    // if (!readRegister(0x0501, &rx, 1)) return false;

    rev = rx;
    return true;
}


bool BQ79631::getPackVoltage_mV(uint16_t& mv) {
    uint8_t rx[2] = {0};

    // if (!wake()) return false;
    // if (!readRegister(0x0004, rx, 2)) return false;

    mv = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getDieTemperature_cC(uint16_t& t) {
    uint8_t rx[2] = {0};

    // if (!wake()) return false;
    // if (!readRegister(0x0005, rx, 2)) return false;

    t = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getCellCount(uint16_t& count) {
    uint8_t rx[2] = {0};

    // if (!wake()) return false;
    // if (!readRegister(0x0006, rx, 2)) return false;

    count = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getFaultFlags(uint16_t& flags) {
    uint8_t rx[2] = {0};

    // if (!wake()) return false;
    // if (!readRegister(0x0010, rx, 2)) return false;

    flags = (rx[0] << 8) | rx[1];
    return true;
}

} // namespace core::dev