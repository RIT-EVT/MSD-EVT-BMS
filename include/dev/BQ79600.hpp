/**
@file BQ79600.hpp
@brief C++ header for interfacing with the BQ79600 bridge device.
This header defines a class for communicating with the BQ79600 over SPI
using STM32 HAL libraries.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: February 2026
*/

#ifndef MSD_EVT_BMS_BQ79600_H
#define MSD_EVT_BMS_BQ79600_H


#include "core/io/SPI.hpp"
#include "core/io/UART.hpp"
#include "core/utils/time.hpp"
#include <cstdint>

namespace core::dev {

class BQ79600 {
public:
    explicit BQ79600(core::io::SPI& spi, uint8_t spi_device, core::io::UART& uart);

    bool readDeviceID(uint16_t& id);

    bool init();

    void broadcastWrite(uint16_t reg, uint8_t val) const;
    void stackWrite(uint16_t reg, uint8_t val) const;
    void singleWrite(uint8_t dev, uint16_t reg, uint8_t val) const;
    bool singleRead(uint8_t dev, uint16_t reg, uint8_t& val) const;
    bool stackRead(uint16_t reg, uint8_t& val) const;

    void autoAddressStack(uint8_t expected_devices) const;

    bool initRegisters();

private:
    core::io::SPI& spi_;
    uint8_t device_;
    core::io::UART& uart_;

    static uint16_t crc16(const uint8_t* data, uint8_t len);

    void writeReg16(uint8_t dev, uint16_t reg, uint8_t val, bool stack, bool broadcast) const;

    bool readReg16(uint8_t dev, uint16_t reg, uint8_t& val, bool stack) const;

};

} // namespace core::dev


#endif//MSD_EVT_BMS_BQ79600_H
