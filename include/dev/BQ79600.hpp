/**
@file BQ79600.hpp
@brief C++ header for interfacing with the BQ79600 bridge device.

This header defines the BQ79600 class, which provides an interface for
SPI communication with the BQ79600 bridge IC. The bridge manages
communication between the MCU and daisy-chained battery monitor devices.

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

    bool readDeviceID(uint16_t& id) const;

    bool init();

    void broadcastWrite(uint16_t reg, uint8_t val) const;
    void stackWrite(uint16_t reg, uint8_t val) const;
    void singleWrite(uint8_t dev, uint16_t reg, uint8_t val) const;
    bool singleRead(uint8_t dev, uint16_t reg, uint8_t& val) const;
    bool stackRead(uint16_t reg, uint8_t& val) const;

    void autoAddressStack(uint8_t expected_devices) const;

    bool initRegisters();

private:
    core::io::SPI& spi_;   // SPI interface used for communication
    uint8_t device_;       // SPI chip select index / device identifier
    core::io::UART& uart_; // UART interface for debug output

    static uint16_t crc16(const uint8_t* data, uint8_t len); // CRC16 calculation for frame integrity

    void writeReg16(uint8_t dev, uint16_t reg, uint8_t val, bool stack, bool broadcast) const; // Low-level write helper

    bool readReg16(uint8_t dev, uint16_t reg, uint8_t& val, bool stack) const; // Low-level read helper
};

} // namespace core::dev


#endif//MSD_EVT_BMS_BQ79600_H
