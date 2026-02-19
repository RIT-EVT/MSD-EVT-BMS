/**
@file BQ79600.hpp
@brief C++ header for interfacing with the BQ79600 bridge device.
This header defines a class for communicating with the BQ79600 over SPI
using STM32 HAL libraries.

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: Fabruary 2026
*/

#ifndef MSD_EVT_BMS_BQ79600_H
#define MSD_EVT_BMS_BQ79600_H


#include "core/io/SPI.hpp"
#include "core/io/UART.hpp"
#include "core/utils/time.hpp"
#include <cstdint>

namespace {
// Existing registers
constexpr uint16_t REG_CONTROL1     = 0x0309;
constexpr uint16_t REG_DIR0_ADDR    = 0x0306;
constexpr uint16_t REG_COMM_CTRL    = 0x0308;
constexpr uint16_t REG_DEV_CONF1    = 0x2001;

// Add these:
constexpr uint16_t REG_CONTROL2     = 0x030A;
constexpr uint16_t REG_TX_HOLD_OFF  = 0x0302;
constexpr uint16_t REG_SPI_CONF     = 0x0303;
constexpr uint16_t REG_FAULT_MSK1   = 0x031C;
constexpr uint16_t REG_FAULT_MSK2   = 0x031D;
constexpr uint16_t REG_FAULT_RST1   = 0x030C;
constexpr uint16_t REG_FAULT_RST2   = 0x030D;
constexpr uint16_t REG_FAULT_SUMMARY = 0x0309;
constexpr uint16_t REG_GPIO_CONF1   = 0x0320;
constexpr uint16_t REG_GPIO_CONF2   = 0x0321;
constexpr uint16_t REG_OTP_ECC_BASE = 0x0343;
constexpr uint8_t  OTP_ECC_COUNT    = 8;
}

namespace core::dev {

class BQ79600 {
public:
    explicit BQ79600(core::io::SPI& spi, uint8_t spi_device, core::io::UART& uart);

    bool readDeviceID(uint16_t& id);

    /** Wake the device (send any I2C activity to wake) */
    bool wake();

    bool ping();

    bool broadcastWrite(uint16_t reg, uint8_t val);
    bool stackWrite(uint16_t reg, uint8_t val);
    bool singleWrite(uint8_t dev, uint16_t reg, uint8_t val);
    bool singleRead(uint8_t dev, uint16_t reg, uint8_t& val);
    bool stackRead(uint8_t dev, uint16_t reg, uint8_t& val);

    bool autoAddressStack(uint8_t expected_devices);

    bool initRegisters();


    // Test methods
    void runSPITests();
    bool testBasicSPI();
    bool testReadWrite();
    bool testWakeSequence();
    void loopbackTest();



private:
    core::io::SPI& spi_;
    uint8_t device_;
    core::io::UART& uart_;

    static uint16_t crc16(const uint8_t* data, uint8_t len);

    bool writeReg16(uint8_t dev, uint16_t reg, uint8_t val, bool stack, bool broadcast);

    bool readReg16(uint8_t dev, uint16_t reg, uint8_t& val, bool stack);

};

} // namespace core::dev


#endif//MSD_EVT_BMS_BQ79600_H
