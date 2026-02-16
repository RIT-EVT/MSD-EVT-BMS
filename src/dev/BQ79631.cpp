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
namespace {


}


BQ79631::BQ79631(core::io::SPI& spi, uint8_t spi_device, core::io::UART& uart)
    : spi_(spi), device_(spi_device), uart_(uart){}


// bool BQ79631::readReg16(uint8_t dev, uint16_t reg, uint8_t& val)
// {
//     uint8_t tx_frame[7];
//
//     tx_frame[0] = 0x80;              // single-device read, 1 byte
//     tx_frame[1] = dev;
//     tx_frame[2] = (reg >> 8) & 0xFF; // reg MSB
//     tx_frame[3] = reg & 0xFF;        // reg LSB
//     tx_frame[4] = 0x00;              // dummy
//
//     uint16_t crc = crc16_ccitt(tx_frame, 5);
//     tx_frame[5] = (crc >> 8) & 0xFF;
//     tx_frame[6] = crc & 0xFF;
//
//     // DEBUG: Print TX frame
//     #ifdef BMS_DEBUG
//     // extern core::io::UART* uart;
//         uart_.printf("READ TX [%d bytes]: ", 7);
//         for (int i = 0; i < 7; i++) {
//             uart_.printf("%02X ", tx_frame[i]);
//         }
//         uart_.printf("(CRC=%04X)\r\n", crc);
//     #endif
//
//     if (!spi_.startTransmission(device_))
//         return false;
//
//     // Send command
//     for (uint8_t b : tx_frame)
//         spi_.write(b);
//
//     // According to datasheet: "After a read command frame is transmitted,
//     // the host must wait for all expected responses to return"
//     // Keep CS LOW and continue clocking to receive response
//
//     // Response frame format: [DEV ADR][REG ADR_H][REG ADR_L][DATA][CRC_H][CRC_L]
//     // Total: 6 bytes for single-byte read
//     uint8_t rx_frame[6];
//
//     for (uint8_t i = 0; i < 6; i++) {
//         spi_.write(0x00);  // Clock out response
//         spi_.read(&rx_frame[i]);
//     }
//
//     spi_.endTransmission(device_);
//
//     #ifdef BMS_DEBUG
//         uart_.printf("READ RX [%d bytes]: ", 6);
//         for (int i = 0; i < 6; i++) {
//             uart_.printf("%02X ", rx_frame[i]);
//         }
//         uart_.printf("\r\n");
//     #endif
//
//     // Parse response frame
//     uint8_t dev_addr = rx_frame[0];
//     uint16_t reg_addr = (rx_frame[1] << 8) | rx_frame[2];
//     val = rx_frame[3];
//     uint16_t rx_crc = (rx_frame[4] << 8) | rx_frame[5];
//
//     // Calculate CRC over [DEV ADR][REG ADR_H][REG ADR_L][DATA]
//     uint16_t calc_crc = crc16_ccitt(rx_frame, 4);
//
//     #ifdef BMS_DEBUG
//         uart_.printf("  Dev: 0x%02X (expect 0x%02X) %s\r\n",
//                      dev_addr, dev, (dev_addr == dev) ? "OK" : "FAIL");
//         uart_.printf("  Reg: 0x%04X (expect 0x%04X) %s\r\n",
//                      reg_addr, reg, (reg_addr == reg) ? "OK" : "FAIL");
//         uart_.printf("  Val: 0x%02X\r\n", val);
//         uart_.printf("  CRC: RX=0x%04X CALC=0x%04X %s\r\n",
//                      rx_crc, calc_crc, (calc_crc == rx_crc) ? "OK" : "FAIL");
//     #endif
//
//     return (calc_crc == rx_crc) && (dev_addr == dev) && (reg_addr == reg);
// }
// namespace {
//
// uint16_t crc16_ccitt(const uint8_t* data, uint8_t len) {
//     uint16_t crc = 0xFFFF;
//
//     for (uint8_t i = 0; i < len; i++) {
//         crc ^= (uint16_t)data[i] << 8;
//         for (uint8_t b = 0; b < 8; b++) {
//             crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
//         }
//     }
//     return crc;
// }
//
// }
//
//
// bool BQ79631::writeReg16(uint8_t dev, uint16_t reg, uint8_t val)
// {
//     uint8_t frame[7];
//
//     frame[0] = 0x90;              // single-device write, 1 byte
//     frame[1] = dev;               // device address
//     frame[2] = (reg >> 8) & 0xFF; // reg MSB
//     frame[3] = reg & 0xFF;        // reg LSB
//     frame[4] = val;               // data
//
//     uint16_t crc = crc16_ccitt(frame, 5);
//     frame[5] = (crc >> 8) & 0xFF;
//     frame[6] = crc & 0xFF;
//
//     if (!spi_.startTransmission(device_))
//         return false;
//
//     for (uint8_t b : frame)
//         spi_.write(b);
//
//     spi_.endTransmission(device_);
//     return true;
// }
//
// bool BQ79631::readReg16(uint8_t dev, uint16_t reg, uint8_t& val)
// {
//     uint8_t frame[7];
//
//     frame[0] = 0x80;              // single-device read, 1 byte
//     frame[1] = dev;
//     frame[2] = (reg >> 8) & 0xFF;
//     frame[3] = reg & 0xFF;
//     frame[4] = 0x00;              // dummy
//
//     uint16_t crc = crc16_ccitt(frame, 5);
//     frame[5] = (crc >> 8) & 0xFF;
//     frame[6] = crc & 0xFF;
//
//     if (!spi_.startTransmission(device_))
//         return false;
//
//     for (uint8_t b : frame)
//         spi_.write(b);
//
//     // Read response
//     uint8_t rx_frame[7] = {0};
//     for (uint8_t i = 0; i < 7; i++) {
//         spi_.write(0x00);  // Clock out data
//         spi_.read(&rx_frame[i]);
//     }
//
//     spi_.endTransmission(device_);
//
//     // Parse response
//     for (uint8_t i = 0; i < 4; i++) {
//         spi_.write(0x00);  // Clock out data
//         spi_.read(&rx_frame[i]);
//     }
//
//     spi_.endTransmission(device_);
//
//     // Parse response
//     uint8_t dev_addr = rx_frame[0];
//     val = rx_frame[1];
//     uint16_t rx_crc = (rx_frame[2] << 8) | rx_frame[3];
//
//     // Validate CRC over [dev_addr][data]
//     uint16_t calc_crc = crc16_ccitt(rx_frame, 2);
//     uart->printf("dev_addr: 0x%X, val: 0x%X, rx_crc: 0x%X, crc: 0x%X\r\n", dev_addr, val, rx_crc, calc_crc);
//
//
//     return (calc_crc == rx_crc) && (dev_addr == dev);
//
//     // uint8_t dev_addr = rx_frame[1];
//     // val = rx_frame[4];
//     // uint16_t rx_crc = (rx_frame[5] << 8) | rx_frame[6];
//     //
//     // // Validate CRC over [dev_addr][data]
//     // uint16_t calc_crc = crc16_ccitt(rx_frame, 5);
//     // core::io::UART* uart = &io::getUART<core::io::Pin::UART_TX, core::io::Pin::UART_RX>(9600); // uart init
//     // uart->printf("dev_addr: 0x%X, val: 0x%X, rx_crc: 0x%X, crc: 0x%X\r\n", dev_addr, val, rx_crc, calc_crc);
//     // return (calc_crc == rx_crc) && (dev_addr == dev);
//
//     // uint16_t rx_crc = (rx_frame[5] << 8) | rx_frame[6];
//     // uint16_t calc_crc = crc16_ccitt(rx_frame, 5);
//     //
//     // if (calc_crc != rx_crc)
//     //     return false;
//     //
//     // if (rx_frame[1] != dev)
//     //     return false;
//     //
//     // val = rx_frame[4];
//     // return true;
// }

/* ---- Wake chain ---- */
bool BQ79631::wake() {
    if (!spi_.startTransmission(device_))
        return false;

    for (int i = 0; i < 480; i++){
        spi_.write(0x00);  // 32 clocks
        core::time::wait(WAKE_DELAY_MS);
    }

    // core::time::wait(3);
    spi_.endTransmission(device_);
    core::time::wait(WAKE_DELAY_MS);
    return true;
}

bool BQ79631::ping() {
    uint8_t frame[2];
    frame[0] = 0x00;  // No-op/wake byte
    frame[1] = 0x00;

    if (!spi_.startTransmission(device_))
        return false;

    for (int i = 0; i < 16; i++)  // 128 clocks
        spi_.write(0x00);

    spi_.endTransmission(device_);
    core::time::wait(10);

    return true;
}

/* ---- Register Read (HVM only) ---- */
bool BQ79631::readRegister(uint16_t reg, uint8_t* data, uint8_t len) const {

    if (!data || len == 0 || len > 32)
        return false;

    /*
     * READ COMMAND FRAME
     * [CMD][ADDR_H][ADDR_L][LEN-1][CRC_H][CRC_L]
     */

    uint8_t cmd[6];
    cmd[0] = 0xC0;                     // Broadcast READ
    cmd[1] = (reg >> 8) & 0xFF;
    cmd[2] = reg & 0xFF;
    cmd[3] = len - 1;

    // uint16_t crc = crc16_ccitt(cmd, 4);
    // cmd[4] = crc >> 8;
    // cmd[5] = crc & 0xFF;

    // Phase 1: send command
    if (!spi_.startTransmission(device_))
        return false;

    for (uint8_t b : cmd)
        spi_.write(b);

    spi_.endTransmission(device_);

    core::time::wait(1);

    // Phase 2: read response
    if (!spi_.startTransmission(device_))
        return false;

    for (uint8_t i = 0; i < len; i++) {
        spi_.write(0x00);              // clock data out
        spi_.read(&data[i]);
    }

    uint8_t crc_rx[2];
    spi_.write(0x00);
    spi_.read(&crc_rx[0]);
    spi_.write(0x00);
    spi_.read(&crc_rx[1]);

    spi_.endTransmission(device_);

    uint16_t rx_crc = (uint16_t(crc_rx[0]) << 8) | crc_rx[1];
     // return (crc16_ccitt(data, len) == rx_crc);
    return 0;
}

/* ---- Device commands ---- */


bool BQ79631::readDeviceID(uint16_t& id) {
    uint8_t rx[2] = {0};

    if (!wake()) return false;
    if (!readRegister(0x0500, rx, 2)) return false;

    id = (uint16_t(rx[0]) << 8) | rx[1];
    return true;
}

bool BQ79631::readRevision(uint8_t &rev) {
    uint8_t rx = 0;

    if (!wake()) return false;

    if (!readRegister(0x0501, &rx, 1)) return false;

    rev = rx;
    return true;
}


bool BQ79631::getPackVoltage_mV(uint16_t& mv) {
    uint8_t rx[2] = {0};

    if (!wake()) return false;
    if (!readRegister(0x0004, rx, 2)) return false;

    mv = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getDieTemperature_cC(uint16_t& t) {
    uint8_t rx[2] = {0};

    if (!wake()) return false;
    if (!readRegister(0x0005, rx, 2)) return false;

    t = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getCellCount(uint16_t& count) {
    uint8_t rx[2] = {0};

    if (!wake()) return false;
    if (!readRegister(0x0006, rx, 2)) return false;

    count = (rx[0] << 8) | rx[1];
    return true;
}

bool BQ79631::getFaultFlags(uint16_t& flags) {
    uint8_t rx[2] = {0};

    if (!wake()) return false;
    if (!readRegister(0x0010, rx, 2)) return false;

    flags = (rx[0] << 8) | rx[1];
    return true;
}

} // namespace core::dev