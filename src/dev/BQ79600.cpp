//
// Created by Key'Mon Jenkins on 2/5/26

//

#include "dev/BQ79600.hpp"

#define BMS_DEBUG // MUST BE DEFINED IN BMS.CPP
#define MESSAGE_DEBUG

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
constexpr uint16_t REG_FAULT_SUMMARY = 0x030E;
constexpr uint16_t REG_GPIO_CONF1   = 0x0320;
constexpr uint16_t REG_GPIO_CONF2   = 0x0321;
constexpr uint16_t REG_OTP_ECC_BASE = 0x0343;
constexpr uint8_t  OTP_ECC_COUNT    = 8;
}

namespace core::dev {
BQ79600::BQ79600(io::SPI& spi, const uint8_t spi_device, io::UART& uart)
    : spi_(spi), device_(spi_device), uart_(uart){}

/*
 * Read & Write helper functions for the device
 * Broadcast - used to send messages to every device
 * Stack - used to communicate with the entire stack
 * Single - used to communicate with a single device
 *
 * "dev" only used for single read/writes
 **/
void BQ79600::broadcastWrite(uint16_t reg, uint8_t val) const {
    writeReg16(0x0, reg, val, false, true);
}

bool BQ79600::stackRead(uint16_t reg, uint8_t& val) const {
    return readReg16(0x0, reg, val, true);
}

void BQ79600::stackWrite(uint16_t reg, uint8_t val) const {
    writeReg16(0x0, reg, val, true, false);
}

bool BQ79600::singleRead(uint8_t dev, uint16_t reg, uint8_t& val) const {
    return readReg16(dev, reg, val, false);
}

void BQ79600::singleWrite(uint8_t dev, uint16_t reg, uint8_t val) const {
    writeReg16(dev, reg, val, false, false);
}

/* Initialization sequence
 *
 *
 **/
bool BQ79600::init() {

    // verify ACTIVE mode
    // Read DEV_CONF1 - should return 0x14
    #ifdef BMS_DEBUG
        uart_.puts("Checking device presence...\r\n");
    #endif

    uint8_t dev_conf = 0;
    bool read_success = singleRead(0x00, REG_DEV_CONF1, dev_conf);

    if (!read_success || dev_conf != 0x14) {
        #ifdef BMS_DEBUG
            uart_.printf("Device not responding. DEV_CONF1 = 0x%02X (expect 0x14)\r\n", dev_conf);
            uart_.puts("Trying different SPI modes...\r\n");
        #endif
        return false;
    }

    #ifdef BMS_DEBUG
        uart_.printf("✓ DEV_CONF1 = 0x%02X - Device in ACTIVE mode!\r\n", dev_conf);
    #endif

    // Configure CONTROL1 to stay in ACTIVE mode
    // bit[5] = SPI_ACTIVE = 1 (keep SPI active)
    // bit[0] = SEND_WAKE = 0 (don't propagate wake yet)
    #ifdef BMS_DEBUG
        uart_.puts("Configuring CONTROL1...\r\n");
    #endif

    singleWrite(0x00, REG_CONTROL1, 0x20);

    core::time::wait(10);


    // Disable communication timeout
    // Prevents auto-sleep due to communication gaps
    #ifdef BMS_DEBUG
        uart_.puts("Disabling communication timeout...\r\n");
    #endif

    singleWrite(0x00, 0x0302, 0x02);  // COMM_TIMEOUT[CTL_ACT] = 1
    core::time::wait(1);

    // Step 5: Clear any fault flags
    #ifdef BMS_DEBUG
        uart_.puts("Clearing fault flags...\r\n");
    #endif

    singleWrite(0x00, 0x030C, 0xFF);  // FAULT_RST1
    singleWrite(0x00, 0x030D, 0xFF);  // FAULT_RST2
    core::time::wait(1);

    // Step 6: Verify FAULT_SUMMARY is clear
    uint8_t fault_sum = 0;
    singleRead(0x00, 0x030E, fault_sum);

    #ifdef BMS_DEBUG
        uart_.printf("  FAULT_SUMMARY = 0x%02X (expect 0x00) %s\r\n",
                     fault_sum,
                     fault_sum == 0x00 ? "✓" : "⚠");
    #endif

    // Step 7: Read Device ID
    #ifdef BMS_DEBUG
        uart_.puts("Reading Device ID...\r\n");
    #endif

    uint16_t device_id = 0;
    if (readDeviceID(device_id)) {
        #ifdef BMS_DEBUG
            uart_.printf("  Device ID = 0x%04X ✓\r\n", device_id);
        #endif
    } else {
        #ifdef BMS_DEBUG
                uart_.puts("  WARNING: Could not read Device ID\r\n");
        #endif
    }

    #ifdef BMS_DEBUG
        uart_.puts("=== INIT COMPLETE - Device in ACTIVE mode ===\r\n\n");
    #endif

    return true;
}

/*
 * CRC-16-IBM generator polynomial
 * polinomial = 8005, reflected A001
 * LSB first!
 **/
uint16_t BQ79600::crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;   // reflected poly
            else
                crc >>= 1;
        }
    }
    return crc;
}

/*
 * Auto-addressing sequence for the BQ79600
 * steps are carefully commented throughout the function
 **/
void BQ79600::autoAddressStack(uint8_t expected_devices) const {

    #ifdef BMS_DEBUG
        uart_.puts("=== AUTO-ADDRESS START ===\r\n");
    #endif


    // ------------------------------------------------------------
    // Step 1: Dummy stack writes to OTP_ECC_DATAIN1–8
    // ------------------------------------------------------------
    #ifdef BMS_DEBUG
        uart_.puts("Step 1: DLL sync (dummy stack writes)...\r\n");
    #endif
    for (uint8_t i = 0; i < OTP_ECC_COUNT; i++) {
        stackWrite(REG_OTP_ECC_BASE + i, 0x00);
        // core::time::wait(2);
    }

    // ------------------------------------------------------------
    // Step 2: Enable auto-addressing
    // bit[0] = ADDR_WR  = 1 → enables auto-addressing
    // ------------------------------------------------------------
    #ifdef BMS_DEBUG
        uart_.puts("Step 2: Enable auto-addressing...\r\n");
    #endif
    broadcastWrite(REG_CONTROL1, 0x01);
    core::time::wait(2);

    // ------------------------------------------------------------
    // Step 3: Assign addresses via DIR0_ADDR
    // ------------------------------------------------------------
    #ifdef BMS_DEBUG
        uart_.puts("Step 3: Assigning addresses...\r\n");
    #endif

    for (uint8_t addr = 0; addr < expected_devices; addr++) {
        broadcastWrite(REG_DIR0_ADDR, addr);

    #ifdef BMS_DEBUG
        uart_.printf("\tAssigned address %u\r\n", addr);
    #endif

        core::time::wait(2);
    }

    // EXIT auto-address mode
    broadcastWrite(REG_CONTROL1, 0x00);
    core::time::wait(2);

    // core::time::wait(10);

    // ------------------------------------------------------------
    // Step 4: broadcast write 0x02 to comm_ctrl register
    // ------------------------------------------------------------
    #ifdef BMS_DEBUG
        uart_.puts("Step 4: Configure all as stack devices...\r\n");
    #endif
    broadcastWrite(REG_COMM_CTRL, 0x02);

    core::time::wait(2);

    // ------------------------------------------------------------
    // Step 5: Single write 0x3 to comm control register to top device
    // ------------------------------------------------------------
    #ifdef BMS_DEBUG
        uart_.printf("Step 5: Mark device %u as top of stack...\r\n", expected_devices-1);
    #endif
    singleWrite(expected_devices-1, REG_COMM_CTRL, 0x03);

    // core::time::wait(10);

    // ------------------------------------------------------------
    // Step 6: Dummy stack reads (DLL sync)
    // ------------------------------------------------------------
    #ifdef BMS_DEBUG
        uart_.puts("Step 6: DLL sync (dummy stack reads)...\r\n");
    #endif

    for (uint8_t i = 0; i < OTP_ECC_COUNT; i++) {
        uint8_t dummy = 0;
        stackRead(REG_OTP_ECC_BASE + i, dummy);

        core::time::wait(2);
    }

    broadcastWrite(REG_CONTROL1, 0x00); // SPI_ACTIVE
    core::time::wait(2);


    #ifdef BMS_DEBUG
        uart_.puts("=== AUTO-ADDRESS COMPLETE ===\r\n\n");
    #endif

}

/*
 * Register initialization
 **/
bool BQ79600::initRegisters()
{
    #ifdef BMS_DEBUG
        uart_.puts("=== BQ79600 REGISTER INIT ===\r\n");
    #endif

    // ============================================================
    // CONTROL1 (0x0309)
    // bit[5] = SEND_WAKE  = 1 → enable wake propagation to stack
    // bit[0] = ADDR_WR  = 1 → enables auto-addressing
    // ============================================================
    singleWrite(0x00, REG_CONTROL1, 0x20);

    #ifdef BMS_DEBUG
        uart_.printf("CONTROL1 write issued (self clearing)\r\n");
    #endif

    core::time::wait(1);

    // ============================================================
    // COMM_CTRL (0x0308)
    // 0x00 = bridge device role (not stack, not top of stack)
    // ============================================================
    uint8_t verify = 0;
    singleWrite(0x00, REG_COMM_CTRL, 0x00);

    singleRead(0x00, REG_COMM_CTRL, verify);
    #ifdef BMS_DEBUG
        uart_.printf("COMM_CTRL  = 0x%02X (expect 0x00) %s\r\n",
                     verify, verify == 0x00 ? "OK" : "FAIL");
    #endif

    core::time::wait(1);

    // ============================================================
    // FAULT_RST1 (0x030C) and FAULT_RST2 (0x030D)
    // Write 0xFF to clear all faults on startup
    // ============================================================
    singleWrite(0x00, 0x030C, 0xFF);
    singleWrite(0x00, 0x030D, 0xFF);
    core::time::wait(1);

    // ============================================================
    // FAULT_MSK1 (0x031C) and FAULT_MSK2 (0x031D)
    // Mask faults during initialization
    // 0xFF = mask all faults
    // ============================================================
    singleWrite(0x00, 0x031C, 0xFF);
    singleWrite(0x00, 0x031D, 0xFF);
    core::time::wait(1);

    // ============================================================
    // Verify DEV_CONF1 (0x2001)
    // Read-only register, should always be 0x14
    // If wrong, device is not properly configured
    // ============================================================
    uint8_t dev_conf = 0;
    singleRead(0x00, REG_DEV_CONF1, dev_conf);
    #ifdef BMS_DEBUG
        uart_.printf("DEV_CONF1  = 0x%02X (expect 0x14) %s\r\n",
                     dev_conf, dev_conf == 0x14 ? "OK" : "FAIL");
    #endif

    if (dev_conf != 0x14) {
        #ifdef BMS_DEBUG
            uart_.puts("ERROR: DEV_CONF1 check failed - device not ready\r\n");
        #endif
        return false;
    }

    #ifdef BMS_DEBUG
        uart_.puts("=== REGISTER INIT COMPLETE ===\r\n");
    #endif

    return true;
}

/*
 * Returns BQ79600 device ID
 **/
bool BQ79600::readDeviceID(uint16_t& id) {
    uint8_t id_low = 0, id_high = 0;

    // Device ID is at register 0x0500 (2 bytes)
    if (!readReg16(0x00, 0x306, id_high, false))
        return false;

    if (!readReg16(0x00, 0x307, id_low, false))
        return false;

    id = (id_high << 8) | id_low;
    return true;
}

/*
 * 16 BIT SPI write for BQ79600
 * Frame structure:
 *  broadcast - [D0] [REG (HI)] [REG(LO)] [DATA] [CRC LSB] [CRC MSB]
 *  stack - [B0] [REG (HI)] [REG(LO)] [DATA] [CRC LSB] [CRC MSB]
 *  single - [90] [DEV] [REG (HI)] [REG(LO)] [DATA] [CRC LSB] [CRC MSB]
 **/
void BQ79600::writeReg16(uint8_t dev, uint16_t reg, uint8_t val, bool stack, bool broadcast) const {
    uint8_t frame[7];
    uint8_t frame_len;

    if (broadcast) {
        // Broadcast: 7 bytes total
        frame[0] = 0xD4;
        frame[1] = (reg >> 8);
        frame[2] = reg & 0xFF;
        frame[3] = val;
        uint16_t crc = crc16(frame, 4);
        frame[4] = crc & 0xFF; // lsb first
        frame[5] = (crc >> 8) & 0xFF;
        frame_len = 6;
    }
    else if (stack) {
        // Stack write: 6 bytes total
        frame[0] = 0xB0;
        frame[1] = (reg >> 8);
        frame[2] = reg & 0xFF;
        frame[3] = val;
        uint16_t crc = crc16(frame, 4);
        frame[4] = crc & 0xFF; // lsb first
        frame[5] = (crc >> 8) & 0xFF;
        frame_len = 6;
    }
    else {
        // Single device: 7 bytes total
        frame[0] = 0x90;
        frame[1] = dev;
        frame[2] = (reg >> 8);
        frame[3] = reg & 0xFF;
        frame[4] = val;
        uint16_t crc = crc16(frame, 5);
        frame[5] = crc & 0xFF; // lsb first
        frame[6] = (crc >> 8) & 0xFF;
        frame_len = 7;
    }

    // print frame
    #ifdef MESSAGE_DEBUG
    uart_.printf("WRITE TX [%d bytes]: ", frame_len);
    for (int i = 0; i < frame_len; i++) {
        uart_.printf("%02X ", frame[i]);
    }
    uart_.printf("\r\n");
    #endif

    spi_.startTransmission(device_);

    spi_.write(frame, frame_len);

    spi_.endTransmission(device_);
}

/*
 * 16 BIT SPI read for BQ79600
 * Frame structure:
 *  broadcast - [C0] [REG (HI)] [REG(LO)] [LEN] [CRC LSB] [CRC MSB] - unused
 *  stack - [A0] [REG (HI)] [REG(LO)] [LEN] [CRC LSB] [CRC MSB]
 *  single - [80] [DEV] [REG (HI)] [REG(LO)] [LEN] [CRC LSB] [CRC MSB]
 *
 *  Must write to request message read, then clock out bytes while reading
 **/
bool BQ79600::readReg16(uint8_t dev, uint16_t reg, uint8_t& val, bool stack) const {
    uint8_t rx_frame[7] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // required to receive data!!
    uint8_t frame[7];
    uint8_t frame_len;

    if (stack) {
        // Stack read: 6 bytes total
        frame[0] = 0xA0;
        frame[1] = (reg >> 8);
        frame[2] = reg & 0xFF;
        frame[3] = 0x1; // read 1 byte of data
        uint16_t crc = crc16(frame, 4);
        frame[4] = crc & 0xFF; // lsb first
        frame[5] = (crc >> 8) & 0xFF;
        frame_len = 6;
    }
    else {
        // Single read: 7 bytes total
        frame[0] = 0x80;
        frame[1] = dev;
        frame[2] = (reg >> 8);
        frame[3] = reg & 0xFF;
        frame[4] = 0x0;
        uint16_t crc = crc16(frame, 5);
        frame[5] = crc & 0xFF;
        frame[6] = (crc >> 8) & 0xFF;
        frame_len = 7;
    }

    #ifdef MESSAGE_DEBUG
    uart_.printf("READ TX [%d bytes]: ", frame_len);
    for (int i = 0; i < frame_len; i++) {
        uart_.printf("%02X ", frame[i]);
    }
    uart_.printf("\r\n");
    #endif

    // ========================================
    // TRANSACTION 1: Send read command
    // ========================================
    if (!spi_.startTransmission(device_))
        return false;

    if (spi_.write(frame, frame_len) != core::io::SPI::SPIStatus::OK) {
        spi_.endTransmission(device_);
        return false;
    }

    spi_.endTransmission(device_);

    core::time::wait(1);  // Wait 1ms for device to prepare response

    // ========================================
    // TRANSACTION 2: Read response
    // ========================================
    if (!spi_.startTransmission(device_))
        return false;

    // Clock out bytes (sends dummy data, captures MISO)
    core::io::SPI::SPIStatus spiStatus = core::io::SPI::SPIStatus::OK;
    for (int i = 0; i < frame_len; i++) {
        spiStatus = spi_.read(&rx_frame[i]);
    }
    // core::io::SPI::SPIStatus spiStatus = spi_.read(rx_frame, 6);
    if (spiStatus != core::io::SPI::SPIStatus::OK) {
        spi_.endTransmission(device_);
        return false;
    }

    spi_.endTransmission(device_);

    core::time::wait(1);  // 1ms delay

    #ifdef MESSAGE_DEBUG
        uart_.printf("READ RX [%d bytes]: ", frame_len);
        for (int i = 0; i < frame_len; i++) {
            uart_.printf("%02X ", rx_frame[i]);
        }
        uart_.printf("\r\n");
    #endif

    // ============================================================
    // Parse response - skip first byte if it's 0x00
    // Response format: [0x00] [DEV] [REG_H] [REG_L] [DATA] [CRC_L] [CRC_H]
    //                   ^^^^^ extra byte
    // ============================================================
    uint8_t offset = (rx_frame[0] == 0x00) ? 1 : 0;

    uint8_t dev_addr = rx_frame[0 + offset];
    uint16_t reg_addr = (rx_frame[1 + offset] << 8) | rx_frame[2 + offset];
    val = rx_frame[3 + offset];
    uint16_t rx_crc = rx_frame[4 + offset] | (rx_frame[5 + offset] << 8);

    // Calculate CRC over [DEV] [REG_H] [REG_L] [DATA]
    // This is the 4 bytes starting at offset
    uint16_t calc_crc = crc16(&rx_frame[0], 4+offset);

    #ifdef MESSAGE_DEBUG
        uart_.printf("  Dev: 0x%02X (expect 0x%02X) %s\r\n",
                     dev_addr, dev, (dev_addr == dev) ? "OK" : "FAIL");
        uart_.printf("  Reg: 0x%04X (expect 0x%04X) %s\r\n",
                     reg_addr, reg, (reg_addr == reg) ? "OK" : "FAIL");
        uart_.printf("  Val: 0x%02X\r\n", val);
        uart_.printf("  CRC: RX=0x%04X CALC=0x%04X %s\r\n",
                     rx_crc, calc_crc, (calc_crc == rx_crc) ? "OK" : "FAIL");
    #endif

    if (!stack) {
        return (calc_crc == rx_crc) && (dev_addr == dev) && (reg_addr == reg);
    }
    return (calc_crc == rx_crc) && (reg_addr == reg);
}

}