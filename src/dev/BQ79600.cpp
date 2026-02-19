//
// Created by Key'Mon Jenkins on 2/5/26.
//

#include "dev/BQ79600.hpp"

#define BMS_DEBUG


namespace core::dev {
core::dev::BQ79600::BQ79600(core::io::SPI& spi, uint8_t spi_device, core::io::UART& uart)
    : spi_(spi), device_(spi_device), uart_(uart){}

bool BQ79600::broadcastWrite(uint16_t reg, uint8_t val)
{
    return writeReg16(0x0, reg, val, true, true);
}

bool BQ79600::stackRead(uint8_t dev, uint16_t reg, uint8_t& val)
{
    return readReg16(dev, reg, val, true); // need to write a stack read function
}

bool BQ79600::stackWrite(uint16_t reg, uint8_t val)
{
    return writeReg16(0x0, reg, val, true, false); // need to make a stack write function
}

bool BQ79600::singleWrite(uint8_t dev, uint16_t reg, uint8_t val)
{
    return writeReg16(dev, reg, val, false, false);
}

bool BQ79600::singleRead(uint8_t dev, uint16_t reg, uint8_t& val)
{
    return readReg16(dev, reg, val, false);
}

bool BQ79600::wake() {
    // CRITICAL: The wake pulse must be 2.5-3.0ms with CS HIGH
    // This is different from what you have in BMS.cpp
    for (int i=0; i<2; i++){
        if (!spi_.startTransmission(device_))
            return false;

        // Send enough 0x00 bytes to create 2.8ms pulse at 1MHz SPI
        // 1MHz = 1us per bit, 8 bits per byte = 8us per byte
        // 2800us / 8us = 350 bytes
        for (int i = 0; i < 385; i++) {
            spi_.write(0x00);
        }
        // then pull mosi back high for 2us
        spi_.write(0xFF);

        spi_.endTransmission(device_);
    }

    // Wait for device to initialize (tSU(WAKE_SHUT) = 2-3.5ms)
    // core::time::wait(4);  // 4ms to be safe

    singleWrite(0x00, 0x0309, 0x20); // send wake

    return true;
}


uint16_t BQ79600::crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;

    // msb first
    // for (uint8_t i = 0; i < len; i++) {
    //     crc ^= static_cast<uint16_t>(data[i]) << 8;
    //     for (uint8_t b = 0; b < 8; b++) {
    //         if (crc & 0x8000)
    //             crc = (crc << 1) ^ 0x8005; // or 0x8005
    //         else
    //             crc = crc << 1;
    //     }
    // }

    // lsb first
        for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;   // reflected poly
            else
                crc >>= 1;
        }
    }
    // return (crc << 8) | (crc >> 8);
    return crc;

}

bool BQ79600::autoAddressStack(uint8_t expected_devices)
{
#ifdef BMS_DEBUG
    uart_.puts("=== BQ79600 AUTO-ADDRESS START ===\r\n");
#endif

    // ------------------------------------------------------------
    // Step 1: Dummy stack writes to OTP_ECC_DATAIN1–8
    // ------------------------------------------------------------
    for (uint8_t i = 0; i < OTP_ECC_COUNT; i++) {
        stackWrite(REG_OTP_ECC_BASE + i, 0x00);
        core::time::wait(1);
    }

    // ------------------------------------------------------------
    // Step 2: Enable auto-addressing
    // CONTROL1[0] = SEND_WAKE = 1
    // ------------------------------------------------------------
    broadcastWrite(REG_CONTROL1, 0x01);

    core::time::wait(2);

    // ------------------------------------------------------------
    // Step 3: Assign addresses via DIR0_ADDR
    // ------------------------------------------------------------
    for (uint8_t addr = 0; addr < expected_devices; addr++) {
        broadcastWrite(REG_DIR0_ADDR, addr);

    #ifdef BMS_DEBUG
            uart_.printf("Assigned stack address %u\r\n", addr);
    #endif

        core::time::wait(1);
    }

    // ------------------------------------------------------------
    // Step 4: Slack write 0x02 to comm_ctrl register
    // ------------------------------------------------------------
    stackWrite(REG_COMM_CTRL, 0x02);

    core::time::wait(2);

    // ------------------------------------------------------------
    // Step 5: Single write 0x3 to comm control register to top device
    // ------------------------------------------------------------
    singleWrite(expected_devices - 1, REG_COMM_CTRL, 0x03);
    core::time::wait(2);

    // ------------------------------------------------------------
    // Step 6: Dummy stack reads (DLL sync)
    // ------------------------------------------------------------
    uint8_t dummy = 0x01;
    for (uint8_t i = 0; i < OTP_ECC_COUNT; i++) {
        stackRead(0x00, REG_OTP_ECC_BASE + i, dummy);
        core::time::wait(1);
    }

    // ------------------------------------------------------------
    // Step 7: Stack read DIR0_ADDR
    // ------------------------------------------------------------
    uint8_t readback = 0;
    stackRead(0x0, REG_DIR0_ADDR, readback);
    // if (!stackRead(0x0, REG_DIR0_ADDR, readback)) {
    //     #ifdef BMS_DEBUG
    //         uart_.puts("ERROR: DIR0_ADDR readback failed\r\n");
    //     #endif
    //     // return false;
    //
    //     #ifdef BMS_DEBUG
    //             uart_.printf("Device %u reports address %u\r\n", 0x0, readback);
    //     #endif
    // }

    // ------------------------------------------------------------
    // Step 8: Verify base device configuration
    // DEV_CONF1 must be 0x14
    // ------------------------------------------------------------
    uint8_t dev_conf = 0;
    singleRead(0x00, REG_DEV_CONF1, dev_conf);
//     if (!singleRead(0x00, REG_DEV_CONF1, dev_conf)) {
// #ifdef BMS_DEBUG
//         uart_.puts("ERROR: DEV_CONF1 read failed\r\n");
// #endif
//         // return false;
//     }

// #ifdef BMS_DEBUG
//     uart_.printf("DEV_CONF1 = 0x%02X (expect 0x14)\r\n", dev_conf);
// #endif
//
//     if (dev_conf != 0x14) {
// #ifdef BMS_DEBUG
//         uart_.puts("ERROR: DEV_CONF1 verification failed\r\n");
// #endif
//         // return false;
//     }
//
// #ifdef BMS_DEBUG
//     uart_.puts("=== AUTO-ADDRESS COMPLETE ===\r\n");
// #endif

    return true;
}

bool BQ79600::initRegisters()
{
#ifdef BMS_DEBUG
    uart_.puts("=== BQ79600 REGISTER INIT ===\r\n");
#endif

    // ============================================================
    // CONTROL1 (0x0309)
    // bit[5] = SPI_ACTIVE = 1 → keep SPI active between frames
    // bit[0] = SEND_WAKE  = 1 → enable wake propagation to stack
    // ============================================================
    if (!singleWrite(0x00, REG_CONTROL1, 0x21)) {
#ifdef BMS_DEBUG
        uart_.puts("ERROR: CONTROL1 write failed\r\n");
#endif
        return false;
    }

    uint8_t verify = 0;
    singleRead(0x00, REG_CONTROL1, verify);
#ifdef BMS_DEBUG
    uart_.printf("CONTROL1   = 0x%02X (expect 0x21) %s\r\n",
                 verify, verify == 0x21 ? "OK" : "FAIL");
#endif

    core::time::wait(1);

    // ============================================================
    // COMM_CTRL (0x0308)
    // 0x00 = bridge device role (not stack, not top of stack)
    // ============================================================
    if (!singleWrite(0x00, REG_COMM_CTRL, 0x00)) {
#ifdef BMS_DEBUG
        uart_.puts("ERROR: COMM_CTRL write failed\r\n");
#endif
        return false;
    }

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

bool BQ79600::readDeviceID(uint16_t& id) {
    uint8_t id_low = 0, id_high = 0;

    // Device ID is at register 0x0500 (2 bytes)
    if (!readReg16(0x00, 0x0500, id_high, false))
        return false;

    if (!readReg16(0x00, 0x0501, id_low, false))
        return false;

    id = (id_high << 8) | id_low;
    return true;
}

bool BQ79600::writeReg16(uint8_t dev, uint16_t reg, uint8_t val, bool stack, bool broadcast)
{
    uint8_t frame[7];
    uint8_t frame_len;

    if (broadcast) {
        // Broadcast: 7 bytes total
        frame[0] = 0xD0;
        frame[1] = (reg >> 8);
        frame[2] = reg & 0xFF;
        frame[3] = val;
        uint16_t crc = crc16(frame, 4);
        // frame[4] = (crc >> 8) & 0xFF; //msb first
        // frame[5] = crc & 0xFF;
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
        // frame[4] = (crc >> 8) & 0xFF;
        // frame[5] = crc & 0xFF;
        frame[4] = crc & 0xFF;
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
        // frame[5] = (crc >> 8) & 0xFF; // msb first
        // frame[6] = crc & 0xFF;
        frame[5] = crc & 0xFF; // lsb first
        frame[6] = (crc >> 8) & 0xFF;
        frame_len = 7;
    }

    // print frame
    #ifdef BMS_DEBUG
    uart_.printf("WRITE TX [%d bytes]: ", frame_len);
    for (int i = 0; i < frame_len; i++) {
        uart_.printf("%02X ", frame[i]);
    }
    uart_.printf("\r\n");
    #endif

    if (!spi_.startTransmission(device_))
        return false;

    bool success = (spi_.write(frame, frame_len) == core::io::SPI::SPIStatus::OK);

    spi_.endTransmission(device_);

    // spi_.writeReg(dev, reg, frame, frame_len);

    // core::time::wait(1);

    return success;
}

bool BQ79600::readReg16(uint8_t dev, uint16_t reg, uint8_t& val, bool stack)
{
    uint8_t rx_frame[6];
    uint8_t frame[7];
    uint8_t frame_len;

    if (stack) {
        // Stack write: 6 bytes total
        frame[0] = 0xA0;
        frame[1] = (reg >> 8);
        frame[2] = reg & 0xFF;
        frame[3] = 0x0;
        uint16_t crc = crc16(frame, 4);
        // frame[4] = (crc >> 8) & 0xFF; // msb first
        // frame[5] = crc & 0xFF;
        frame[4] = crc & 0xFF; // lsb first
        frame[5] = (crc >> 8) & 0xFF;
        frame_len = 6;
    }
    else {
        // Single device: 7 bytes total
        frame[0] = 0x80;
        frame[1] = dev;
        frame[2] = (reg >> 8);
        frame[3] = reg & 0xFF;
        frame[4] = 0x0;
        uint16_t crc = crc16(frame, 5);
        // frame[5] = (crc >> 8) & 0xFF;
        // frame[6] = crc & 0xFF;
        frame[5] = crc & 0xFF;
        frame[6] = (crc >> 8) & 0xFF;
        frame_len = 7;
    }

    #ifdef BMS_DEBUG
    uart_.printf("READ TX [%d bytes]: ", frame_len);
    for (int i = 0; i < frame_len; i++) {
        uart_.printf("%02X ", frame[i]);
    }
    uart_.printf("\r\n");
    #endif

    // ========================================
    // TRANSACTION 1: Send read command
    // ========================================
    // spi_.writeReg(dev, reg, frame, frame_len);

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
    // spi_.readReg(dev, reg, rx_frame, frame_len);

    if (!spi_.startTransmission(device_))
        return false;

    // Clock out 6 bytes (sends dummy data, captures MISO)
    core::io::SPI::SPIStatus spiStatus = spi_.read(rx_frame, 6);
    if (spiStatus != core::io::SPI::SPIStatus::OK) {
        spi_.endTransmission(device_);
        return false;
    }

        core::time::wait(1);


    spi_.endTransmission(device_);

    // core::time::wait(1);  // 1ms delay


    #ifdef BMS_DEBUG
    uart_.printf("READ RX [%d bytes]: ", frame_len);
    for (int i = 0; i < frame_len; i++) {
        uart_.printf("%02X ", rx_frame[i]);
    }
    uart_.printf("\r\n");
    #endif

    // Parse response frame
    uint8_t dev_addr = rx_frame[0];
    uint16_t reg_addr = (rx_frame[1] << 8) | rx_frame[2];
    val = rx_frame[3];
    uint16_t rx_crc =  rx_frame[4] | (rx_frame[5] << 8);

    // Calculate CRC over [DEV ADR][REG ADR_H][REG ADR_L][DATA]
    uint16_t calc_crc = crc16(rx_frame, 4);

    #ifdef BMS_DEBUG
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


// tests
// Add this to BQ79600.cpp
//
// void BQ79600::runSPITests() {
//     uart_.puts("\n========================================\n");
//     uart_.puts("    BQ79600 SPI DIAGNOSTIC TESTS\n");
//     uart_.puts("========================================\n\n");
//
//     // Test 1: Basic SPI functionality
//     uart_.puts("TEST 1: Basic SPI Write\n");
//     uart_.puts("----------------------------------------\n");
//     testBasicSPI();
//     core::time::wait(100);
//
//     // Test 2: Simple read attempt
//     uart_.puts("\nTEST 2: Simple Read Test\n");
//     uart_.puts("----------------------------------------\n");
//     testReadWrite();
//     core::time::wait(100);
//
//     // Test 3: Wake sequence
//     uart_.puts("\nTEST 3: Wake Sequence\n");
//     uart_.puts("----------------------------------------\n");
//     testWakeSequence();
//     core::time::wait(100);
//
//     // Test 4: Loopback (if you can connect MOSI to MISO)
//     uart_.puts("\nTEST 4: Loopback Test\n");
//     uart_.puts("----------------------------------------\n");
//     uart_.puts("Connect MOSI (PC12) to MISO (PC11) for this test\n");
//     uart_.puts("Press any key when ready...\n");
//     core::time::wait(2000);
//     loopbackTest();
//
//     uart_.puts("\n========================================\n");
//     uart_.puts("    TESTS COMPLETE\n");
//     uart_.puts("========================================\n\n");
// }
//
// bool BQ79600::testBasicSPI() {
//     uart_.puts("Sending simple byte pattern...\n");
//     uart_.puts("Watch: CS, CLK, MOSI on scope\n\n");
//
//     uint8_t test_pattern[] = {0x00, 0xFF, 0xAA, 0x55, 0x0F, 0xF0};
//
//     if (!spi_.startTransmission(device_)) {
//         uart_.puts("ERROR: startTransmission failed\n");
//         return false;
//     }
//
//     uart_.puts("TX: ");
//     for (uint8_t byte : test_pattern) {
//         uart_.printf("%02X ", byte);
//         if (spi_.write(byte) != core::io::SPI::SPIStatus::OK) {
//             uart_.puts("\nERROR: SPI write failed\n");
//             spi_.endTransmission(device_);
//             return false;
//         }
//         core::time::wait(10); // Slow down for scope viewing
//     }
//     uart_.puts("\n");
//
//     spi_.endTransmission(device_);
//     uart_.puts("SUCCESS: Basic write complete\n");
//     return true;
// }

// bool BQ79600::testReadWrite() {
//     uart_.puts("Testing SPI read (MISO should respond)...\n");
//     uart_.puts("Watch: MISO line should go LOW during reads\n\n");
//
//     // Send a simple write command first
//     uint8_t write_cmd[] = {0x90, 0x00, 0x03, 0x09, 0x00};
//     uint16_t crc = crc16(write_cmd, 5);
//
//     uart_.puts("WRITE PHASE:\n");
//     uart_.printf("TX: 90 00 03 09 00 %02X %02X\n", crc & 0xFF, (crc >> 8) & 0xFF);
//
//     if (!spi_.startTransmission(device_))
//         return false;
//
//     spi_.write(write_cmd, 5);
//     spi_.write(crc & 0xFF);
//     spi_.write((crc >> 8) & 0xFF);
//
//     spi_.endTransmission(device_);
//     core::time::wait(2);
//
//     // Now try to read
//     uart_.puts("\nREAD PHASE:\n");
//     uart_.puts("Sending read command...\n");
//
//     uint8_t read_cmd[] = {0x80, 0x00, 0x03, 0x09, 0x00};
//     crc = crc16(read_cmd, 5);
//
//     uart_.printf("TX: 80 00 03 09 00 %02X %02X\n", crc & 0xFF, (crc >> 8) & 0xFF);
//
//     if (!spi_.startTransmission(device_))
//         return false;
//
//     spi_.write(read_cmd, 5);
//     spi_.write(crc & 0xFF);
//     spi_.write((crc >> 8) & 0xFF);
//
//     spi_.endTransmission(device_);
//     core::time::wait(2);
//
//     // Read response
//     uart_.puts("Reading response...\n");
//     uart_.puts(">>> WATCH MISO NOW <<<\n");
//
//     uint8_t response[6];
//
//     if (!spi_.startTransmission(device_))
//         return false;
//
//     for (int i = 0; i < 6; i++) {
//         if (spi_.read(&response[i]) != core::io::SPI::SPIStatus::OK) {
//             uart_.puts("ERROR: Read failed\n");
//             spi_.endTransmission(device_);
//             return false;
//         }
//     }
//
//     spi_.endTransmission(device_);
//
//     uart_.printf("RX: ");
//     for (int i = 0; i < 6; i++) {
//         uart_.printf("%02X ", response[i]);
//     }
//     uart_.puts("\n");
//
//     if (response[0] == 0xFF && response[1] == 0xFF) {
//         uart_.puts("WARNING: All 0xFF - MISO stuck high or device not responding\n");
//         return false;
//     }
//
//     uart_.puts("SUCCESS: Got response\n");
//     return true;
// }
//
// bool BQ79600::testWakeSequence() {
//     uart_.puts("Sending wake pulse...\n");
//     uart_.puts("Watch: CS should be LOW, MOSI should be LOW for ~2.8ms\n\n");
//
//     if (!spi_.startTransmission(device_)) {
//         uart_.puts("ERROR: startTransmission failed\n");
//         return false;
//     }
//
//     uart_.puts("Sending 350 x 0x00 bytes at ~1MHz...\n");
//
//     for (int i = 0; i < 350; i++) {
//         spi_.write(0x00);
//     }
//
//     spi_.endTransmission(device_);
//     uart_.puts("Wake pulse complete\n");
//     uart_.puts("Waiting 4ms for device wake...\n");
//     core::time::wait(4);
//
//     uart_.puts("SUCCESS: Wake sequence complete\n");
//     return true;
// }
//
// void BQ79600::loopbackTest() {
//     uart_.puts("Running loopback test (MOSI -> MISO)...\n");
//     uart_.puts("This verifies SPI read functionality\n\n");
//
//     uint8_t test_bytes[] = {0x00, 0xFF, 0xAA, 0x55, 0x0F, 0xF0, 0x12, 0x34};
//     bool all_passed = true;
//
//     if (!spi_.startTransmission(device_)) {
//         uart_.puts("ERROR: startTransmission failed\n");
//         return;
//     }
//
//     for (uint8_t tx_byte : test_bytes) {
//         uint8_t rx_byte = 0;
//
//         // SPI is full-duplex, so we write and read simultaneously
//         // Write the byte
//         if (spi_.write(tx_byte) != core::io::SPI::SPIStatus::OK) {
//             uart_.puts("ERROR: Write failed\n");
//             all_passed = false;
//             break;
//         }
//
//         // Now read what we should have received
//         // (In a true loopback, we'd see the same byte we sent)
//         // But since write() already clears RX, we need to do another transaction
//     }
//
//     spi_.endTransmission(device_);
//
//     // Alternative loopback: Write then read
//     core::time::wait(10);
//
//     uart_.puts("\nAlternative loopback method:\n");
//
//     for (uint8_t tx_byte : test_bytes) {
//         if (!spi_.startTransmission(device_))
//             break;
//
//         // Use transfer-style operation
//         spi_.write(tx_byte);
//
//         uint8_t rx_byte = 0;
//         spi_.read(&rx_byte);
//
//         spi_.endTransmission(device_);
//
//         uart_.printf("TX: 0x%02X  RX: 0x%02X  ", tx_byte, rx_byte);
//
//         if (rx_byte == tx_byte) {
//             uart_.puts("✓ PASS\n");
//         } else {
//             uart_.puts("✗ FAIL\n");
//             all_passed = false;
//         }
//
//         core::time::wait(10);
//     }
//
//     if (all_passed) {
//         uart_.puts("\nSUCCESS: All loopback tests passed\n");
//         uart_.puts("SPI read/write is working correctly\n");
//     } else {
//         uart_.puts("\nFAILURE: Some loopback tests failed\n");
//         uart_.puts("Check MISO configuration\n");
//     }
// }

// Continuous pattern generator for scope triggering
// void BQ79600::scopeTriggerPattern() {
//     uart_.puts("\n========================================\n");
//     uart_.puts("  SCOPE TRIGGER PATTERN (Infinite Loop)\n");
//     uart_.puts("========================================\n");
//     uart_.puts("Sending repeating pattern for scope triggering\n");
//     uart_.puts("Pattern: 0xAA 0x55 0xAA 0x55 ... (100ms period)\n");
//     uart_.puts("Press RESET to stop\n\n");
//
//     while (true) {
//         if (!spi_.startTransmission(device_))
//             continue;
//
//         // Distinctive pattern for scope triggering
//         spi_.write(0xAA);
//         spi_.write(0x55);
//         spi_.write(0xAA);
//         spi_.write(0x55);
//
//         spi_.endTransmission(device_);
//
//         core::time::wait(100); // 100ms between bursts
//     }
// }

}