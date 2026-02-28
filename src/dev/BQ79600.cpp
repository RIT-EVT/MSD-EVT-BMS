//
// Created by Key'Mon Jenkins on 2/5/26.
//

#include "dev/BQ79600.hpp"

#define BMS_DEBUG // MUST BE DEFINED IN BMS.CPP


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
    core::time::wait(4);  // 4ms to be safe

    singleWrite(0x00, 0x0309, 0x20); // send wake

    return true;
}

bool BQ79600::init() {

    core::time::wait(10);  // tREADY

    // ============================================================
    // Step 2: Check if device is responding (verify ACTIVE mode)
    // Read DEV_CONF1 - should return 0x14
    // ============================================================
#ifdef BMS_DEBUG
    uart_.puts("Step 2: Checking device presence...\r\n");
#endif

    uint8_t dev_conf = 0;
    bool read_success = singleRead(0x00, REG_DEV_CONF1, dev_conf);

    if (!read_success || dev_conf != 0x14) {
#ifdef BMS_DEBUG
        uart_.printf("Device not responding. DEV_CONF1 = 0x%02X (expect 0x14)\r\n", dev_conf);
        uart_.puts("Trying different SPI modes...\r\n");
#endif

        // Try all 4 SPI modes to find the correct one
        return false;  // Caller should try different SPI modes
    }

#ifdef BMS_DEBUG
    uart_.printf("✓ DEV_CONF1 = 0x%02X - Device in ACTIVE mode!\r\n", dev_conf);
#endif

    // ============================================================
    // Step 3: Configure CONTROL1 to stay in ACTIVE mode
    // bit[5] = SPI_ACTIVE = 1 (keep SPI active)
    // bit[0] = SEND_WAKE = 0 (don't propagate wake yet)
    // ============================================================
#ifdef BMS_DEBUG
    uart_.puts("Step 3: Configuring CONTROL1...\r\n");
#endif

    if (!singleWrite(0x00, REG_CONTROL1, 0x20)) {
#ifdef BMS_DEBUG
        uart_.puts("ERROR: CONTROL1 write failed\r\n");
#endif
        return false;
    }

    core::time::wait(2);

    // Verify CONTROL1 write
    uint8_t ctrl1_verify = 0;
    singleRead(0x00, REG_CONTROL1, ctrl1_verify);

#ifdef BMS_DEBUG
    uart_.printf("  CONTROL1 = 0x%02X (expect 0x20) %s\r\n",
                 ctrl1_verify,
                 ctrl1_verify == 0x20 ? "✓" : "✗");
#endif

    // ============================================================
    // Step 4: Disable communication timeout
    // Prevents auto-sleep due to communication gaps
    // ============================================================
#ifdef BMS_DEBUG
    uart_.puts("Step 4: Disabling communication timeout...\r\n");
#endif

    singleWrite(0x00, 0x0302, 0x02);  // COMM_TIMEOUT[CTL_ACT] = 1
    core::time::wait(1);

    // ============================================================
    // Step 5: Clear any fault flags
    // ============================================================
#ifdef BMS_DEBUG
    uart_.puts("Step 5: Clearing fault flags...\r\n");
#endif

    singleWrite(0x00, 0x030C, 0xFF);  // FAULT_RST1
    singleWrite(0x00, 0x030D, 0xFF);  // FAULT_RST2
    core::time::wait(1);

    // ============================================================
    // Step 6: Verify FAULT_SUMMARY is clear
    // ============================================================
    uint8_t fault_sum = 0;
    singleRead(0x00, 0x030E, fault_sum);

#ifdef BMS_DEBUG
    uart_.printf("  FAULT_SUMMARY = 0x%02X (expect 0x00) %s\r\n",
                 fault_sum,
                 fault_sum == 0x00 ? "✓" : "⚠");
#endif

    // ============================================================
    // Step 7: Read Device ID
    // ============================================================
#ifdef BMS_DEBUG
    uart_.puts("Step 7: Reading Device ID...\r\n");
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

    // Autoaddressing test
    #ifdef BMS_DEBUG
    uart_.puts("\n=== DAISY CHAIN DIAGNOSTIC ===\r\n");
    uart_.puts("Checking if BQ79631 is connected via daisy chain...\r\n");

    // Enable SEND_WAKE
    singleWrite(0x00, 0x0309, 0x21);
    core::time::wait(10);

    // Try stack write (should always succeed)
    stackWrite(0x0343, 0x55);
    uart_.puts("Stack write sent\r\n");
    core::time::wait(5);

    // Try stack read (fails if daisy chain broken)
    uint8_t test = 0;
    if (stackRead(0x00, 0x0343, test)) {
        uart_.printf("Stack read SUCCESS: 0x%02X\r\n", test);
        uart_.puts("→ Daisy chain is working! ✓\r\n");
    } else {
        uart_.puts("Stack read FAILED\r\n");
        uart_.puts("→ Check:\r\n");
        uart_.puts("  1. BQ79631 COMH/COML connections\r\n");
        uart_.puts("  2. BQ79631 power (AVDD, DVDD)\r\n");
        uart_.puts("  3. BQ79600 COMH/COML output enable\r\n");
        return false;
    }
    #endif
#ifdef BMS_DEBUG
    uart_.puts("=== BQ79600 AUTO-ADDRESS START ===\r\n");
#endif


    // CONTROL1 = 0x21: SPI_ACTIVE=1, SEND_WAKE=1
    singleWrite(0x00, REG_CONTROL1, 0x21);
    core::time::wait(10);  // Wait for wake to propagate to stack

    #ifdef BMS_DEBUG
    uart_.puts("Verifying stack is awake...\r\n");

    // Try a simple stack write to see if devices respond
    stackWrite(0x0343, 0xAA);
    core::time::wait(5);

    // Try a stack read
    uint8_t test_val = 0;
    if (stackRead(0x00, 0x0343, test_val)) {
        uart_.printf("Stack device responded with 0x%02X ✓\r\n", test_val);
    } else {
        uart_.puts("WARNING: Stack not responding yet\r\n");
    }
    #endif

    // ------------------------------------------------------------
    // Step 1: Dummy stack writes to OTP_ECC_DATAIN1–8
    // ------------------------------------------------------------
    for (uint8_t i = 0; i < OTP_ECC_COUNT; i++) {
        stackWrite(REG_OTP_ECC_BASE + i, 0x00);
        core::time::wait(2);
    }

    // ------------------------------------------------------------
    // Step 2: Enable auto-addressing
    // CONTROL1[0] = SEND_WAKE = 1
    // ------------------------------------------------------------
    broadcastWrite(REG_CONTROL1, 0x21);

    core::time::wait(5);

    // ------------------------------------------------------------
    // Step 3: Assign addresses via DIR0_ADDR
    // ------------------------------------------------------------
    for (uint8_t addr = 0; addr < expected_devices; addr++) {
        broadcastWrite(REG_DIR0_ADDR, addr);

    #ifdef BMS_DEBUG
            uart_.printf("Assigned stack address %u\r\n", addr);
    #endif

        core::time::wait(2);
    }

    core::time::wait(5);

    // ------------------------------------------------------------
    // Step 4: broadcast write 0x02 to comm_ctrl register
    // ------------------------------------------------------------
    broadcastWrite(REG_COMM_CTRL, 0x02);

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
        // core::time::wait(5);
    }

    core::time::wait(5);

    // ------------------------------------------------------------
    // Step 7: Stack read DIR0_ADDR
    // ------------------------------------------------------------
    uint8_t readback = 0;
    bool verify = stackRead(0x0, REG_DIR0_ADDR, readback);
    if (!verify) {
        #ifdef BMS_DEBUG
            uart_.puts("ERROR: DIR0_ADDR readback failed\r\n");
        #endif
        return false;
    }
    #ifdef BMS_DEBUG
        uart_.printf("Device %u reports address %u\r\n", 0x0, readback);
    #endif

    core::time::wait(5);

    // ------------------------------------------------------------
    // Step 8: Verify base device configuration
    // DEV_CONF1 must be 0x14
    // ------------------------------------------------------------
    uint8_t dev_conf = 0;
    verify = singleRead(0x00, REG_DEV_CONF1, dev_conf);
    if (!verify) {
        #ifdef BMS_DEBUG
            uart_.puts("ERROR: DEV_CONF1 read failed\r\n");
        #endif
         return false;
    }

    #ifdef BMS_DEBUG
        uart_.printf("DEV_CONF1 = 0x%02X (expect 0x14)\r\n", dev_conf);
    #endif

    if (dev_conf != 0x14) {
        #ifdef BMS_DEBUG
            uart_.puts("ERROR: DEV_CONF1 verification failed\r\n");
        #endif
        // return false;
    }

    #ifdef BMS_DEBUG
        uart_.puts("=== AUTO-ADDRESS COMPLETE ===\r\n");
    #endif

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
    if (!readReg16(0x00, 0x306, id_high, false))
        return false;

    if (!readReg16(0x00, 0x307, id_low, false))
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
    uint8_t rx_frame[7] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // required to receive data!!
    uint8_t frame[7];
    uint8_t frame_len;

    if (stack) {
        // Stack read: 6 bytes total
        frame[0] = 0xA0;
        frame[1] = (reg >> 8);
        frame[2] = reg & 0xFF;
        frame[3] = 0x0; // read 12 bytes of data
        uint16_t crc = crc16(frame, 4);
        // frame[4] = (crc >> 8) & 0xFF; // msb first
        // frame[5] = crc & 0xFF;
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


    #ifdef BMS_DEBUG
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

}