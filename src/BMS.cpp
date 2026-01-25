/**
 * @file BMS.cpp
 * @brief Interface for the Battery Management System (BMS) master control.
 *
 * This source implements the functions used for managing the BMS.
 * The BMS master is responsible for coordinating communications, safety checks,
 * and fault handling in the Battery Management System.
 *
 * Project: MSD_BMS
 * Author: Key'Mon Jenkins
 * Date: October 2025
 */

#include "BMS.hpp"
#include "core/dev/storage/M24C32.hpp"
#include "core/io/UART.hpp"
#include "dev/BQ34.hpp"
// #include "core/io/I2C.hpp"

namespace IO = core::io;
namespace {
IO::UART* uart = nullptr;
IO::I2C* i2c = nullptr;

BQ34* fuel_gauge = nullptr;
core::dev::M24C32* eeprom = nullptr;
}

msd::bms::BmsMaster& msd::bms::BmsMaster::instance() {
    static msd::bms::BmsMaster instance;
    return instance;
}


/* Initialization of the BMS master. */
void msd::bms::BmsMaster::init() {
    if (initialized_) {
        return;
    }
    // Initialize communication interfaces (e.g., I2C, UART)
    // Setup safety monitoring systems
    // Initialize fault handling mechanisms
    uart = &IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600); // uart init
    i2c = &IO::getI2C<IO::Pin::I2C_SCL, IO::Pin::I2C_SDA>(); // i2c init

    uart->puts("BMS init start\r\n");

    static BQ34 fuel_gage_inst{i2c, uart};
    static core::dev::M24C32 eeprom_inst{0x50, *i2c};
    fuel_gauge = &fuel_gage_inst;
    eeprom = &eeprom_inst;

    // fuel_gauge = new BQ34(i2c);
    // eeprom = new core::dev::M24C32(0x50, *i2c);

    uart->puts("Detecting BQ34z100...\r\n");
    uint16_t controlStatus = 0;

    // detect bq34
    if (!fuel_gauge->readWord(CONTROL, controlStatus)) {
        uart->puts("ERROR: BQ34Z100 not detected!\r\n");
        return; //go to error state
    }
    uart->puts("BQ34Z100 detected!\r\n");

    // detect m24c32
    constexpr uint32_t test = 0xBEEF;
    constexpr uint16_t EEPROM_TEST_ADDR = 0x0000;
    eeprom->writeWord(EEPROM_TEST_ADDR, test);
    if (eeprom->readWord(EEPROM_TEST_ADDR) != test) {
        uart->puts("ERROR: EEPROM failed integrity check\r\n");
    }
    uart->puts("EEPROM OK\r\n");

    initialized_ = true;
    uart->puts("BMS Master initialized!\r\n");
}



/**
 * @brief Update routine for the BMS master.
 *
 * This function should be called periodically to monitor system status,
 * process communications, and handle safety events.
 */
void msd::bms::BmsMaster::update() {
    // Monitor battery parameters (voltage, current, temperature)
    // Process incoming messages and commands
    // Check for safety violations and trigger fault handling if necessary
    if (!initialized_) {
        return;
    }
    update_measurements();
    update_protection();
    update_state_machine();
}

void msd::bms::BmsMaster::update_measurements() {
    // measurement updates bq34
    if (!(fuel_gauge->getVoltage(voltage) &&
      fuel_gauge->getTemperature(temperature) &&
      fuel_gauge->getCurrent(current) &&
      fuel_gauge->getSOC(soc) &&
      fuel_gauge->getSOH(soh))) {
        uart->puts("BQ34 read error (one or more values invalid)\r\n");
        return;
      }


    if (fuel_gauge->getFlags(flags)) {
        uart->printf("flags: 0x%X\r\n", flags);
    }
    // raw voltage may be unused
    if (fuel_gauge->getVoltageRaw(voltage_raw)) {
        uart->printf("voltage_raw: 0x%X\r\n", voltage_raw);
    }
}

void msd::bms::BmsMaster::update_protection() {
    // protection logic placeholder
    // will handle OV/UV, OT/UT & OSC/SCD
    // interpret flags from BQ34, BQ79631 & BQ79616
    if (flags != 0) {
        state_ = BmsState::WARNING;
    }
}

void msd::bms::BmsMaster::update_state_machine() {
    switch (state_) {
        case BmsState::INIT:
            state_ = BmsState::NORMAL;
            break;
        case BmsState::NORMAL:
            // stay here unless issue happens
            break;
        case BmsState::WARNING:
            uart->puts("BMS WARNING STATE!\r\n");
            break;
        case BmsState::FAULT:
            uart->puts("BMS FAULT STATE!\r\n");
            break;
        case BmsState::SHUTDOWN:
            shutdown();
            break;
    }
}


/**
 * @brief Shutdown the BMS master.
 *
 * Safely de-initializes the BMS master and performs any necessary cleanup.
 */
void msd::bms::BmsMaster::shutdown() {
    // Safely shutdown communication interfaces
    // Perform any necessary cleanup
    if (!initialized_) {
        return;
    }

    uart->puts("BMS Master Shutdown\r\n");

    initialized_ = false;
}

