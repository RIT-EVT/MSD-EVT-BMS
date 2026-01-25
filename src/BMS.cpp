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

// helper functions

union FloatBytes {
    float f;
    uint8_t b[4];
};

void writeThermistorCoeffs(uint16_t eeprom_addr, float sh_a, float sh_b, float sh_c) {
    FloatBytes fb;

    fb.f = sh_a;
    eeprom->writeByte(eeprom_addr + 0, fb.b[0]);
    eeprom->writeByte(eeprom_addr + 1, fb.b[1]);
    eeprom->writeByte(eeprom_addr + 2, fb.b[2]);
    eeprom->writeByte(eeprom_addr + 3, fb.b[3]);

    fb.f = sh_b;
    eeprom->writeByte(eeprom_addr + 4, fb.b[0]);
    eeprom->writeByte(eeprom_addr + 5, fb.b[1]);
    eeprom->writeByte(eeprom_addr + 6, fb.b[2]);
    eeprom->writeByte(eeprom_addr + 7, fb.b[3]);

    fb.f = sh_c;
    eeprom->writeByte(eeprom_addr + 8, fb.b[0]);
    eeprom->writeByte(eeprom_addr + 9, fb.b[1]);
    eeprom->writeByte(eeprom_addr + 10, fb.b[2]);
    eeprom->writeByte(eeprom_addr + 11, fb.b[3]);
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

    static BQ34 fuel_gage_inst{i2c};
    static core::dev::M24C32 eeprom_inst{0x50, *i2c};
    fuel_gauge = &fuel_gage_inst;
    eeprom = &eeprom_inst;

    // fuel_gauge = new BQ34(i2c);
    // eeprom = new core::dev::M24C32(0x50, *i2c);

    uart->puts("Detecting BQ34z100...\r\n");

    // detect bq34
    if (uint16_t controlStatus = 0; !fuel_gauge->readWord(CONTROL, controlStatus)) {
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
    } else {
        // default coefficients
        constexpr float DEF_SH_A = 0.001129148f;
        constexpr float DEF_SH_B = 0.000234125f;
        constexpr float DEF_SH_C = 0.0000000876741f;

        for (uint8_t i = 0; i < NUM_THERMISTORS; i++) {
            writeThermistorCoeffs(0x0100 + i*0x10, DEF_SH_A, DEF_SH_B, DEF_SH_C);
        }
    }

    // adc setup for thermistors

    therm_adcs_[0] = &IO::getADC<IO::Pin::PA_0>();
    therm_adcs_[1] = &IO::getADC<IO::Pin::PA_1>();
    // therm_adcs_[2] = &IO::getADC<IO::Pin::PB_4>();
    // therm_adcs_[3] = &IO::getADC<IO::Pin::PB_3>();
    // therm_adcs_[4] = &IO::getADC<IO::Pin::PD_2>();

    static ThermistorArray therm_array{therm_adcs_};
    thermistors_ = &therm_array;

    // Load calibration data

    thermistors_->loadCoefficients(*eeprom, EEPROM_THERM_BASE);
    uart->puts("Thermistors initialized\r\n");

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
    if (!(fuel_gauge->getVoltage(bq34_voltage) &&
      fuel_gauge->getTemperature(bq34_temperature) &&
      fuel_gauge->getCurrent(bq34_current) &&
      fuel_gauge->getSOC(bq34_soc) &&
      fuel_gauge->getSOH(bq34_soh) &&
      fuel_gauge->getVoltageRaw(bq34_voltage_raw) &&
      fuel_gauge->getFlags(bq34_flags))) {
        uart->puts("BQ34 read error (one or more values invalid)\r\n");
      }

    // debug print statements
    int16_t temp_c = bq34_temperature/10 - 273;
    uart->printf("Voltage: %d mV%\r\n", bq34_voltage);
    uart->printf("Temperature: %d C%\r\n", temp_c);
    uart->printf("Current: %d mA%\r\n", bq34_current);
    uart->printf("SOC: %d mA%\r\n", bq34_soc);
    uart->printf("SOH: %d mA%\r\n", bq34_soh);
    uart->printf("Voltage (raw): %d mV%\r\n", bq34_voltage_raw);

    uart->printf("Flags: 0x%X:%\r\n", bq34_flags);
    // Decode common flag bits
    if (bq34_flags & 0x0001) uart->printf("%\t- DSG (Discharging detected)%\r\n");
    if (bq34_flags & 0x0002) uart->printf("%\t- SOCF (SOC Final Threshold)%\r\n");
    if (bq34_flags & 0x0004) uart->printf("%\t- SOC1 (SOC Threshold 1)%\r\n");
    if (bq34_flags & 0x0008) uart->printf("%\t- BAT_DET (Battery Detected)%\r\n");
    if (bq34_flags & 0x0010) uart->printf("%\t- WAIT_ID (Waiting for ID)%\r\n");
    if (bq34_flags & 0x0020) uart->printf("%\t- OCV_TAKEN (OCV Measurement Taken)%\r\n");
    if (bq34_flags & 0x0100) uart->printf("%\t- CHG (Charging Detected)%\r\n");
    if (bq34_flags & 0x0200) uart->printf("%\t- FC (Fully Charged)%\r\n");
    if (bq34_flags & 0x0400) uart->printf("%\t- OTD (Over Temperature Discharge)%\r\n");
    if (bq34_flags & 0x0800) uart->printf("%\t- OTC (Over Temperature Charge)%\r\n");

    // other measurements

    // thermistors
    thermistors_->update();


    for (uint8_t i = 0; i < 2; i++) {
        auto r = thermistors_->getSensor(i);

        uart->printf("TH%d: %d.%d C  Fault=%d\r\n",
                     i,
                     r.temperature_dC / 10,
                     abs(r.temperature_dC % 10),
                     static_cast<uint8_t>(r.fault));
    }

    int16_t avg = thermistors_->getAverage();
    uart->printf("Therm Avg: %d.%d C\r\n",
                 avg / 10,
                 abs(avg % 10));
}

void msd::bms::BmsMaster::update_protection() {
    // protection logic placeholder
    // will handle OV/UV, OT/UT & OSC/SCD
    // interpret flags from BQ34, BQ79631 & BQ79616
    if (state_ == BmsState::FAULT || state_ == BmsState::SHUTDOWN) {
        return;
    }

    if (bq34_flags & BQ34_FAULT_MASK) {
        uart->puts("BMS FAULT condition detected!\r\n");
        state_ = BmsState::FAULT;
        return;
    }

    if (bq34_flags & BQ34_WARN_MASK) {
        uart->puts("BMS WARNING condition detected!\r\n");
        state_ = BmsState::WARNING;
        return;
    }

    // no flags, normal
    if (state_ != BmsState::NORMAL) {
        uart->puts("BMS conditions normal!\r\n");
    }
    state_ = BmsState::NORMAL;
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


