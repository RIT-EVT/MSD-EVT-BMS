/**
 * @file BMS.cpp
 * @brief Interface for the Battery Management System (BMS) master control.
 *
 * This source implements the functions used for managing the BMS.
 * The BMS master is responsible for coordinating communications, safety checks,
 * and fault handling in the Battery Management System.
 *
 * currently have a ton of uart print statements that slow down processing
 * will need to eliminate them in the future!
 *
 * Project: MSD_BMS
 * Author: Key'Mon Jenkins
 * Date: October 2025
 */

#include "BMS.hpp"
#define BMS_DEBUG

namespace IO = core::io;
namespace DEV = core::dev;

// comms devices
namespace {
IO::UART* uart = nullptr;
IO::I2C* i2c = nullptr;
core::io::GPIO& hv_cs_gpio = core::io::getGPIO<core::io::Pin::PA_15>();
core::io::GPIO* spi_cs_pins[] = { &hv_cs_gpio };
IO::SPI* spi = nullptr;
}

// Other devices
namespace {
BQ34* fuel_gauge = nullptr;
DEV::M24C32* eeprom = nullptr;
DEV::BQ79631* hv_monitor = nullptr;
}

// LEDs & GPIO
namespace {
core::io::GPIO& status_gpio = core::io::getGPIO<core::io::Pin::PA_5>();
DEV::LED status_led(status_gpio, DEV::LED::ActiveState::LOW);
core::io::GPIO& warning_gpio = core::io::getGPIO<core::io::Pin::PA_6>();
DEV::LED warning_led(warning_gpio, DEV::LED::ActiveState::LOW);
core::io::GPIO& error_gpio = core::io::getGPIO<core::io::Pin::PA_7>();
DEV::LED error_led(error_gpio, DEV::LED::ActiveState::LOW);
core::io::GPIO& extra_gpio = core::io::getGPIO<core::io::Pin::PB_0>();
DEV::LED extra_led(extra_gpio, DEV::LED::ActiveState::LOW);
}

// BMS instance
msd::bms::BmsMaster& msd::bms::BmsMaster::instance() {
    static msd::bms::BmsMaster instance;
    return instance;
}

/* Initialization of the BMS master. */
void msd::bms::BmsMaster::init() {
    if (initialized_) {
        return;
    }

    // ALL leds are set high upon initialization (POST test)
    // each led turns off as post completes
    status_led.setState(core::io::GPIO::State::HIGH);
    warning_led.setState(core::io::GPIO::State::HIGH);
    error_led.setState(core::io::GPIO::State::HIGH);
    extra_led.setState(core::io::GPIO::State::HIGH);


                       // POST TEST //
    // Initialize communication interfaces (e.g., I2C, UART)
    // Setup safety monitoring systems
    // Initialize fault handling mechanisms


    /**
     * COMMUNICATION DEVICE INITIALIZATION
     * uart, i2c spi
     * need to add success/failure verification (shouldn't fail though)
    */

    uart = &IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600); // uart init
    i2c = &IO::getI2C<IO::Pin::I2C_SCL, IO::Pin::I2C_SDA>(); // i2c init
    spi = &IO::getSPI<IO::Pin::PA_10, IO::Pin::PA_12, IO::Pin::PA_11>(spi_cs_pins, 1); // spi init

    uart->puts("              BMS init start           \r\n");
    uart->puts("---------------------------------------\r\n\r\n");

    uart->puts("Starting initializations!\r\n\r\n");


    /**
     * ON-BOARD DEVICE INITILIZATION
     * BQ34, M24C32, BQ79631
     * need to add success/failure verification (shouldn't fail though)
     */
    static BQ34 fuel_gage_inst{i2c};
    static DEV::M24C32 eeprom_inst{0x50, *i2c};
    static DEV::BQ79631 hv_monitor_inst{*spi};
    fuel_gauge = &fuel_gage_inst;
    hv_monitor = &hv_monitor_inst;
    eeprom = &eeprom_inst;

    /**
     * SLAVE DEVICE INITILIZATION
     *
     * need to add success/failure verification (shouldn't fail though)
     */
    // init goes here :)

    /**
     * SENSOR SETUP
     * thermistors
     * need to add success/failure verification (shouldn't fail though)
     */

    // Thermistors

    // low voltage thermistors
    therm_adcs_[0] = &IO::getADC<IO::Pin::PA_0>();
    therm_adcs_[1] = &IO::getADC<IO::Pin::PA_1>();

    // high voltage thermistors on HVM
    // GPIO8 - GPIO4
    // therm_adcs_[2] = &IO::getADC<IO::Pin::PB_4>();
    // therm_adcs_[3] = &IO::getADC<IO::Pin::PB_3>();
    // therm_adcs_[4] = &IO::getADC<IO::Pin::PD_2>();

    static ThermistorArray therm_array{therm_adcs_};
    thermistors_ = &therm_array;

    extra_led.setState(core::io::GPIO::State::LOW); // init stage passes

    #ifdef BMS_DEBUG
        uart->puts("All devices initialized!\r\n");
        uart->puts("---------------------------------------\r\n\r\n");
    #endif


    /**
     * DEVICE HANDSHAKE
     * BQ34Z100 (Fuel Gage), BQ79631 (HVM), BQ79161 (slave), M24C32 (EEPROM)
     */
    // uart->puts("Detecting High Voltage Monitor...\r\n");
    //
    // // wake and detect bq79634
    // hv_monitor->wake();
    // core::time::wait(2);
    // uint16_t id;
    // hv_monitor->readDeviceID(id);
    // uart->printf("High Voltage Monitor ID: 0x%04X\r\n", id);
    // // state_ = BmsState::FAULT;
    //
    // uart->puts("High Voltage Monitor detected!\r\n");
    // error_led.setState(core::io::GPIO::State::LOW);

    // detect bq34
    if (uint16_t controlStatus = 0; !fuel_gauge->readWord(CONTROL, controlStatus)) {
        uart->puts("ERROR: BQ34Z100 not detected!\r\n");
        state_ = BmsState::FAULT;
        return; //go to error state
    }
    #ifdef BMS_DEBUG
        uart->puts("BQ34Z100 detected!\r\n");
    #endif

    // detect m24c32
    #ifdef BMS_DEBUG
        uart->puts("Checking EEPROM integrity...\r\n");
    #endif
    constexpr uint32_t test = 0xBEEF;
    constexpr uint16_t EEPROM_TEST_ADDR = 0x0000;
    eeprom->writeWord(EEPROM_TEST_ADDR, test);
    if (eeprom->readWord(EEPROM_TEST_ADDR) != test) {
        uart->puts("ERROR: EEPROM failed integrity check\r\n");
        state_ = BmsState::FAULT;
        return; // go to error state
    }
    #ifdef BMS_DEBUG
        uart->puts("EEPROM passed integrity check!\r\n");
    #endif
    error_led.setState(core::io::GPIO::State::LOW); // device detection complete


    // add any device configurations here
    status_led.setState(core::io::GPIO::State::LOW);
    warning_led.setState(core::io::GPIO::State::LOW); // device configuration complete

    initialized_ = true;
    core::time::wait(200);
    #ifdef BMS_DEBUG
        uart->puts("BMS Master initialized!\r\n\r\n");
    #endif
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
    status_led.setState(core::io::GPIO::State::HIGH);
    update_measurements();
    update_protection();
    update_state_machine();
    status_led.setState(core::io::GPIO::State::LOW);
}

void msd::bms::BmsMaster::update_measurements() {
    // measurement updates bq34
    if (!(fuel_gauge->getVoltage(bq34_voltage_) &&
      fuel_gauge->getTemperature(bq34_temperature_) &&
      fuel_gauge->getCurrent(bq34_current_) &&
      fuel_gauge->getSOC(bq34_soc_) &&
      fuel_gauge->getMaxError(bq34_max_error_) &&
      fuel_gauge->getVoltageRaw(bq34_voltage_raw_) &&
      fuel_gauge->getFlags(bq34_flags_))) {
        uart->puts("BQ34 read error (one or more values invalid)\r\n");
      }

    #ifdef BMS_DEBUG
        uart->puts("           BQ34 MEASUREMENTS           \r\n");
        uart->puts("---------------------------------------\r\n\r\n\r\n");

        // debug print statements
        int16_t temp_c = bq34_temperature_/10 - 273;
        uart->printf("Voltage: %d mV%\r\n", bq34_voltage_);
        uart->printf("Temperature: %d C%\r\n", temp_c);
        uart->printf("Current: %d mA%\r\n", bq34_current_);
        uart->printf("SOC: %d mA%\r\n", bq34_soc_);
        uart->printf("Max Error: %d %%\r\n", bq34_max_error_);
        uart->printf("Voltage (raw): %d mV%\r\n", bq34_voltage_raw_);

        uart->printf("\tFlags: 0x%X:%\r\n", bq34_flags_);

        // Decode common flag bits
        if (bq34_flags_ & 0x0100) uart->printf("%\t- CHG (Charging Allowed)%\r\n");
        if (bq34_flags_ & 0x0200) uart->printf("%\t- FC (Fully Charged)%\r\n");
        if (bq34_flags_ & 0x0400) uart->printf("%\t- XCHG (Charging not allowed)%\r\n");
        if (bq34_flags_ & 0x0800) uart->printf("%\t- CHG_INH (Unable to charge)%\r\n");
        if (bq34_flags_ & 0x1000) uart->printf("%\t- BATLOW (Low Battery Voltage)%\r\n");
        if (bq34_flags_ & 0x2000) uart->printf("%\t- BATHI (High Battery Voltage)%\r\n");
        if (bq34_flags_ & 0x4000) uart->printf("%\t- OTD (Over Temperature Discharge)%\r\n");
        if (bq34_flags_ & 0x8000) uart->printf("%\t- OTC (Over Temperature Charge)%\r\n");

        if (bq34_flags_ & 0x0001) uart->printf("%\t- DSG (Discharging detected)%\r\n");
        if (bq34_flags_ & 0x0004) uart->printf("%\t- SOC1 (SOC First Threshold)%\r\n");
        if (bq34_flags_ & 0x0002) uart->printf("%\t- SOCF (SOC Final Threshold)%\r\n");
        if (bq34_flags_ & 0x0010) uart->printf("%\t- CF (Update Cycle Needed)%\r\n");
        if (bq34_flags_ & 0x0080) uart->printf("%\t- REST (OCV reading taken)%\r\n");
    #endif



    // other measurements

    // measurement updates bq79631
    if (!(hv_monitor->getPackVoltage_mV(pack_voltage_mV_) &&
      hv_monitor->getDieTemperature_cC(die_temp_cC_) &&
      hv_monitor->getFaultFlags(fault_flags_) &&
      hv_monitor->getCellCount(cell_count_))) {
        uart->puts("BQ79631 read error (one or more values invalid)\r\n");
      }
    #ifdef BMS_DEBUG
        uart->puts("         BQ79631 MEASUREMENTS         \r\n");
        uart->puts("---------------------------------------\r\n\r\n\r\n");

        uart->printf("Pack Voltage: %d mV%\r\n", pack_voltage_mV_);
        uart->printf("Cell count: %d \r\n", cell_count_);
        uart->printf("Die Temperature: %d C%\r\n", die_temp_cC_);

        uart->printf("\tFlags: 0x%X:%\r\n", fault_flags_);

        // thermistors

        uart->puts("\r\n      THERMISTOR MEASUREMENTS      \r\n");
        uart->puts("---------------------------------------\r\n\r\n\r\n");
        thermistors_->update();

        for (uint8_t i = 0; i < 2; i++) {
            auto r = thermistors_->getSensor(i);

            uart->printf("Raw ADC Value: %d   TH-%d: %d.%d C   Fault=%d\r\n",
                         r.adc_raw,
                         i,
                         r.temperature_dC,
                         abs(r.temperature_dC % 10),
                         static_cast<uint8_t>(r.fault));
        }

        int16_t avg = thermistors_->getAverage();
        uart->printf("Avg Temp: %d.%d C\r\n", avg, abs(avg % 10));
    #endif
}

void msd::bms::BmsMaster::update_protection() {
    // protection logic placeholder
    // will handle OV/UV, OT/UT & OSC/SCD
    // interpret flags from BQ34, BQ79631 & BQ79616
    if (state_ == BmsState::FAULT || state_ == BmsState::SHUTDOWN) {
        return;
    }

    if (bq34_flags_ & BQ34_FAULT_MASK) {
        uart->puts("\r\nBMS FAULT condition detected!\r\n");
        state_ = BmsState::FAULT;
        return;
    }

    if ((bq34_flags_ & BQ34_WARN_MASK)) {
        uart->puts("\r\nBMS WARNING condition detected!\r\n");
        state_ = BmsState::WARNING;
        return;
    }

    // no flags, normal
    if (state_ != BmsState::NORMAL) {
        uart->puts("\r\nBMS conditions normal!\r\n");
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
            // do something
            warning_led.toggle();
            break;
        case BmsState::FAULT:
            // hault all operation
            // open contactors
            error_led.toggle();
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


