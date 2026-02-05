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
// #define BMS_DEBUG

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

// helper vars
static bool bq79600_present = false;

// BMS instance
msd::bms::BmsMaster& msd::bms::BmsMaster::instance() {
    static msd::bms::BmsMaster instance;
    return instance;
}
bool BQ79600_readReg(core::io::SPI& spi, uint8_t device,
                     uint8_t reg, uint8_t& out)
{
    if (!spi.startTransmission(device))
        return false;

    // READ command: MSB = 1
    spi.write(0x80 | reg);

    // Clock out response
    spi.write(0x00);
    spi.read(&out);

    spi.endTransmission(device);
    return true;
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

    // Set initial state for GPIO PINS
    hv_cs_gpio.writePin(core::io::GPIO::State::HIGH);

    core::time::wait(10);


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
    spi = &IO::getSPI<IO::Pin::PC_10, IO::Pin::PC_12, IO::Pin::PC_11>(spi_cs_pins, 1); // spi init
    spi->configureSPI(SPI_SPEED_1MHZ, IO::SPI::SPIMode::SPI_MODE0, SPI_MSB_FIRST);
    #ifdef BMS_DEBUG
        uart->puts("              BMS init start           \r\n");
        uart->puts("---------------------------------------\r\n\r\n");
        uart->puts("Starting initializations!\r\n\r\n");
    #endif


    /**
     * ON-BOARD DEVICE INITILIZATION
     * BQ34, M24C32, BQ79631
     * need to add success/failure verification (shouldn't fail though)
     */
    static BQ34 fuel_gage_inst{i2c};
    static DEV::M24C32 eeprom_inst{0x50, *i2c};
    static DEV::BQ79631 hv_monitor_inst{*spi, 0x0, *uart};
    fuel_gauge = &fuel_gage_inst;
    hv_monitor = &hv_monitor_inst;
    eeprom = &eeprom_inst;

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
        uart->puts("Checking Master Devices...\r\n\r\n");
    #endif


    /**
     * DEVICE HANDSHAKE
     * BQ34Z100 (Fuel Gage), M24C32 (EEPROM)
     */

    // detect bq34
    if (uint16_t controlStatus = 0; !fuel_gauge->readWord(CONTROL, controlStatus)) {
        #ifdef BMS_DEBUG
            uart->puts("ERROR: BQ34Z100 not detected!\r\n");
        #endif
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
    core::time::wait(5);
    if (eeprom->readWord(EEPROM_TEST_ADDR) != test) {
        #ifdef BMS_DEBUG
            uart->puts("ERROR: EEPROM failed integrity check\r\n");
        #endif
        state_ = BmsState::FAULT;
        return; // go to error state
    }
    #ifdef BMS_DEBUG
        uart->puts("EEPROM passed integrity check!\r\n");
    #endif
    error_led.setState(core::io::GPIO::State::LOW); // device detection complete

    #ifdef BMS_DEBUG
        uart->puts("Master Devices OK!\r\n");
        uart->puts("---------------------------------------\r\n\r\n");
        uart->puts("Checking Slave Devices...\r\n\r\n");
    #endif

    /**
     * SLACK DEVICE HANDSHAKE
     * BQ79631 (HVM) + BQ79161 (slave)
     * need to add success/failure verification (shouldn't fail though)
     */
    #ifdef BMS_DEBUG
        uart->puts("Detecting High Voltage Monitor...\r\n");
    #endif

    /* --------- BQ79600 test!! -------------
    // wake and detect bq79631
    #ifdef BMS_DEBUG
    uart->puts("=== BQ79600 INITIALIZATION ===\r\n");
    uart->puts("---------------------------------------\r\n");
    #endif

    // Step 1: Send WAKE PING to bring device from SHUTDOWN to ACTIVE
    uart->puts("Step 1: Sending WAKE ping...\r\n");
    uart->puts("  (RX pin held LOW for 2.5-3ms)\r\n");

    // In SPI mode, WAKE ping = MOSI low for 2.5-3ms while nCS is HIGH
    // Your wake() function sends 10 bytes of 0x00, which at 1MHz SPI = 80us
    // This is NOT enough! You need 2500us minimum!

    // Better wake implementation:
    if (!spi->startTransmission(0)) {
        uart->puts("ERROR: SPI transmission start failed\r\n");
        state_ = BmsState::FAULT;
        return;
    }

        hv_monitor->wake();

    spi->endTransmission(0);
    uart->puts("  WAKE ping sent\r\n");

    // Step 2: Wait for device to initialize (tSU(WAKE_SHUT))
    uart->puts("\r\nStep 2: Waiting for device initialization...\r\n");
    uart->puts("  (tSU(WAKE_SHUT) = 2-3.5ms)\r\n");
    core::time::wait(5);  // Wait 5ms to be safe
    uart->puts("  Device should now be in ACTIVE mode\r\n");

    // Step 3: Now try to read/write
    uart->puts("\r\nStep 3: Testing communication...\r\n");
    uart->puts("  Writing to CONTROL1 (0x0309) = 0x01...\r\n");

    if (!hv_monitor->writeReg16(0x00, 0x0309, 0x01)) {
        uart->puts("ERROR: Write failed\r\n");
        state_ = BmsState::FAULT;
        return;
    }
    uart->puts("  Write successful!\r\n");
    core::time::wait(10);

    // Step 4: Read back to verify
    uart->puts("\r\nStep 4: Reading CONTROL1 back...\r\n");
    uint8_t ctrl_val = 0;
    if (!hv_monitor->readReg16(0x00, 0x0309, ctrl_val)) {
        uart->puts("ERROR: Read failed\r\n");
        state_ = BmsState::FAULT;
        return;
    }
    uart->printf("SUCCESS! CONTROL1 = 0x%02X\r\n", ctrl_val);

    // Step 5: Read Device ID
    uart->puts("\r\nStep 5: Reading Device ID...\r\n");
    uint16_t device_id = 0;
    if (!hv_monitor->readDeviceID(device_id)) {
        uart->puts("ERROR: Could not read device ID\r\n");
        state_ = BmsState::FAULT;
        return;
    }
    uart->printf("SUCCESS! Device ID: 0x%04X\r\n", device_id);
    uart->puts("\r\n=== BQ79600 INITIALIZED SUCCESSFULLY ===\r\n\r\n");
    */

    #ifdef BMS_DEBUG
        uart->puts("Initializing BQ79600...\r\n");
    #endif

    hv_monitor->wake();
    core::time::wait(2);
    uint16_t id;

    uint8_t ctrl1 = 0;
    bool ok = hv_monitor->writeReg16(0x00, 0x0309, 0x01);
    // hv_monitor->readReg16(0x00, 0x0309, ctrl1);
    core::time::wait(2);

    if (ok) {
        #ifdef BMS_DEBUG
            uart->puts("Write sent!\r\n");
        #endif
    }
    if (hv_monitor->readReg16(0x00, 0x0309, ctrl1)) {
        #ifdef BMS_DEBUG
            uart->printf("BQ79600 OK code: 0x%02X!\r\n", ctrl1);
        #endif
        bq79600_present = true;
        warning_led.setState(core::io::GPIO::State::LOW);
    } else {
        #ifdef BMS_DEBUG
            uart->printf("WARNING: BQ79600 not responding (low-power / no stack power)\r\n"
            "         Continuing without HV monitoring\r\n");
        #endif
        bq79600_present = false;
    }

    if (bq79600_present) {
        if (!hv_monitor->readDeviceID(id)) {
            #ifdef BMS_DEBUG
                uart->printf("HVM ID: %d\r\n", id);
                uart->puts("ERROR: HVM not responding!\r\n");
            #endif
            state_ = BmsState::FAULT;
            return;
        }
        #ifdef BMS_DEBUG
            uart->printf("High Voltage Monitor ID: 0x%04X\r\n", id);
        #endif
        // state_ = BmsState::FAULT;
        #ifdef BMS_DEBUG
            uart->puts("High Voltage Monitor detected!\r\n");
        #endif
        status_led.setState(core::io::GPIO::State::LOW);

        // doesn't work yet
        uint16_t slave_id = 0;
        if (!hv_monitor->readDeviceID(slave_id)) {
            #ifdef BMS_DEBUG
                uart->puts("ERROR: Slave not responding!\r\n");
            #endif
            state_ = BmsState::FAULT;
            return;
        }
    }



    // add any device configurations here
    status_led.setState(core::io::GPIO::State::LOW); // device configuration complete

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
        #ifdef BMS_DEBUG
            uart->puts("BQ34 read error (one or more values invalid)\r\n");
        #endif
      } else {
          #ifdef BMS_DEBUG
          uart->puts("           BQ34 MEASUREMENTS           \r\n");
          uart->puts("---------------------------------------\r\n\r\n\r\n");

          // debug print statements
          int16_t temp_c = bq34_temperature_/10 - 273;
          uart->printf("Voltage: %d mV\r\n", bq34_voltage_);
          uart->printf("Temperature: %d C\r\n", temp_c);
          uart->printf("Current: %d mA\r\n", bq34_current_);
          uart->printf("SOC: %d mA\r\n", bq34_soc_);
          uart->printf("Max Error: %d \r\n", bq34_max_error_);
          uart->printf("Voltage (raw): %d mV\r\n", bq34_voltage_raw_);

          uart->printf("\tFlags: 0x%X:\r\n", bq34_flags_);

          // Decode common flag bits
          if (bq34_flags_ & 0x0100) uart->printf("\t- CHG (Charging Allowed)\r\n");
          if (bq34_flags_ & 0x0200) uart->printf("\t- FC (Fully Charged)\r\n");
          if (bq34_flags_ & 0x0400) uart->printf("\t- XCHG (Charging not allowed)\r\n");
          if (bq34_flags_ & 0x0800) uart->printf("\t- CHG_INH (Unable to charge)\r\n");
          if (bq34_flags_ & 0x1000) uart->printf("\t- BATLOW (Low Battery Voltage)\r\n");
          if (bq34_flags_ & 0x2000) uart->printf("\t- BATHI (High Battery Voltage)\r\n");
          if (bq34_flags_ & 0x4000) uart->printf("\t- OTD (Over Temperature Discharge)\r\n");
          if (bq34_flags_ & 0x8000) uart->printf("\t- OTC (Over Temperature Charge)\r\n");

          if (bq34_flags_ & 0x0001) uart->printf("\t- DSG (Discharging detected)\r\n");
          if (bq34_flags_ & 0x0004) uart->printf("\t- SOC1 (SOC First Threshold)\r\n");
          if (bq34_flags_ & 0x0002) uart->printf("\t- SOCF (SOC Final Threshold)\r\n");
          if (bq34_flags_ & 0x0010) uart->printf("\t- CF (Update Cycle Needed)\r\n");
          if (bq34_flags_ & 0x0080) uart->printf("\t- REST (OCV reading taken)\r\n");
          #endif
      }



    // other measurements
    if (bq79600_present) {
        if (!(hv_monitor->getPackVoltage_mV(pack_voltage_mV_) &&
          hv_monitor->getDieTemperature_cC(die_temp_cC_) &&
          hv_monitor->getFaultFlags(fault_flags_) &&
          hv_monitor->getCellCount(cell_count_))) {
            #ifdef BMS_DEBUG
                uart->puts("BQ79631 read error (one or more values invalid)\r\n");
            #endif
          }else {
              #ifdef BMS_DEBUG
              uart->puts("         BQ79631 MEASUREMENTS         \r\n");
              uart->puts("---------------------------------------\r\n\r\n\r\n");

              uart->printf("Pack Voltage: %d mV\r\n", pack_voltage_mV_);
              uart->printf("Cell count: %d \r\n", cell_count_);
              uart->printf("Die Temperature: %d C\r\n", die_temp_cC_);

              uart->printf("\tFlags: 0x%X:\r\n", fault_flags_);
              #endif
          }
    }
    // thermistors
    #ifdef BMS_DEBUG
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
        #ifdef BMS_DEBUG
            uart->puts("\r\nBMS FAULT condition detected!\r\n");
        #endif
        state_ = BmsState::FAULT;
        return;
    }

    if ((bq34_flags_ & BQ34_WARN_MASK)) {
        #ifdef BMS_DEBUG
            uart->puts("\r\nBMS WARNING condition detected!\r\n");
        #endif
        state_ = BmsState::WARNING;
        return;
    }

    // no flags, normal
    if (state_ != BmsState::NORMAL) {
        #ifdef BMS_DEBUG
            uart->puts("\r\nBMS conditions normal!\r\n");
        #endif
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
    #ifdef BMS_DEBUG
        uart->puts("BMS Master Shutdown\r\n");
    #endif

    initialized_ = false;
}


