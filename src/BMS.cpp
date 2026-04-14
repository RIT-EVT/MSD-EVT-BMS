/**
 * @file BMS.cpp
 * @brief Implementation of the Battery Management System (BMS) master controller.
 *
 * This file contains the implementation of the BmsMaster class, which manages
 * system initialization, device communication, measurement acquisition, and
 * safety/state handling.
 *
 * @note Extensive UART debug logging is currently enabled and significantly
 * impacts timing. This should be gated or removed for production deployment.
 *
 * Project: MSD_BMS
 * Author: Key'Mon Jenkins
 * Date: October 2025
 */


#include "BMS.hpp"

#include "core/utils/log.hpp"
#include "dev/BMS_meas.hpp"
#include "dev/BQ79616.hpp"
#define BMS_DEBUG

namespace IO = core::io;
namespace DEV = core::dev;

/* =========================
 * Communication Interfaces
 * ========================= */
namespace {
IO::UART* uart = nullptr;     // UART interface for debug logging
bool uart_safe_mode = true;   // Prevents blocking if UART unavailable
IO::I2C* i2c = nullptr;       // I2C bus handle
IO::SPI* spi = nullptr;       // SPI bus handle
IO::CAN* can = nullptr;       // CAN interface
}

/* =========================
 * Device Instances
 * ========================= */
namespace {
BQ34* fuel_gauge = nullptr;         // Fuel gauge (BQ34Z100)
DEV::M24C32* eeprom = nullptr;      // EEPROM storage
DEV::BQ79600* bridge = nullptr;     // SPI bridge device
DEV::BQ79631* hv_monitor = nullptr; // High-voltage monitor
DEV::BQ79616* slave = nullptr;      // Slave board

}

/* =========================
 * LED Indicators
 * ========================= */
namespace {
core::io::GPIO& status_gpio  = core::io::getGPIO<core::io::Pin::PA_5>();
DEV::LED status_led(status_gpio, DEV::LED::ActiveState::LOW);

core::io::GPIO& warning_gpio = core::io::getGPIO<core::io::Pin::PA_6>();
DEV::LED warning_led(warning_gpio, DEV::LED::ActiveState::LOW);

core::io::GPIO& error_gpio   = core::io::getGPIO<core::io::Pin::PA_7>();
DEV::LED error_led(error_gpio, DEV::LED::ActiveState::LOW);

core::io::GPIO& extra_gpio   = core::io::getGPIO<core::io::Pin::PB_0>();
DEV::LED extra_led(extra_gpio, DEV::LED::ActiveState::LOW);
}

/* =========================
 * GPIO Control Signals
 * ========================= */
namespace {
core::io::GPIO& hv_cs_gpio = core::io::getGPIO<core::io::Pin::PA_15>(); // SPI CS for HV devices
core::io::GPIO& can_gpio   = core::io::getGPIO<core::io::Pin::PB_4>();  // CAN standby control

core::io::GPIO* spi_cs_pins[] = { &hv_cs_gpio };

core::io::GPIO& dia_en_gpio = core::io::getGPIO<core::io::Pin::PC_0>();
core::io::GPIO& sel1_gpio   = core::io::getGPIO<core::io::Pin::PC_1>();
core::io::GPIO& fault_gpio  = core::io::getGPIO<core::io::Pin::PC_2>();
core::io::GPIO& latch_gpio  = core::io::getGPIO<core::io::Pin::PC_3>();
core::io::GPIO& sw_en_shared_gpio = core::io::getGPIO<core::io::Pin::PC_5>();
core::io::GPIO& mosi_gpio   = core::io::getGPIO<core::io::Pin::PC_12>();
}

/* =========================
 * CAN IDs
 * ========================= */
namespace {
constexpr uint32_t CAN_ID_WARNING       = 0x2;
constexpr uint32_t CAN_ID_BQ34_BASIC    = 0x100;
constexpr uint32_t CAN_ID_BQ34_STATUS   = 0x101;
constexpr uint32_t CAN_ID_HVM_BASIC     = 0x110;
constexpr uint32_t CAN_ID_THERM         = 0x120;
constexpr uint32_t CAN_ID_STATE         = 0x130;
constexpr uint32_t CAN_ID_SLAVE_CELLS_1 = 0x140;
constexpr uint32_t CAN_ID_SLAVE_CELLS_2 = 0x141;
constexpr uint32_t CAN_ID_SLAVE_STATUS  = 0x142;
constexpr uint32_t CAN_ID_ERROR         = 0x1FF;

}

/* =========================
 * Internal State Flags
 * ========================= */
static bool bq79600_present = false;
static bool bq79631_present = false;
static bool slave1_present  = false;
static bool slave2_present  = false;
static bool init_success    = false;

constexpr uint8_t STACK_DEVICES = 2; // bridge counts as a stack device
uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xBE, 0xEF, 0x00, 0x01, 0x02};
IO::CANMessage transmit_message(1, 8, &payload[0], true);
IO::CANMessage received_message;


/* =========================
 * Singleton Accessor
 * ========================= */

/**
 * @brief Get singleton instance of BmsMaster
 *
 * @return Reference to static BmsMaster instance
 */
msd::bms::BmsMaster& msd::bms::BmsMaster::instance() {
    static msd::bms::BmsMaster instance;
    return instance;
}

/**
 * @brief Microsecond delay using TIM2
 *
 * Busy-wait loop based on a 1 MHz hardware timer.
 *
 * @param us Delay duration in microseconds
 */
void msd::bms::BmsMaster::delay_us(uint32_t us) const {
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2_);
    while ((__HAL_TIM_GET_COUNTER(&htim2_) - start) < us) {
        __NOP();
    }
}

/* =========================
 * CAN Helper functions
 * should be moved to a different file
 * ========================= */

/**
 * @brief Control CAN transceiver standby mode
 *
 * @param standby true = standby (disabled), false = active
 */
void toggle_can(const bool standby) {
    if (standby) {
        can_gpio.writePin(IO::GPIO::State::HIGH);
    }
    else {
        can_gpio.writePin(IO::GPIO::State::LOW);
    }
}

/**
 * @brief Control CAN transceiver standby mode
 *
 * @param id CAN ID
 * @param data data to send via can
 * @param len length of message
 */
inline void send_can(uint32_t id, uint8_t* data, uint8_t len) {
    if (!can) return;

    IO::CANMessage msg(id, len, data, true); // standard ID
    toggle_can(false);
    core::time::wait(2);
    can->transmit(msg);
    toggle_can(true);
    core::time::wait(2);
}

/**
 * @brief Control CAN transceiver standby mode
 *
 * @param code specific error code
 * @param detail fault flags for error
 */
void send_error(uint8_t code, uint16_t detail) {
    uint8_t data[4];

    data[0] = code;
    data[1] = 0;
    data[2] = (detail >> 8) & 0xFF;
    data[3] = detail & 0xFF;

    send_can(CAN_ID_ERROR, data, 4);
}

/**
 *
 * @brief CAN IRQ
 *
 * @param message address to store received message
 * @param priv uart instance
 *
 * @note only used for testing
 */
void canIRQHandler(IO::CANMessage& message, void* priv) {
    uart = static_cast<IO::UART*>(priv);
    uart->printf("Message received\r\n");
    uart->printf("Message id: 0x%X \r\n", message.getId());
    uart->printf("Message length: %d\r\n", message.getDataLength());
    uart->printf("Message contents: ");

    uint8_t* message_payload = message.getPayload();
    for (int i = 0; i < message.getDataLength(); i++) {
        uart->printf("0x%02X ", message_payload[i]);
    }
    uart->printf("\r\n\r\n");
}


/**
 * @brief Perform GPIO-based wake sequence for BQ79600
 *
 * Implements the required wake pulse timing using bit-banged GPIO control.
 * Temporarily overrides SPI pin configuration to manually drive MOSI and CS.
 *
 * @note Timing is critical and aligned with datasheet specifications.
 */
void msd::bms::BmsMaster::gpiowake() const
{
    // Reconfigure MOSI and CS as GPIO outputs temporarily
    GPIOC->MODER &= ~GPIO_MODER_MODE12;        // Clear MOSI (PC12)
    GPIOC->MODER |= GPIO_MODER_MODE12_0;       // Set as output
    GPIOA->MODER &= ~GPIO_MODER_MODE15;        // Clear CS (PA15)
    GPIOA->MODER |= GPIO_MODER_MODE15_0;       // Set as output


    for (int pulse = 0; pulse < 2; pulse++) {

        // CS LOW
        hv_cs_gpio.writePin(IO::GPIO::State::LOW);
        delay_us(2);  // tCSS

        // MOSI LOW - start wake pulse
        mosi_gpio.writePin(IO::GPIO::State::LOW);
        core::time::wait(2);   // 2ms
        delay_us(750); //delay_us(300);              // + 750µs = 2.75ms total

        // MOSI HIGH - end wake pulse
        mosi_gpio.writePin(IO::GPIO::State::HIGH);
        delay_us(2);  // tCSH

        // CS HIGH
        hv_cs_gpio.writePin(IO::GPIO::State::HIGH);
    }

    core::time::wait(10);   // tREADY - wait for device to be ready

    // Restore MOSI to SPI alternate function
    GPIOC->MODER &= ~GPIO_MODER_MODE12;        // Clear MOSI
    GPIOC->MODER |= GPIO_MODER_MODE12_1;       // Restore AF mode
    GPIOC->AFR[1] &= ~(0xF << ((12-8)*4));
    GPIOC->AFR[1] |= (6 << ((12-8)*4));        // AF6 = SPI3
}

/**
 * @brief Initialize the BMS master system
 *
 * Performs:
 * - Timer initialization
 * - GPIO default configuration
 * - Communication peripheral setup (I2C, SPI, CAN, UART)
 * - Device instantiation
 * - Device handshake and validation
 *
 * @note Designed to run once; subsequent calls are ignored
 */
void msd::bms::BmsMaster::init() {
    if (initialized_) {
        return;
    }

    Timer2_Init_1MHz();

    // POST indicator: all LEDs ON initially
    extra_led.setState(core::io::GPIO::State::HIGH);
    error_led.setState(core::io::GPIO::State::HIGH);
    warning_led.setState(core::io::GPIO::State::HIGH);
    status_led.setState(core::io::GPIO::State::HIGH);

    // Default GPIO states
    dia_en_gpio.writePin(core::io::GPIO::State::LOW);
    sel1_gpio.writePin(core::io::GPIO::State::LOW);
    latch_gpio.writePin(core::io::GPIO::State::LOW);
    hv_cs_gpio.writePin(core::io::GPIO::State::HIGH);
    can_gpio.writePin(IO::GPIO::State::LOW);
    sw_en_shared_gpio.writePin(IO::GPIO::State::HIGH);
    fault_gpio.writePin(IO::GPIO::State::HIGH);
    mosi_gpio.writePin(IO::GPIO::State::HIGH);

    core::time::wait(10);

    // Initialize communication peripherals
    i2c = &IO::getI2C<IO::Pin::I2C_SCL, IO::Pin::I2C_SDA>();

    can = &IO::getCAN<IO::Pin::PB_6, IO::Pin::PB_5>();

    can->addIRQHandler(canIRQHandler, &uart);

    // join CAN network
    IO::CAN::CANStatus result = can->connect();

    spi = &IO::getSPI<IO::Pin::PC_10, IO::Pin::PC_12, IO::Pin::PC_11>(spi_cs_pins, 1);

    spi->configureSPI(SPI_SPEED_1MHZ, IO::SPI::SPIMode::SPI_MODE0, SPI_MSB_FIRST);

    #ifdef BMS_DEBUG
    uart = &IO::getUART<IO::Pin::UART_TX, IO::Pin::UART_RX>(9600);
    core::log::LOGGER.setUART(uart);
    core::log::LOGGER.setLogLevel(core::log::Logger::LogLevel::DEBUG);

    if (!uart->isWritable()) {
        uart_safe_mode = false;
    }

    uart->puts("              BMS init start           \r\n");
    uart->puts("---------------------------------------\r\n\r\n");
    uart->puts("Starting initializations!\r\n\r\n");
    #endif


    if (result != IO::CAN::CANStatus::OK) {
        #ifdef BMS_DEBUG
        uart->printf("Failed to connect to the CAN network\r\n");
        #endif
        state_ = BmsState::FAULT;
        return;
    }

    /**
     * ON-BOARD DEVICE INITILIZATION
     * BQ34, M24C32
     * need to add success/failure verification (shouldn't fail though)
     */
    static BQ34 fuel_gage_inst{i2c};
    static DEV::M24C32 eeprom_inst{0x50, *i2c};

    fuel_gauge = &fuel_gage_inst;
    eeprom     = &eeprom_inst;

    /**
     * SENSOR SETUP
     * thermistors
     */

    // low voltage thermistors
    therm_adcs_[0] = &IO::getADC<IO::Pin::PA_0>();
    therm_adcs_[1] = &IO::getADC<IO::Pin::PA_1>();

    // high voltage thermistors on HVM
    therm_adcs_[2] = &IO::getADC<IO::Pin::PB_4>();
    therm_adcs_[3] = &IO::getADC<IO::Pin::PB_3>();
    therm_adcs_[4] = &IO::getADC<IO::Pin::PD_2>();

    static ThermistorArray therm_array{therm_adcs_};
    thermistors_ = &therm_array;

    extra_led.setState(core::io::GPIO::State::LOW); // init stage passes

    #ifdef BMS_DEBUG
    if (uart_safe_mode) {
        uart->puts("All devices initialized!\r\n");
        uart->puts("---------------------------------------\r\n\r\n");
        uart->puts("Checking Master Devices...\r\n\r\n");
    }
    #endif


    /**
     * MASTER DEVICE HANDSHAKE
     * BQ34Z100 (Fuel Gage), M24C32 (EEPROM)
     */

    // detect bq34
    if (uint16_t controlStatus = 0; !fuel_gauge->readWord(CONTROL, controlStatus)) {
        #ifdef BMS_DEBUG
        if (uart_safe_mode) {
            uart->puts("ERROR: BQ34Z100 not detected!\r\n");
        }
        #endif
        // state_ = BmsState::FAULT;
        // return; //go to error state
    }
    #ifdef BMS_DEBUG
    if (uart_safe_mode) {
        uart->puts("BQ34Z100 detected!\r\n");
    }
    #endif

    // detect m24c32
    #ifdef BMS_DEBUG
    if (uart_safe_mode) {
        uart->puts("Checking EEPROM integrity...\r\n");
    }
    #endif
    constexpr uint32_t test = 0xBEEF;
    constexpr uint16_t EEPROM_TEST_ADDR = 0x0000;
    eeprom->writeWord(EEPROM_TEST_ADDR, test);
    core::time::wait(5);
    if (eeprom->readWord(EEPROM_TEST_ADDR) != test) {
        #ifdef BMS_DEBUG
        if (uart_safe_mode) {
            uart->puts("ERROR: EEPROM failed integrity check\r\n");
        }
        #endif
        // state_ = BmsState::FAULT;
        // return; // go to error state
    }
    #ifdef BMS_DEBUG
    if (uart_safe_mode) {
        uart->puts("EEPROM passed integrity check!\r\n");
    }
    #endif
    error_led.setState(core::io::GPIO::State::LOW); // MASTER device detection complete

    #ifdef BMS_DEBUG
    if (uart_safe_mode) {
        uart->puts("Master Devices OK!\r\n");
        uart->puts("---------------------------------------\r\n\r\n");
        uart->puts("Checking Slave Devices...\r\n\r\n");
    }
    #endif


    /**
     * SLAVE DEVICE HANDSHAKE
     * BQ79600 (SPI bridge), BQ79631 (HVM), BQ79616 (Slave board)
     */

    // detect BQ79600
    #ifdef BMS_DEBUG
    if (uart_safe_mode) {
        uart->puts("=== BQ79600 INITIALIZATION ===\r\n");
        uart->puts("---------------------------------------\r\n");
    }
    #endif

    // Initialize bridge device instance
    static DEV::BQ79600 bridge_inst{*spi, 0x0, *uart};
    bridge = &bridge_inst;

    // Wake from GPIO
    gpiowake();

    // while (true) {
    //     bridge->singleWrite(0x00, 0x309, 0x21);
    //     core::time::wait(1000);
    // }

    core::time::wait(50);

    // Simple read test - don't call init(), just try reading
    uint8_t dev_conf = 0;
    if (bridge->singleRead(0x00, 0x2001, dev_conf)) {
        #ifdef BMS_DEBUG
        if (uart_safe_mode) {
            uart->printf("READ SUCCESS! DEV_CONF1 = 0x%02X\r\n", dev_conf);
        }
        #endif
        if (dev_conf == 0x14) {
            bridge->init(); // call init
            init_success = true;
            bq79600_present = true;
        }
    } else {
        #ifdef BMS_DEBUG
        if (uart_safe_mode) {
            uart->puts("ERROR: Bridge not identified!\r\n");
        }
        #endif
        // shouldn't get here
        bq79600_present = false;
    }

    if (!init_success) {
        #ifdef BMS_DEBUG
        if (uart_safe_mode) {
            uart->puts("ERROR: Could not initialize BQ79600\r\n");
        }
        #endif
        // shouldn't get here
        bq79600_present = false;
    }

    warning_led.setState(core::io::GPIO::State::LOW); // bridge detection complete

    // return;

    #ifdef BMS_DEBUG
    if (uart_safe_mode) {
        uart->puts("Sending wake to devices again...\r\n");
    }
    #endif
    bridge->singleWrite(0x00, 0x309, 0x21);


    core::time::wait(11*STACK_DEVICES);

    // start auto-addressing
    bridge->autoAddressStack(STACK_DEVICES);

    // test loop
    // while (true) {
    // bridge->singleWrite(0x00, 0x309, 0x21);
    //     core::time::wait(500);
    // }

    // create instances of devices
    static DEV::BQ79631 hv_monitor_inst{(*bridge), 1};
    hv_monitor = &hv_monitor_inst;

    static DEV::BQ79616 slave_inst{(*bridge), 1};
    slave = &slave_inst;


    // Verify HVM at address 1
    if (uint16_t hvm_val = 0; hv_monitor->readDeviceID(hvm_val)) {
        uart->printf("✓ Device 1 (HVM): DIR0_ADDR = 0x%02X\r\n", hvm_val);
        bq79631_present = true;
    } else {
        uart->puts("✗ Device 1 (HVM) not responding\r\n");
    }

    core::time::wait(5);


    // Verify Slave at address 2
    if (uint16_t slave_val = 0; slave->readDeviceID(slave_val)) {
        uart->printf("✓ Device 2 (Slave1): DIR0_ADDR = 0x%02X\r\n", slave_val);
        slave1_present = true;
    } else {
        uart->puts("✗ Device 2 (Slave1) not responding\r\n");
    }

    if (!(bq79600_present && slave1_present)) {
        #ifdef BMS_DEBUG
            if (uart_safe_mode) {
                uart->puts("ERROR: Auto-addressing failed\r\n");
            }
        #endif
        // state_ = BmsState::FAULT;
    } else {
        #ifdef BMS_DEBUG
        if (uart_safe_mode) {
            uart->puts("Auto-addressing Success!\r\n");
        }
        #endif
        // initialize register
        if (!bridge->initRegisters()) {
            #ifdef BMS_DEBUG
            if (uart_safe_mode) {
                uart->puts("Initializing registers...\r\n");
                uart->puts("ERROR: Register Init failed\r\n");
            }
            #endif
            state_ = BmsState::FAULT;
        }
    }

    status_led.setState(core::io::GPIO::State::LOW); // full device configuration complete

    initialized_ = true;
    core::time::wait(300);
    #ifdef BMS_DEBUG
        if (uart_safe_mode) {
            uart->puts("BMS Master initialized!\r\n\r\n");
        }
    #endif
}


/**
 * @brief Main periodic update loop
 *
 * Executes measurement acquisition, protection evaluation,
 * and state machine progression.
 */
void msd::bms::BmsMaster::update() {
    if (!initialized_) {
        return;
    }

    status_led.setState(core::io::GPIO::State::HIGH);

    update_measurements();
    update_protection();
    update_state_machine();

    status_led.setState(core::io::GPIO::State::LOW);

    // test contactor switch
    // when fault low -> switch on, else off
    // device draws less current when fault on, proving latch open
    // extra_led.setState(core::io::GPIO::State::HIGH);
    // fault_gpio.writePin(core::io::GPIO::State::LOW);
    // core::time::wait(5000);
    // fault_gpio.writePin(core::io::GPIO::State::HIGH);
    // extra_led.setState(core::io::GPIO::State::LOW);

}

/**
 * @brief Update all sensor and device measurements
 *
 * Reads:
 * - Slave (BQ79616)
 * - Fuel gauge (BQ34)
 * - HV monitor (BQ79631)
 * - Thermistor array
 *
 * Updates internal state variables and triggers FAULT state on failure.
 *
 * @note this function should be separated into a separate file
 *        there should be a function for reading data from each device
 *        with debug output and CAN output included
 */
void msd::bms::BmsMaster::update_measurements() {
    // main slave updates first
    // will implement after daisy chain fixed


    uint16_t fg[7];
    uint16_t hvm[4];
    int16_t therm[3];

    uint16_t slave_cells[16]; // max supported
    uint16_t slave_aux[2];    // temp + faults

    bool slave_ok = false;

    if (slave) {
        bool v_ok = slave->getCellVoltages_mV(slave_cells, 6); // start with 6 cells
        bool t_ok = slave->getDieTemperature_cC(slave_aux[0]);
        bool f_ok = slave->getFaultFlags(slave_aux[1]);

        slave_ok = v_ok && t_ok && f_ok;

        if (!slave_ok) {
            state_ = BmsState::WARNING;
            send_error(0x04, 0);
        }
    }

    bool fg_ok   = get_FuelGauge_measurements(fg, fuel_gauge);
    bool hvm_ok  = get_HVM_measurements(hvm, hv_monitor);
    bool th_ok   = get_Thermistors_measurements(therm, thermistors_);

    if (!fg_ok) {
        state_ = BmsState::FAULT;
        send_error(0x01, 0);
        return;
    }

    if (!hvm_ok) {
        state_ = BmsState::FAULT;
        send_error(0x02, 0);
        return;
    }

    if (!th_ok) {
        state_ = BmsState::WARNING;
        send_error(0x03, 0);
        return;
    }

    // Assign to class variables (clean separation)
    bq34_voltage_      = fg[0];
    bq34_temperature_  = fg[1];
    bq34_current_      = fg[2];
    bq34_soc_          = fg[3];
    bq34_max_error_    = fg[4];
    bq34_voltage_raw_  = fg[5];
    bq34_flags_        = fg[6];

    pack_voltage_mV_   = hvm[0];
    die_temp_cC_       = hvm[1];
    fault_flags_       = hvm[2];
    cell_count_        = hvm[3];



    state_ = BmsState::NORMAL;

    // Slave CAN message //

    //-----------------//

    // frame 1 cell count 1-4
    uint8_t slave_data1[8];

    // Cell 1–4
    for (int i = 0; i < 4; i++) {
        slave_data1[i*2]     = (slave_cells[i] >> 8) & 0xFF;
        slave_data1[i*2 + 1] = slave_cells[i] & 0xFF;
    }

    send_can(CAN_ID_SLAVE_CELLS_1, slave_data1, 8); // should be moved to BMS_can file soon
    core::time::wait(100); // should remove & use global throttle

    uint8_t slave_data2[8];

    // frame 2 cell count 5-6

    // Cell 5–6 (rest padded)
    for (int i = 0; i < 2; i++) {
        slave_data2[i*2]     = (slave_cells[i+4] >> 8) & 0xFF;
        slave_data2[i*2 + 1] = slave_cells[i+4] & 0xFF;
    }

    // pad remaining bytes
    for (int i = 4; i < 8; i++) {
        slave_data2[i] = 0;
    }

    send_can(CAN_ID_SLAVE_CELLS_2, slave_data2, 8);
    core::time::wait(100);

    // frame 3 temp & faults
    uint8_t slave_status[8];

    // die temp
    slave_status[0] = (slave_aux[0] >> 8) & 0xFF;
    slave_status[1] = slave_aux[0] & 0xFF;

    // fault flags
    slave_status[2] = (slave_aux[1] >> 8) & 0xFF;
    slave_status[3] = slave_aux[1] & 0xFF;

    // padding
    for (int i = 4; i < 8; i++) {
        slave_status[i] = 0;
    }

    send_can(CAN_ID_SLAVE_STATUS, slave_status, 8);
    core::time::wait(100);

    //-----------------//


    // BQ34 CAN message //

    //-----------------//
    uint8_t bq34_data[8];

    // voltage
    bq34_data[0] = (bq34_voltage_ >> 8) & 0xFF;
    bq34_data[1] = bq34_voltage_ & 0xFF;

    // current
    bq34_data[2] = (bq34_current_ >> 8) & 0xFF;
    bq34_data[3] = bq34_current_ & 0xFF;

    bq34_data[4] = fg[3] & 0xFF;   // SOC

    int16_t temp_c = fg[1]/10 - 273;
    bq34_data[5] = temp_c & 0xFF;

    bq34_data[6] = 0;
    bq34_data[7] = 0;

    send_can(CAN_ID_BQ34_BASIC, bq34_data, 8);

    core::time::wait(100);

    // BQ34 Flag data
    uint8_t bq34_flag_data[4];

    bq34_flag_data[0] = (bq34_flags_ >> 8) & 0xFF;  // flags
    bq34_flag_data[1] = bq34_flags_ & 0xFF;
    bq34_flag_data[2] = fg[4];          // max error
    bq34_flag_data[3] = 0;

    send_can(CAN_ID_BQ34_BASIC, bq34_flag_data, 4);
    core::time::wait(100);

    //-----------------//

    // HVM CAN message //

    //-----------------//
    uint8_t hvm_data[8];

    // pack voltage -> 2 bytes
    hvm_data[0] = (pack_voltage_mV_ >> 8) & 0xFF;
    hvm_data[1] = pack_voltage_mV_ & 0xFF;

    // die temp -> 2 bytes
    hvm_data[2] = (die_temp_cC_ >> 8) & 0xFF;
    hvm_data[3] = die_temp_cC_ & 0xFF;

    // fault flags -> 2 bytes
    hvm_data[0] = (fault_flags_ >> 8) & 0xFF;
    hvm_data[1] = fault_flags_ & 0xFF;

    // cell count -> 2 bytes
    hvm_data[2] = (cell_count_ >> 8) & 0xFF;
    hvm_data[3] = cell_count_ & 0xFF;

    send_can(CAN_ID_HVM_BASIC, hvm_data, 8);
    core::time::wait(100);


    //-----------------//

    // Thermistor CAN message //

    //-----------------//

    uint8_t thermistor_data[8];

    thermistor_data[0] = therm[0] / 10; // T1
    thermistor_data[1] = therm[1] / 10; // T2
    thermistor_data[2] = therm[2] / 10; // avg

    thermistor_data[3] = 0;
    thermistor_data[4] = 0;
    thermistor_data[5] = 0;
    thermistor_data[6] = 0;
    thermistor_data[7] = 0;

    send_can(CAN_ID_THERM, thermistor_data, 8);
    core::time::wait(100);

    //-----------------//

}

/**
 * @brief Evaluate protection conditions
 *
 * Checks device flags against defined masks and updates system state:
 * - FAULT if critical condition detected
 * - WARNING if non-critical condition detected
 * - NORMAL otherwise
 */
void msd::bms::BmsMaster::update_protection() {
    // protection logic placeholder
    // will handle OV/UV, OT/UT & OSC/SCD
    // interpret flags from BQ34, BQ79631 & BQ79616
    if (state_ == BmsState::FAULT || state_ == BmsState::SHUTDOWN) {
        return;
    }

    if (bq34_flags_ & BQ34_FAULT_MASK) {
        #ifdef BMS_DEBUG
            if (uart_safe_mode) {
                uart->puts("\r\nBMS FAULT condition detected!\r\n");
            }
        #endif
        state_ = BmsState::FAULT;
        send_error(CAN_ID_ERROR, fault_flags_);
        return;
    }

    if ((bq34_flags_ & BQ34_WARN_MASK)) {
        #ifdef BMS_DEBUG
            if (uart_safe_mode) {
                uart->puts("\r\nBMS WARNING condition detected!\r\n");
            }
        #endif
        state_ = BmsState::WARNING;
        send_error(CAN_ID_WARNING, bq34_flags_);
        return;
    }

    // no flags, normal
    if (state_ != BmsState::NORMAL) {
        #ifdef BMS_DEBUG
            if (uart_safe_mode) {
                uart->puts("\r\nBMS conditions normal!\r\n");
            }
        #endif
    }
    state_ = BmsState::NORMAL;
}

/**
 * @brief Update BMS state machine
 *
 * Handles transitions and associated behaviors for each state.
 */
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

    uint8_t data[2];

    data[0] = static_cast<uint8_t>(state_);
    data[1] = 0;

    send_can(CAN_ID_STATE, data, 2);
}


/**
 * @brief Shutdown the BMS system
 *
 * Safely disables system operation and resets initialization state.
 */
void msd::bms::BmsMaster::shutdown() {
    if (!initialized_) {
        return;
    }
    #ifdef BMS_DEBUG
        if (uart_safe_mode) {
            uart->puts("BMS Master Shutdown\r\n");
        }
    #endif

    initialized_ = false;
}


