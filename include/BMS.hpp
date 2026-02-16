#ifndef BMS_H
#define BMS_H

#ifdef __cplusplus
#endif

#pragma once

/**
 * @file BMS.hpp
 * @brief C++ interface for the Battery Management System (BMS) master control.
 *
 * This header defines a C++ class used for managing the BMS. The BMS master is
 * responsible for coordinating communications, safety checks, and fault handling
 * in the Battery Management System.
 *
 * Project: MSD_BMS
 * Author: Key'Mon Jenkins
 * Date: October 2025
 */

#include "Thermistor.hpp"
// #include "core/dev/Thermistor.hpp"
#include "core/dev/RTCTimer.hpp"
#include "core/dev/storage/M24C32.hpp"
#include "core/manager.hpp"
#include "dev/BQ34.hpp"
#include "dev/BQ79600.hpp"
#include "dev/BQ79631.hpp"
#include <cmath>
#include <core/dev/LED.hpp>

static constexpr uint32_t NORMAL_UPDATE_MS = 5000;
static constexpr uint32_t FAST_UPDATE_MS   = 1000;
static constexpr uint32_t SLOW_UPDATE_MS   = 10000;
constexpr uint16_t EEPROM_THERM_BASE = 0x0100;
constexpr uint16_t EEPROM_THERM_STRIDE = 12;
static constexpr uint8_t NUM_THERMISTORS = 2;


namespace msd::bms {

/**
 * @brief High-level BMS master controller.
 *
 * Create an instance or use the singleton via instance() to manage the
 * lifecycle of the BMS master. Methods are lightweight and designed to be
 * called from the application's initialization and periodic update loops.
 */

class BmsMaster {
public:
    BmsMaster() = default;
    ~BmsMaster() = default;

    BmsMaster(const BmsMaster&) = delete;
    BmsMaster& operator=(const BmsMaster&) = delete;
    BmsMaster(BmsMaster&&) = delete;
    BmsMaster& operator=(BmsMaster&&) = delete;

    /**
     * @brief Initialize the BMS master.
     *
     * Perform hardware and software initialization required before normal
     * operation. Safe to call multiple times; subsequent calls will be no-ops.
     */
    void init();

    /**
     * @brief Periodic update routine for the BMS master.
     *
     * Should be called periodically to monitor system status, process
     * communications, and handle safety events.
     */
    void update();

    void delay_us(uint32_t us);

    /**
     * @brief Shutdown the BMS master.
     *
     * Safely de-initializes the BMS master and performs any necessary cleanup.
     */
    void shutdown();

    /**
     * @brief Access a process-wide singleton instance.
     *
     * Convenience for code that prefers a global BMS master instance.
     */
    static BmsMaster& instance();

    /**
     * @brief Query whether the master is initialized.
     */
    bool is_initialized() const noexcept { return initialized_; }

    void spiCommClear();


private:

    TIM_HandleTypeDef htim2_;

    void Timer2_Init_1MHz() {
        __HAL_RCC_TIM2_CLK_ENABLE();

        htim2_.Instance = TIM2;
        htim2_.Init.Prescaler = 83;
        htim2_.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim2_.Init.Period = 0xFFFFFFFF;
        htim2_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim2_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        HAL_TIM_Base_Init(&htim2_);
        HAL_TIM_Base_Start(&htim2_);
    }

    bool initialized_ = false;

    // BMS state
    enum class BmsState {
        INIT,
        NORMAL,
        WARNING,
        FAULT,
        SHUTDOWN
    };
    BmsState state_ = BmsState::INIT;


    // BQ34
    uint16_t bq34_voltage_;     // in mV
    uint16_t bq34_temperature_; // convert to C: (temp / 10) - 273.15
    int16_t  bq34_current_;      // in mA; positive = discharging, negative = charging
    uint16_t bq34_soc_;         // state of charge in %
    uint16_t bq34_max_error_;         // state of health in %
    uint16_t bq34_flags_;       // flags
    uint16_t bq34_voltage_raw_; // raw voltage value

    // BQ79631
    uint16_t pack_voltage_mV_;
    uint16_t  die_temp_cC_;
    uint16_t fault_flags_;
    uint16_t cell_count_;

    static constexpr uint16_t BQ34_WARN_MASK =
        // 0x0002 |  // SOCF
        // 0x0004 |  // SOC1
        // 0x0010;   // CF
        0; // placeholder until battery connected

    static constexpr uint16_t BQ34_FAULT_MASK =
        0x0400 |  // XCHG
        0x0800 |  // CHG_INH
        0x2000 |  // BATHI
        0x4000 |  // OTD
        0x8000;   // OTC

    // EEPROM
    core::io::ADC* therm_adcs_[5];
    ThermistorArray* thermistors_;

    struct ThermistorLogEntry {
        uint8_t sensor_id;
        uint32_t timestamp;
        int16_t temperature_x10;  // 0.1°C resolution
        uint8_t fault;
    };

    void update_measurements();
    void update_protection();
    void update_state_machine();


};
};
#endif
