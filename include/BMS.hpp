/**
 * @file BMS.hpp
 * @brief C++ interface for the Battery Management System (BMS) master controller.
 *
 * This header defines the BmsMaster class, which coordinates system-level
 * behavior including device communication, safety monitoring, state management,
 * and data logging within the BMS.
 *
 * The master operates as a centralized controller responsible for:
 * - Periodic data acquisition from sensors and ICs
 * - Fault detection and protection handling
 * - System state transitions (INIT → NORMAL → WARNING → FAULT → SHUTDOWN)
 *
 * Project: MSD_BMS
 * Author: Key'Mon Jenkins
 * Date: October 2025
 */

#ifndef BMS_H
#define BMS_H

#ifdef __cplusplus
#endif

#pragma once


#include "Thermistor.hpp"
#include "core/dev/RTCTimer.hpp"
#include "core/dev/storage/M24C32.hpp"
#include "core/manager.hpp"
#include "dev/BQ34.hpp"
#include "dev/BQ79600.hpp"
#include "dev/BQ79631.hpp"
#include <cmath>
#include <core/dev/LED.hpp>

/* =========================
 * System Timing Configuration
 * ========================= */
static constexpr uint32_t NORMAL_UPDATE_MS = 5000;
static constexpr uint32_t FAST_UPDATE_MS   = 1000;
static constexpr uint32_t SLOW_UPDATE_MS   = 10000;

/* =========================
 * EEPROM Layout (Thermistor Logging)
 * ========================= */
constexpr uint16_t EEPROM_THERM_BASE = 0x0100;
constexpr uint16_t EEPROM_THERM_STRIDE = 12;
static constexpr uint8_t NUM_THERMISTORS = 2;


namespace msd::bms {

/**
 * @class BmsMaster
 * @brief High-level BMS master controller
 *
 * Central orchestrator for the BMS. Intended to be used as a singleton or
 * instantiated once at system startup.
 *
 * @note Not thread-safe. Designed for single-threaded embedded execution.
 */
class BmsMaster {
public:
    BmsMaster() = default;
    ~BmsMaster() = default;

    BmsMaster(const BmsMaster&) = delete;
    BmsMaster& operator=(const BmsMaster&) = delete;
    BmsMaster(BmsMaster&&) = delete;
    BmsMaster& operator=(BmsMaster&&) = delete;

    void gpiowake() const;
    void init();
    void update();
    void delay_us(uint32_t us) const;
    void shutdown();
    static BmsMaster& instance();

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

private:

    /* =========================
     * Hardware Timer (1 MHz)
     * ========================= */
    TIM_HandleTypeDef htim2_;

    /**
     * @brief Initialize TIM2 for 1 MHz operation
     *
     * Configures TIM2 to increment every 1 µs for precise delay generation.
     */
    void Timer2_Init_1MHz() {
        __HAL_RCC_TIM2_CLK_ENABLE();

        htim2_.Instance = TIM2;
        htim2_.Init.Prescaler = 83; // 84 MHz / (83 + 1) = 1 MHz
        htim2_.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim2_.Init.Period = 0xFFFFFFFF;
        htim2_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim2_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        HAL_TIM_Base_Init(&htim2_);
        HAL_TIM_Base_Start(&htim2_);
    }

    bool initialized_ = false; // Initialization state flag

    /* =========================
     * BMS State Machine
     * ========================= */
    enum class BmsState {
        INIT,
        NORMAL,
        WARNING,
        FAULT,
        SHUTDOWN
    };
    BmsState state_ = BmsState::INIT; // Current system state


    /* =========================
     * BQ34 Fuel Gauge Data
     * ========================= */
    uint16_t bq34_voltage_;      ///< Battery voltage (mV)
    uint16_t bq34_temperature_;  ///< Temperature (0.1 K units)
    int16_t  bq34_current_;      ///< Current (mA), +discharge / -charge
    uint16_t bq34_soc_;          ///< State of charge (%)
    uint16_t bq34_max_error_;    ///< Maximum error (%)
    uint16_t bq34_flags_;        ///< Status flags
    uint16_t bq34_voltage_raw_;  ///< Raw voltage register value

    /* =========================
     * BQ79631 Monitor Data
     * ========================= */
    uint16_t pack_voltage_mV_; ///< Total pack voltage (mV)
    uint16_t die_temp_cC_;     ///< Die temperature (centi-degrees C)
    uint16_t fault_flags_;     ///< Device fault flags
    uint16_t cell_count_;      ///< Number of detected cells

    /* =========================
     * Protection Masks
     * ========================= */
    static constexpr uint16_t BQ34_WARN_MASK =
        // 0x0002 |  // SOCF
        // 0x0004 |  // SOC1
        // 0x0010;   // CF
        0; // placeholder until battery integration

    static constexpr uint16_t BQ34_FAULT_MASK =
        0x0400 |  // XCHG
        0x0800 |  // CHG_INH
        0x2000 |  // BATHI
        0x4000 |  // OTD
        0x8000;   // OTC

    /* =========================
    * Thermistor System
    * ========================= */
    core::io::ADC* therm_adcs_[5]; // ADC channels for thermistors
    ThermistorArray* thermistors_; // Thermistor processing interface

    /**
     * @struct ThermistorLogEntry
     * @brief EEPROM log entry format for thermistor data
     */
    struct ThermistorLogEntry {
        uint8_t sensor_id;        // Thermistor index
        uint32_t timestamp;       // Timestamp (system timebase)
        int16_t temperature_x10;  // Temperature (0.1°C resolution)
        uint8_t fault;            // Fault indicator flag
    };

    /* =========================
     * Internal Update Routines
     * ========================= */

    /**
     * @brief Update all sensor and IC measurements
     */
    void update_measurements();

    /**
     * @brief Evaluate protection conditions and faults
     */
    void update_protection();

    /**
     * @brief Advance state machine based on system conditions
     */
    void update_state_machine();
};
};
#endif
