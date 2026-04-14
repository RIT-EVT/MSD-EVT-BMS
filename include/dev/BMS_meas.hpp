/**
@file BMS_meas.hpp
@brief C++ header for reading device measurements and reporting back to BMS.cpp.

This header defines functions to read device measurements for the use of BMS.cpp

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: April 2026
*/

#ifndef MSD_EVT_BMS_BMS_MEAS_HPP
#define MSD_EVT_BMS_BMS_MEAS_HPP

#include "BMS.hpp"
#include "core/dev/Thermistor.hpp"

/**
 * @brief Read fuel gauge measurements
 * @param val_array Output buffer
 * @return true if all reads successful
 */
bool get_FuelGauge_measurements(uint16_t* val_array, BQ34* fuel_gauge);

/**
 * @brief Read HVM measurements
 */
bool get_HVM_measurements(uint16_t* val_array, core::dev::BQ79631* hv_monitor);

/**
 * @brief Read slave board measurements
 */
bool get_Slave_measurements(uint16_t* val_array, core::dev::BQ79616* slave);

/**
 * @brief Read thermistor measurements
 */
bool get_Thermistors_measurements(int16_t* val_array, msd::bms::ThermistorArray* thermistor);

#endif // MSD_EVT_BMS_BMS_MEAS_HPP
