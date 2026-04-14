/**
@file BMS_meas.cpp
@brief C++ file for reading device measurements and reporting back to BMS.cpp.

This file defines functions to read device measurements for the use of BMS.cpp

Project: MSD_BMS
Author: Key'Mon Jenkins
Date: April 2026
*/

#include "dev/BMS_meas.hpp"


bool get_FuelGauge_measurements(uint16_t* v, BQ34* fuel_gauge) {
    if (!fuel_gauge || !v) return false;

    bool ok = true;

    ok &= fuel_gauge->getVoltage(v[0]);       // mV
    ok &= fuel_gauge->getTemperature(v[1]);   // 0.1K
    ok &= fuel_gauge->getCurrent(v[2]);       // mA
    ok &= fuel_gauge->getSOC(v[3]);           // %
    ok &= fuel_gauge->getMaxError(v[4]);      // %
    ok &= fuel_gauge->getVoltageRaw(v[5]);    // raw mV
    ok &= fuel_gauge->getFlags(v[6]);         // flags

    return ok;
}

bool get_HVM_measurements(uint16_t* v, core::dev::BQ79631* hv_monitor) {
    if (!hv_monitor || !v) return false;

    bool ok = true;

    ok &= hv_monitor->getPackVoltage_mV(v[0]);
    ok &= hv_monitor->getDieTemperature_cC(v[1]); // centi-C
    ok &= hv_monitor->getFaultFlags(v[2]);
    ok &= hv_monitor->getCellCount(v[3]);

    return ok;
}

bool get_Slave_measurements(uint16_t* v, core::dev::BQ79616* slave) {
    if (!slave || !v) return false;

    // TODO: implement when daisy chain fully working
    // Example placeholders
    v[0] = 0;
    v[1] = 0;

    return true;
}

bool get_Thermistors_measurements(int16_t* v, msd::bms::ThermistorArray* thermistors) {
    if (!thermistors || !v) return false;

    thermistors->update();

    for (uint8_t i = 0; i < 2; i++) {
        auto [adc_raw, temp_dC, fault] = thermistors->getSensor(i);

        v[i] = temp_dC; // store deci-C directly
    }

    v[2] = thermistors->getAverage(); // deci-C

    return true;
}