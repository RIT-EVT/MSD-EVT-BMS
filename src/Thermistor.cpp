//
// Created by Key'Mon Jenkins on 1/25/26.
//

#include "Thermistor.hpp"
#include <cmath>
#include <limits>

namespace msd::bms {

ThermistorArray::ThermistorArray(core::io::ADC* adcs[SENSOR_COUNT]) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        adcs_[i] = adcs[i];
        readings_[i] = {0, 0, Fault::ADC_FAULT};
    }
}

ThermistorArray::Reading ThermistorArray::getSensor(uint8_t index) const {
    if (index >= SENSOR_COUNT) {
        return {0, 0, Fault::ADC_FAULT};
    }
    return readings_[index];
}

int16_t ThermistorArray::getAverage() const {
    int32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (readings_[i].fault == Fault::NONE) {
            sum += readings_[i].temperature_dC;
            ++count;
        }
    }

    if (count == 0) {
        return std::numeric_limits<int16_t>::min();
    }

    return static_cast<int16_t>(sum / count);
}

// ======================
// Conversion helpers
// ======================

// Need to check logic to get actual temperature values!!!


float ThermistorArray::thermistorBetaTemp(uint32_t adc,float vref,uint32_t adc_max,float r_pullup) {
    if (adc == 0 || adc >= adc_max) {
        return NAN; // open or short
    }

    float v = (static_cast<float>(adc) * vref) / static_cast<float>(adc_max);
    float r = r_pullup * v / (vref - v);

    constexpr float B  = 3435.0f;
    constexpr float R0 = 10000.0f;
    constexpr float T0 = 298.15f;

    float invT = (1.0f / T0) + (1.0f / B) * logf(r / R0);
    float T = 1.0f / invT;

    return T - 273.15f;
}


void ThermistorArray::update() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        readings_[i].adc_raw = adcs_[i]->readRaw();
        readings_[i].fault = detectFault(readings_[i].adc_raw);
        readings_[i].temperature_dC = thermistorBetaTemp(readings_[i].adc_raw, 3.3f, 4095, 10000.0f);
    }
    //     uint32_t adc = adcs_[i]->read();
    //
    //     Fault fault = detectFault(adc);
    //     readings_[i].fault = fault;
    //
    //     if (fault != Fault::NONE) {
    //         readings_[i].temperature_dC = 0;
    //         continue;
    //     }
    //
    //     float v = adcToVoltage(adc);
    //     float r = voltageToResistance(v);
    //     readings_[i].temperature_dC = resistanceToTemperature(r, i);
    // }
}

// ======================
// Fault detection
// ======================

ThermistorArray::Fault ThermistorArray::detectFault(uint32_t adc) const {
    if (adc == 0) {
        return Fault::SHORT_CIRCUIT;
    }

    if (adc >= ADC_MAX) {
        return Fault::OPEN_CIRCUIT;
    }

    return Fault::NONE;
}

} // namespace msd::bms