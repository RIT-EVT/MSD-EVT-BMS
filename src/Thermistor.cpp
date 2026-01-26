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
        readings_[i] = {0, Fault::ADC_FAULT};
    }
}

void ThermistorArray::update() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        uint32_t adc = adcs_[i]->read();

        Fault fault = detectFault(adc);
        readings_[i].fault = fault;

        if (fault != Fault::NONE) {
            readings_[i].temperature_dC = 0;
            continue;
        }

        float v = adcToVoltage(adc);
        float r = voltageToResistance(v);
        readings_[i].temperature_dC = resistanceToTemperature(r, i);
    }
}

ThermistorArray::Reading ThermistorArray::getSensor(uint8_t index) const {
    if (index >= SENSOR_COUNT) {
        return {0, Fault::ADC_FAULT};
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

float ThermistorArray::adcToVoltage(uint32_t adc) const {
    return (static_cast<float>(adc) / ADC_MAX) * VREF;
}

float ThermistorArray::voltageToResistance(float v) const {
    return PULLUP_OHMS * (v / (VREF - v));
}

int16_t ThermistorArray::resistanceToTemperature(float r, uint8_t index) const {
    const auto& c = coeffs_[index];

    float lnR = logf(r);
    float invT = c.a + c.b * lnR + c.c * lnR * lnR * lnR;
    float tempC = (1.0f / invT) - 273.15f;

    return static_cast<int16_t>(tempC * 10.0f);
}

void ThermistorArray::loadCoefficients(core::dev::M24C32& eeprom, uint16_t base_addr) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        uint16_t addr = base_addr + (i * 12);

        eeprom.readBytes(addr + 0,  reinterpret_cast<uint8_t*>(&coeffs_[i].a), 4);
        eeprom.readBytes(addr + 4,  reinterpret_cast<uint8_t*>(&coeffs_[i].b), 4);
        eeprom.readBytes(addr + 8,  reinterpret_cast<uint8_t*>(&coeffs_[i].c), 4);
    }
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