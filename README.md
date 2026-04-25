# MSD_BMS Firmware
Firmware for the Battery Management System (BMS) developed for RIT's Electric Vehicle Team (EVT).

This project implements a high-level BMS master controller responsible for:
- Monitoring battery health and operating conditions
- Managing communication with battery ICs
- Enforcing safety protections
- Coordinating system-level behavior

Built on top of the EVT-core library for embedded hardware abstraction. 

In order for this firmware to work with the EVT-core library the following lines must be 
changed in manager.hpp

```c++
void init() -> inline void init()

RTC& getRTC() -> inline RTC& getRTC()
```

---

# Repository Structure
- docs - Project documentation, diagrams, and reports
- inc - Header files 
- src - Source files 
- lib - External dependencies (EVT-core)
- tests - Test code and validation utilities
- build - Build output (generated)
- LICENSE 

---

# System Overview

The BMS firmware runs on an STM32 microcontroller and interfaces with multiple devices:

### Core Devices
- **BQ34Z100-R2** – Fuel Gauge (I2C)
- **BQ79600** – SPI Bridge (stack communication)
- **BQ79631** – High Voltage Monitor
- **M24C32** – EEPROM
- Thermistors – Temperature sensing via ADC
- CAN Transceiver – External communication (planned)

### Responsibilities
- Measurement acquisition (voltage, current, temperature)
- Safety monitoring and fault handling
- Device communication (I2C, SPI, UART, CAN)
- State machine control

---

# Getting Started

## 1. Clone the Repository
```
git clone <repo-url>
cd MSD_BMS
```
## 2. Setup Dependencies
Follow setup instructions in:
```
lib/EVT-core/README.md
```
This includes
- ARM GCC Toolchain
- CMake configuration
- HAL/CMSIS setup

# Build Instructions
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=lib/EVT-core/cmake/arm-gcc-toolchain.cmake
cmake --build build
```

This will generate the firmware binary in the `build/` directory.

# Flashing the Firmware
To flash the compiled firmware onto the target STM32 microcontroller, use OpenOCD, STM32CUBE or ST-Link.
Ensure the following:
- Correct target selected (STM32F446RE)
- Proper clock configuration
- Stable power supply
- Utilize BMS.cpp for testing


# Initialization Flow init()
The system performs:

1. Hardware Setup
   -	TIM2 configured for microsecond timing
   -	GPIO initialized (LEDs, control lines)

2. Communication Interfaces
   -	I2C (fuel gauge + EEPROM)
   -	SPI (BQ79600 stack)
   -	UART (debug)
   -	CAN (initialized, not fully used)

3. Device Initialization
   -	Fuel gauge (BQ34)
   -	EEPROM integrity check (write/read test)
   -	BQ79631 monitor setup
   
4. BQ79600 Bring-Up (Critical)
   -	GPIO-based wake sequence (not SPI)
   -	SPI communication validation
   -	Bridge initialization

5. Sensor Setup
   -	Thermistors initialized via ADC

# Update Loop Behavior 
Each update() cycle performs:

1. Measurements
   - Slave Device:
     - need to complete...
   - Fuel gauge:
     - Voltage
     - Current
     - Temperature
     - State of Charge (SOC)
     - Flags
   - High Voltage Monitor:
     -	Die temperature
     -	Fault flags
     -  Pack Voltage
   - Thermistors:
     - Individual readings + average

2. Protection Logic
   -	Fault detection via BQ34 flags
   -	Warning/fault classification

3. State Machine
   -	INIT → NORMAL
   -	NORMAL → steady operation
   -	WARNING → LED indication
   -	FAULT → system halt behavior
   -	SHUTDOWN → safe stop

# Key Engineering Notes

## Debug Mode
Enabled via: 
``` 
#define BMS_DEBUG
#define MESSAGE_DEBUG
```
_Heavy UART logging significantly slows execution and should only be used while debugging_

## First Bring-Up Checklist

_If the system is not functioning:_

Fuel Gauge (I2C)
- Confirm valid register reads

EEPROM
- Ensure write/read test passes

BQ79600 (SPI)
- Verify wake sequence timing
- Confirm DEV_CONF1 == 0x14

SPI
- Mode = MODE0
- Proper chip select handling

General
- Verify stable voltage rails