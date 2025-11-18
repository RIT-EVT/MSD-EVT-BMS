# BMS Revamp
Repository for the firmware &amp; software for the Battery Management System for the EVT

# Repository Structure
- docs - Documentation for the project
- inc - Header files for the project
- lib - External libraries used by the project
- LICENSE - License information for the project
- src - Source files for the project
- tests - Test files for the project

# Getting Started
To get started with the project, clone the repository and navigate to the project directory. Follow the
instructions in the `lib/EVT-core/README.md` file to set up the EVT-core library, which is a dependency for this project.
Then, follow the build instructions below to compile the firmware.

## Build Instructions
The project uses CMake as the build system. Ensure you have CMake installed on your system
and the GCC ARM toolchain set up as described in the `lib/EVT-core/README.md`.
To build the project, run the following commands from the project root directory:
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=lib/EVT-core/cmake/arm-gcc-toolchain.cmake
cmake --build build
```
This will generate the firmware binary in the `build/` directory.

## Flashing the Firmware
To flash the compiled firmware onto the target STM32 microcontroller, use OpenOCD, STM32CUBE or ST-Link.
Refer to the documentation for your specific hardware for instructions on how to flash the firmware.

## Testing
The project includes test files located in the `tests/` directory. To run the tests, navigate to the `tests/` directory and follow the instructions in the `README.md` file located there.

# Documentation
For detailed documentation on the architecture, components, and workflows of the project, refer to the `docs/` directory. This includes information on the core library, BMS components, critical workflows, key integration points, and project-specific conventions.

# Overview
The MSD_BMS repository contains firmware for a Battery Management System developed for RIT's Electric Vehicle Team (EVT). The system is built on top of the EVT-core library which provides MCU abstractions for embedded development.
