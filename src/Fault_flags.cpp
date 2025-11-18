/**
 * @file fault_flags.cpp
 * @brief Implementation of fault flag management for the Battery Management System (BMS).
 *
 * This file contains functions to raise, clear, get, and reset fault flags.
 * @file fault_flags.cpp
 * @brief Implementation of fault flag management for the Battery Management System (BMS).
 *
 * This file contains functions to raise, clear, get, and reset fault flags.
 * Project: MSD_BMS
 * Author: Key'Mon Jenkins
 * Date: October 2025
 */

#include <Fault_flags.hpp>
#include <iostream>

/* Global fault flag register */
volatile uint32_t fault_flags = 0;

/**
 * @brief Raise (set) the specified fault flag.
 *
 * @param fault The fault type to raise.
 */
void fault_flags_raise(uint32_t fault) {
    fault_flags |= fault;
    std::cout << "fault_flags_raise: Raised fault 0x" << std::hex << fault << ", new fault_flags = 0x" << std::hex << fault_flags << std::endl;
}

/**
 * @brief Clear (reset) the specified fault flag.
 *
 * @param fault The fault type to clear.
 */
void fault_flags_clear(uint32_t fault) {
    fault_flags &= ~fault;
    std::cout << "fault_flags_clear: Cleared fault 0x" << std::hex << fault << ", new fault_flags = 0x" << std::hex << fault_flags << std::endl;
}

/**
 * @brief Get the current fault flags.
 *
 * @return uint32_t The current fault flags.
 */
uint32_t fault_flags_get(void) {
    std::cout << "fault_flags_get: Current fault_flags = 0x" << std::hex << fault_flags << std::endl;
    return fault_flags;
}

/**
 * @brief Reset all fault flags.
 */
void fault_flags_reset(void) {
    fault_flags = 0;
    std::cout << "fault_flags_reset: All fault flags reset to 0" << std::endl;
}

