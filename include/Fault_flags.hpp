#pragma once

/**
 * @file fault_flags.hpp
 * @brief C++ header for fault flag management for the Battery Management System (BMS).
 *
 *
 * Project: MSD_BMS
 * Author: Key'Mon Jenkins
 * Date: October 2025
 */

#include <cstdint>
#include <atomic>

/// Fault types for the Battery Management System (bitmaskable)
enum class FaultType : uint32_t {
    FAULT_NONE            = 0x00,
    FAULT_OVERVOLTAGE     = 0x01,
    FAULT_UNDERVOLTAGE    = 0x02,
    FAULT_OVERTEMPERATURE = 0x04,
    FAULT_COMMUNICATION   = 0x08,
    FAULT_HARDWARE        = 0x10
};

// Allow bitwise operations on FaultType for convenience
inline constexpr FaultType operator|(FaultType a, FaultType b) noexcept {
    return static_cast<FaultType>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline constexpr FaultType operator&(FaultType a, FaultType b) noexcept {
    return static_cast<FaultType>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
inline constexpr FaultType operator~(FaultType a) noexcept {
    return static_cast<FaultType>(~static_cast<uint32_t>(a));
}
inline FaultType& operator|=(FaultType& a, FaultType b) noexcept {
    a = a | b;
    return a;
}
inline FaultType& operator&=(FaultType& a, FaultType b) noexcept {
    a = a & b;
    return a;
}

// Global atomic fault flag register (header-only inline variable, C++17+)
inline std::atomic<uint32_t> fault_flags{0u};

/**
 * @brief Raise (set) the specified fault flag.
 *
 * @param fault The fault type to raise.
 */
inline void fault_flags_raise(FaultType fault) noexcept {
    fault_flags.fetch_or(static_cast<uint32_t>(fault), std::memory_order_relaxed);
}

/**
 * @brief Clear (reset) the specified fault flag.
 *
 * @param fault The fault type to clear.
 */
inline void fault_flags_clear(FaultType fault) noexcept {
    fault_flags.fetch_and(~static_cast<uint32_t>(fault), std::memory_order_relaxed);
}

/**
 * @brief Get the current fault flags.
 *
 * @return uint32_t The current fault flags.
 */
inline uint32_t fault_flags_get() noexcept {
    return fault_flags.load(std::memory_order_relaxed);
}

/**
 * @brief Reset all fault flags.
 */
inline void fault_flags_reset() noexcept {
    fault_flags.store(0u, std::memory_order_relaxed);
}