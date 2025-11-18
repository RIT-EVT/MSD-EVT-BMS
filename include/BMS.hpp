#ifndef BMS_H
#define BMS_H

#ifdef __cplusplus
extern "C" {
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

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <memory>

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

private:
    bool initialized_ = false;
};

} // namespace msd::bms

