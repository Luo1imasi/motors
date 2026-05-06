#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/// EtherCAT frame abstraction (backend-specific layout).
/// Concrete backends (SOEM, IgH, etc.) map this to their native representation.
struct ethercat_frame {
    uint16_t slave_id;
    uint16_t index;
    uint8_t  sub_index;
    std::vector<uint8_t> data;
};

// Callback types shared by all EtherCAT backends
using EthercatCbkFunc = std::function<void(const ethercat_frame&)>;
using EthercatCbkId = uint16_t;
using EthercatCbkKeyExtractor = std::function<EthercatCbkId(const ethercat_frame&)>;

/**
 * @brief Abstract interface for EtherCAT communication backends.
 *
 * Concrete implementations:
 *   - MotorsSOEM: SOEM (Simple Open EtherCAT Master)
 *   - MotorsIgHEtherCAT: IgH EtherCAT Master
 *
 * All backends share the same transmit/callback API, allowing motor drivers
 * to switch transport transparently.
 */
class MotorsEtherCAT {
public:
    virtual ~MotorsEtherCAT() = default;

    /// Send an EtherCAT frame to a specific slave.
    virtual void transmit(const ethercat_frame& frame) = 0;

    /// Register a receive callback keyed by slave ID.
    virtual void add_ethercat_callback(const EthercatCbkFunc& callback, EthercatCbkId id) = 0;

    /// Remove a previously registered callback.
    virtual void remove_ethercat_callback(EthercatCbkId id) = 0;

    /// Remove all registered callbacks.
    virtual void clear_ethercat_callbacks() = 0;

    /// Customize the key extractor (default: frame.slave_id).
    virtual void set_ethercat_key_extractor(EthercatCbkKeyExtractor extractor) = 0;

    /**
     * @brief Factory: create or retrieve an EtherCAT backend instance.
     * @param interface  Interface name (e.g. "eth0")
     * @param backend    Backend type: "soem" (default) | "igh"
     */
    static std::shared_ptr<MotorsEtherCAT> get(
        const std::string& interface,
        const std::string& backend = "soem");

    static void init_logger(std::shared_ptr<spdlog::logger> logger);

protected:
    MotorsEtherCAT() = default;
};
