#pragma once

#include <linux/can.h>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// Callback types shared by all CANFD backends
using CanFdCbkFunc = std::function<void(const canfd_frame&)>;
using CanFdCbkId = uint16_t;
using CanFdCbkKeyExtractor = std::function<CanFdCbkId(const canfd_frame&)>;
using CanFdCbkMap = std::unordered_map<CanFdCbkId, CanFdCbkFunc>;

class MotorsSocketCANFD;

/**
 * @brief Abstract interface for CANFD communication backends.
 *
 * Concrete implementations:
 *   - MotorsSocketCANFD: Linux SocketCAN (direct CANFD)
 *   - MotorsEthercatCANFD: EtherCAT-to-CANFD bridge
 *
 * All backends share the same transmit/callback API, allowing motor drivers
 * to switch transport transparently.
 */
class MotorsCANFD {
public:
    virtual ~MotorsCANFD() = default;

    /// Send a CANFD frame.
    virtual void transmit(const canfd_frame& frame) = 0;

    /// Register a receive callback keyed by CAN ID.
    virtual void add_canfd_callback(const CanFdCbkFunc& callback, CanFdCbkId id) = 0;

    /// Remove a previously registered callback.
    virtual void remove_canfd_callback(CanFdCbkId id) = 0;

    /// Remove all registered callbacks.
    virtual void clear_canfd_callbacks() = 0;

    /// Customize the key extractor (default: frame.can_id).
    virtual void set_canfd_key_extractor(CanFdCbkKeyExtractor extractor) = 0;

    /**
     * @brief Factory: create or retrieve a CANFD backend instance.
     * @param interface  Interface name (e.g. "can0", or EtherCAT master name)
     * @param backend   Backend type: "socketcan" (default) | "ethercat" | "shm"
     */
    static std::shared_ptr<MotorsCANFD> get(
        const std::string& interface,
        const std::string& backend = "socketcan");

    static void init_logger(std::shared_ptr<spdlog::logger> logger);

protected:
    MotorsCANFD() = default;
    inline static std::shared_ptr<spdlog::logger> logger_ = nullptr;

    inline static void ensure_logger() {
        if (!logger_) {
            logger_ = spdlog::get("motors");
            if (!logger_) {
                std::vector<spdlog::sink_ptr> sinks;
                sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
                logger_ = std::make_shared<spdlog::logger>("motors", std::begin(sinks), std::end(sinks));
                spdlog::register_logger(logger_);
            }
        }
    }
};

inline void MotorsCANFD::init_logger(std::shared_ptr<spdlog::logger> logger) { logger_ = logger; }
