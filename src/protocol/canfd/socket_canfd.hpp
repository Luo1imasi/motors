/**
 * @file
 * This file declares the SocketCAN implementation of the CANFD interface.
 * Direct CANFD via Linux SocketCAN.
 */

#pragma once

#include "canfd_iso.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <atomic>
#include <boost/lockfree/queue.hpp>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

constexpr const int FD_INIT_FD = -1;
constexpr const int FD_TIMEOUT_SEC = 0;
constexpr const int FD_TIMEOUT_USEC = 1000;
constexpr const int FD_TX_QUEUE_SIZE = 4096;
constexpr const int FD_MAX_RETRY_COUNT = 3;

using LFQueueFD = boost::lockfree::queue<canfd_frame, boost::lockfree::fixed_sized<true>>;

class MotorsSocketCANFD : public MotorsCANFD {
   private:
    std::string interface_;
    int sockfd_ = -1;
    std::atomic<bool> receiving_;
    LFQueueFD tx_queue_;
    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;

    sockaddr_can addr_;
    ifreq if_request_;

    /// Receiving
    std::thread receiver_thread_;
    CanFdCbkMap canfd_callback_list_;
    std::mutex canfd_callback_mutex_;
    CanFdCbkKeyExtractor key_extractor_ = [](const canfd_frame& frame) -> CanFdCbkId {
        return static_cast<CanFdCbkId>(frame.can_id);
    };

    /// Transmitting
    std::thread sender_thread_;
    std::atomic<int> send_sleep_us_{0};

    MotorsSocketCANFD(const std::string& interface);

    static std::shared_ptr<MotorsSocketCANFD> createInstance(const std::string& interface) {
        return std::shared_ptr<MotorsSocketCANFD>(new MotorsSocketCANFD(interface));
    }
    static std::unordered_map<std::string, std::shared_ptr<MotorsSocketCANFD>> instances_;

    void open(const std::string& interface);
    void close();

   public:
    MotorsSocketCANFD(const MotorsSocketCANFD&) = delete;
    MotorsSocketCANFD& operator=(const MotorsSocketCANFD&) = delete;
    ~MotorsSocketCANFD() override;

    static std::shared_ptr<MotorsSocketCANFD> get(const std::string& interface);

    void transmit(const canfd_frame& frame) override;
    void add_canfd_callback(const CanFdCbkFunc& callback, CanFdCbkId id) override;
    void remove_canfd_callback(CanFdCbkId id) override;
    void clear_canfd_callbacks() override;
    void set_canfd_key_extractor(CanFdCbkKeyExtractor extractor) override;
    void set_send_sleep(int us) { send_sleep_us_ = us; }

    /// SocketCAN-specific: direct socket fd access (for advanced use).
    int get_fd() const { return sockfd_; }
};
