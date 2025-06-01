/**
 * @file
 * This file implements functions to receive
 * and transmit CAN frames via SocketCAN.
 */

#include "SocketCAN.hpp"

std::shared_ptr<spdlog::logger> SocketCAN::logger_ = nullptr;
std::unordered_map<std::string, std::shared_ptr<SocketCAN>> SocketCAN::instances_;

SocketCAN::SocketCAN(std::string interface)
    : interface_(interface), sockfd_(INIT_FD), receiving_(false), tx_queue_(TX_QUEUE_SIZE) {
    open(interface);
}

SocketCAN::~SocketCAN() { this->close(); }

void SocketCAN::open(std::string interface) {
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ == INIT_FD) {
        logger_->error("Failed to create CAN socket");
        return;
    }

    strncpy(if_request_.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(sockfd_, SIOCGIFINDEX, &if_request_) == -1) {
        logger_->error("Unable to detect CAN interface {}", interface);

        this->close();
        return;
    }

    // Bind the socket to the network interface
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = if_request_.ifr_ifindex;
    int rc = ::bind(sockfd_, reinterpret_cast<struct sockaddr *>(&addr_), sizeof(addr_));
    if (rc == -1) {
        logger_->error("Failed to bind socket to network interface {}", interface);
        this->close();
        return;
    }

    receiving_ = true;
    receiver_thread_ = std::thread([this]() {
        fd_set descriptors;
        int maxfd = sockfd_;
        struct timeval timeout;
        can_frame rx_frame;

        while (receiving_) {
            FD_ZERO(&descriptors);
            FD_SET(sockfd_, &descriptors);

            timeout.tv_sec = TIMEOUT_SEC;
            timeout.tv_usec = TIMEOUT_USEC;

            if (::select(maxfd + 1, &descriptors, NULL, NULL, &timeout) == 1) {
                int len = ::read(sockfd_, &rx_frame, CAN_MTU);
                if (len < 0) {
                    continue;
                }
                auto it = can_callback_list_.find((CanCbkId)(rx_frame.can_id));
                if (it != can_callback_list_.end()) {
                    std::lock_guard<std::mutex> lock(can_callback_mutex_);
                    it->second(rx_frame);
                }
            }
        }
    });

    sender_thread_ = std::thread([this]() {
        can_frame tx_frame;
        int count = 0;
        while (receiving_) {
            {
                std::unique_lock<std::mutex> lock(tx_mutex_);
                tx_cv_.wait(lock, [this]() { return !tx_queue_.empty() || !receiving_; });
                if (!receiving_) break;
                if (!tx_queue_.pop(tx_frame)) continue;
            }
            while (!::write(sockfd_, &tx_frame, sizeof(can_frame)) && count < MAX_RETRY_COUNT) {
                count += 1;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 避免忙等待
            }
            if (count >= MAX_RETRY_COUNT) logger_->error("Failed to transmit CAN frame");
            count = 0;
        }
    });
}

void SocketCAN::close() {
    receiving_ = false;
    tx_cv_.notify_one();
    if (receiver_thread_.joinable()) receiver_thread_.join();
    if (sender_thread_.joinable()) sender_thread_.join();

    if (sockfd_ != INIT_FD) ::close(sockfd_);
    sockfd_ = INIT_FD;
}

void SocketCAN::transmit(const can_frame &frame) {
    if (sockfd_ == INIT_FD) {
        logger_->error("Unable to transmit: Socket not open");
        return;
    }
    tx_queue_.bounded_push(frame);
    tx_cv_.notify_one();
}

void SocketCAN::add_can_callback(const CanCbkFunc callback, const CanCbkId id) {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    can_callback_list_[id] = callback;
}

void SocketCAN::remove_can_callback(CanCbkId id) {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    can_callback_list_.erase(id);
}

void SocketCAN::clear_can_callbacks() {
    std::lock_guard<std::mutex> lock(can_callback_mutex_);
    can_callback_list_.clear();
}
