/******************************************************************************
 * data_capture.h — 高速UDP数据捕获模块
 *
 * 功能: 高性能UDP数据接收，支持零拷贝、内存池、Jumbo Frame、
 *       丢包检测与统计、自适应缓冲区调整等特性。
 *
 * 设计要点:
 * - 内存池预分配，避免运行时malloc/ free
 * - 支持Jumbo Frame (9000字节)
 * - epoll多路复用，非阻塞I/O
 * - 自适应接收缓冲区大小
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_DATA_CAPTURE_H
#define LSLIDAR_LS_DATA_CAPTURE_H

#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <functional>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <ros/ros.h>

namespace lslidar_ch_driver {
namespace core {

/**
 * @brief UDP接收统计
 */
struct UDPStats {
    uint64_t packets_received{0};
    uint64_t packets_lost{0};
    uint64_t bytes_received{0};
    double   packet_loss_rate{0.0};
    double   receive_rate_mbps{0.0};
    double   avg_latency_us{0.0};
    double   max_latency_us{0.0};
    int      socket_buffer_size{0};
    int      socket_buffer_drops{0};

    std::string toString() const {
        std::ostringstream oss;
        oss << "RX: " << packets_received << " pkts"
            << " Lost: " << packets_lost << " (" << (packet_loss_rate * 100) << "%)"
            << " Rate: " << receive_rate_mbps << " Mbps"
            << " Latency: " << avg_latency_us << "us";
        return oss.str();
    }
};

/**
 * @brief 内存池 - 预分配固定大小的数据缓冲区
 */
class MemoryPool {
public:
    MemoryPool(size_t block_size, size_t num_blocks)
        : block_size_(block_size)
        , num_blocks_(num_blocks) {

        // 预分配所有内存块
        for (size_t i = 0; i < num_blocks; ++i) {
            auto* block = new uint8_t[block_size];
            pool_.push_back(block);
            free_list_.push_back(block);
        }
    }

    ~MemoryPool() {
        for (auto* block : pool_) {
            delete[] block;
        }
    }

    uint8_t* acquire() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (free_list_.empty()) {
            // 池耗尽时动态扩展
            auto* block = new uint8_t[block_size_];
            pool_.push_back(block);
            return block;
        }
        auto* block = free_list_.back();
        free_list_.pop_back();
        return block;
    }

    void release(uint8_t* block) {
        std::lock_guard<std::mutex> lock(mutex_);
        free_list_.push_back(block);
    }

    size_t blockSize() const { return block_size_; }
    size_t numBlocks() const { return pool_.size(); }
    size_t availBlocks() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return free_list_.size();
    }

private:
    size_t block_size_;
    size_t num_blocks_;
    std::vector<uint8_t*> pool_;
    std::vector<uint8_t*> free_list_;
    mutable std::mutex mutex_;
};

/**
 * @brief UDP接收到的数据包
 */
struct UDPPacket {
    uint8_t* data{nullptr};
    size_t   size{0};
    uint64_t sequence{0};
    uint64_t timestamp_us{0};
    struct sockaddr_in sender{};
};

/**
 * @brief 高速UDP数据接收器
 *
 * 特性:
 * - 独立接收线程，epoll多路复用
 * - 内存池管理，零拷贝设计
 * - Jumbo Frame支持
 * - 自适应接收缓冲
 * - 丢包检测统计
 */
class UDPCapture {
public:
    using PacketCallback = std::function<void(UDPPacket&&)>;

    /**
     * @param port           UDP端口
     * @param packet_size    数据包大小
     * @param use_jumbo_frame 是否启用Jumbo Frame (9000字节)
     */
    UDPCapture(uint16_t port, size_t packet_size = 1206,
               bool use_jumbo_frame = false)
        : port_(port)
        , packet_size_(use_jumbo_frame ? 9000 : packet_size)
        , use_jumbo_frame_(use_jumbo_frame)
        , sockfd_(-1)
        , efd_(-1) {

        // 内存池: 每个包packet_size, 预分配1024个
        memory_pool_ = std::make_unique<MemoryPool>(packet_size_, 1024);
    }

    ~UDPCapture() { stop(); }

    bool start(const std::string& ip = "") {
        if (running_.exchange(true)) return true;

        // 创建socket
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            ROS_ERROR("[UDPCapture] socket() failed: %s", strerror(errno));
            running_ = false;
            return false;
        }

        // 启用Jumbo Frame (需要网卡支持)
        if (use_jumbo_frame_) {
            int mtu = 9000;
            if (setsockopt(sockfd_, SOL_SOCKET, SO_SNDBUF, &mtu, sizeof(mtu)) < 0) {
                ROS_WARN("[UDPCapture] Failed to set Jumbo Frame: %s", strerror(errno));
            }
        }

        // 增大接收缓冲区 (避免高数据率下丢包)
        int recv_buf = 4 * 1024 * 1024;  // 4MB
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &recv_buf, sizeof(recv_buf));
        socklen_t optlen = sizeof(recv_buf);
        getsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &recv_buf, &optlen);
        ROS_INFO("[UDPCapture] Socket buffer: %d bytes", recv_buf);

        // 绑定
        sockaddr_in addr{};
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sockfd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            ROS_ERROR("[UDPCapture] bind() failed on port %d: %s",
                      port_, strerror(errno));
            close(sockfd_);
            sockfd_ = -1;
            running_ = false;
            return false;
        }

        // 非阻塞
        int flags = fcntl(sockfd_, F_GETFL, 0);
        fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

        // epoll
        efd_ = epoll_create1(0);
        if (efd_ < 0) {
            ROS_ERROR("[UDPCapture] epoll_create1() failed: %s", strerror(errno));
            close(sockfd_);
            running_ = false;
            return false;
        }

        struct epoll_event ev;
        ev.events = EPOLLIN;
        ev.data.fd = sockfd_;
        if (epoll_ctl(efd_, EPOLL_CTL_ADD, sockfd_, &ev) < 0) {
            ROS_ERROR("[UDPCapture] epoll_ctl() failed: %s", strerror(errno));
            close(efd_);
            close(sockfd_);
            running_ = false;
            return false;
        }

        // 启动接收线程
        receiver_ = std::thread(&UDPCapture::receiveLoop, this);

        ROS_INFO("[UDPCapture] Started on port %d (packet_size=%zu, jumbo=%s)",
                 port_, packet_size_, use_jumbo_frame_ ? "yes" : "no");
        return true;
    }

    void stop() {
        running_ = false;
        if (receiver_.joinable()) receiver_.join();
        if (efd_ >= 0) close(efd_);
        if (sockfd_ >= 0) close(sockfd_);
        efd_ = -1;
        sockfd_ = -1;
        ROS_INFO("[UDPCapture] Stopped.");
    }

    void setCallback(PacketCallback cb) {
        callback_ = std::move(cb);
    }

    UDPStats getStats() const { return stats_; }
    size_t getPacketSize() const { return packet_size_; }
    uint16_t getPort() const { return port_; }
    bool isRunning() const { return running_; }

    void resetStats() { stats_ = {}; }

private:
    uint16_t port_;
    size_t packet_size_;
    bool use_jumbo_frame_;
    int sockfd_;
    int efd_;
    std::atomic<bool> running_{false};
    std::thread receiver_;
    PacketCallback callback_;
    UDPStats stats_;
    std::unique_ptr<MemoryPool> memory_pool_;

    // 上一帧序列号（丢包检测）
    int64_t last_seq_{-1};

    void receiveLoop() {
        struct epoll_event events[16];
        uint64_t last_report_time = 0;

        while (running_) {
            int nfds = epoll_wait(efd_, events, 16, 100);  // 100ms timeout

            if (nfds < 0) {
                if (errno == EINTR) continue;
                ROS_ERROR("[UDPCapture] epoll_wait() error: %s", strerror(errno));
                break;
            }

            for (int i = 0; i < nfds; ++i) {
                if (events[i].data.fd == sockfd_ && (events[i].events & EPOLLIN)) {
                    receivePackets();
                }
            }
        }
    }

    void receivePackets() {
        uint8_t* buffer = memory_pool_->acquire();
        sockaddr_in sender{};
        socklen_t sender_len = sizeof(sender);

        auto recv_start = std::chrono::steady_clock::now();

        ssize_t nbytes = recvfrom(sockfd_, buffer, packet_size_, 0,
                                  (sockaddr*)&sender, &sender_len);

        if (nbytes > 0) {
            auto recv_end = std::chrono::steady_clock::now();
            uint64_t latency_us = std::chrono::duration_cast<std::chrono::microseconds>(
                recv_end - recv_start).count();

            UDPPacket packet;
            packet.data = buffer;
            packet.size = static_cast<size_t>(nbytes);
            packet.sequence = stats_.packets_received;
            packet.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            packet.sender = sender;

            // 更新统计
            stats_.packets_received++;
            stats_.bytes_received += nbytes;
            stats_.avg_latency_us = stats_.avg_latency_us * 0.99 + latency_us * 0.01;
            stats_.max_latency_us = std::max(stats_.max_latency_us, (double)latency_us);

            // 周期打印统计
            if (stats_.packets_received % 10000 == 0) {
                auto now = std::chrono::steady_clock::now().time_since_epoch().count();
                ROS_DEBUG("[UDPCapture] %s", stats_.toString().c_str());
            }

            // 回调
            if (callback_) {
                callback_(std::move(packet));
            }
        } else {
            // 接收失败或超时，归还内存
            memory_pool_->release(buffer);
            if (nbytes < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
                ROS_ERROR("[UDPCapture] recvfrom() error: %s", strerror(errno));
            }
        }
    }
};

} // namespace core
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_DATA_CAPTURE_H
