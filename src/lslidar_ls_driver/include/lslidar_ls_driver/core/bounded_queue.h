#ifndef LSLIDAR_BOUNDED_QUEUE_H
#define LSLIDAR_BOUNDED_QUEUE_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <queue>
#include <utility>

namespace lslidar_ch_driver {

enum class OverflowPolicy { Block, RejectNewest, DropOldest };

template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity = 64,
                          OverflowPolicy policy = OverflowPolicy::Block)
        : capacity_(capacity == 0 ? 1 : capacity), policy_(policy) {}

    bool push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (policy_ == OverflowPolicy::Block) {
            not_full_.wait(lock, [this] {
                return queue_.size() < capacity_ || shutdown_;
            });
        } else if (queue_.size() >= capacity_) {
            if (policy_ == OverflowPolicy::RejectNewest) {
                ++dropped_;
                return false;
            }
            queue_.pop();
            ++dropped_;
        }
        if (shutdown_) return false;
        queue_.push(std::move(item));
        not_empty_.notify_one();
        return true;
    }

    bool pop(T& item, int timeout_ms = -1) {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto ready = [this] { return !queue_.empty() || shutdown_; };
        if (timeout_ms >= 0) {
            if (!not_empty_.wait_for(lock, std::chrono::milliseconds(timeout_ms), ready)) {
                return false;
            }
        } else {
            not_empty_.wait(lock, ready);
        }
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        not_full_.notify_one();
        return true;
    }

    void shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        not_empty_.notify_all();
        not_full_.notify_all();
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) queue_.pop();
        shutdown_ = false;
        dropped_ = 0;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    std::size_t dropped() const { return dropped_.load(); }

private:
    mutable std::mutex mutex_;
    std::condition_variable not_full_;
    std::condition_variable not_empty_;
    std::queue<T> queue_;
    const std::size_t capacity_;
    const OverflowPolicy policy_;
    bool shutdown_{false};
    std::atomic<std::size_t> dropped_{0};
};

}  // namespace lslidar_ch_driver
#endif
