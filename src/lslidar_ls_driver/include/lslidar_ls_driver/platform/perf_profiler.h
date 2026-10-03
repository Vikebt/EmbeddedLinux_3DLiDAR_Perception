/******************************************************************************
 * perf_profiler.h — 性能分析器
 *
 * 功能: 提供轻量级的性能采样、延迟分析和置信报告生成，
 *       用于识别流水线瓶颈，量化各处理阶段的耗时。
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_PERF_PROFILER_H
#define LSLIDAR_LS_PERF_PROFILER_H

#include <string>
#include <unordered_map>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <ros/ros.h>

namespace lslidar_ch_driver {
namespace platform {

/**
 * @brief 单次采样数据
 */
struct SampleData {
    std::string  label;
    double       elapsed_ms{0.0};
    size_t       data_size{0};     // 处理的数据量（点数/字节数等）
};

/**
 * @brief 统计信息
 */
struct StatInfo {
    std::string label;
    size_t      count{0};
    double      total_ms{0.0};
    double      avg_ms{0.0};
    double      min_ms{1e9};
    double      max_ms{0.0};
    double      p50_ms{0.0};
    double      p95_ms{0.0};
    double      p99_ms{0.0};
    double      stddev_ms{0.0};
    double      throughput_hz{0.0};   // 吞吐量 (Hz)
    double      data_rate_mbps{0.0};  // 数据率 (MB/s)

    std::string toString() const {
        std::ostringstream oss;
        oss << label << ": count=" << count
            << " avg=" << avg_ms << "ms"
            << " min=" << min_ms << "ms"
            << " max=" << max_ms << "ms"
            << " p99=" << p99_ms << "ms"
            << " " << throughput_hz << "Hz";
        return oss.str();
    }
};

/**
 * @brief 计时器RAII包装
 *
 * 在作用域内自动计时，退出时记录耗时。
 */
class ScopedTimer {
public:
    ScopedTimer(const std::string& label, size_t data_size = 0)
        : label_(label), data_size_(data_size),
          start_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer();

private:
    std::string label_;
    size_t data_size_;
    std::chrono::steady_clock::time_point start_;
};

/**
 * @brief 性能分析器（单例）
 */
class ProfilerManager {
public:
    static ProfilerManager& instance() {
        static ProfilerManager inst;
        return inst;
    }

    void recordSample(const std::string& label, double elapsed_ms,
                      size_t data_size = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& samples = samples_[label];
        samples.push_back({label, elapsed_ms, data_size});
        if (samples.size() > max_samples_) {
            samples.erase(samples.begin());
        }
    }

    std::vector<StatInfo> getStats() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<StatInfo> stats;

        for (const auto& [label, samples] : samples_) {
            if (samples.empty()) continue;

            StatInfo info;
            info.label = label;
            info.count = samples.size();

            std::vector<double> times;
            times.reserve(samples.size());
            for (const auto& s : samples) {
                info.total_ms += s.elapsed_ms;
                info.min_ms = std::min(info.min_ms, s.elapsed_ms);
                info.max_ms = std::max(info.max_ms, s.elapsed_ms);
                times.push_back(s.elapsed_ms);
            }
            info.avg_ms = info.total_ms / info.count;

            // 标准差
            double variance = 0.0;
            for (double t : times) {
                variance += (t - info.avg_ms) * (t - info.avg_ms);
            }
            info.stddev_ms = std::sqrt(variance / info.count);
            info.throughput_hz = info.avg_ms > 0 ? 1000.0 / info.avg_ms : 0.0;

            // 百分位
            std::sort(times.begin(), times.end());
            info.p50_ms = times[static_cast<size_t>(times.size() * 0.50)];
            info.p95_ms = times[static_cast<size_t>(times.size() * 0.95)];
            info.p99_ms = times[static_cast<size_t>(times.size() * 0.99)];

            // 数据率
            double total_bytes = 0;
            for (const auto& s : samples) {
                total_bytes += s.data_size;
            }
            info.data_rate_mbps = total_bytes / (info.total_ms * 125.0);  // bytes/ms to Mbps

            stats.push_back(info);
        }
        return stats;
    }

    void printReport() {
        auto stats = getStats();
        ROS_INFO("========== Performance Profiler Report ==========");
        ROS_INFO("%-25s %8s %8s %8s %8s %8s %10s",
                 "Stage", "Avg(ms)", "Min(ms)", "Max(ms)", "P99(ms)", "Count", "Hz");
        ROS_INFO("--------------------------------------------------------");
        for (const auto& s : stats) {
            ROS_INFO("%-25s %8.2f %8.2f %8.2f %8.2f %8zu %10.1f",
                     s.label.c_str(), s.avg_ms, s.min_ms, s.max_ms,
                     s.p99_ms, s.count, s.throughput_hz);
        }
        ROS_INFO("========================================================");
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        samples_.clear();
    }

    void setMaxSamples(size_t n) { max_samples_ = n; }

private:
    ProfilerManager() = default;
    ~ProfilerManager() = default;
    ProfilerManager(const ProfilerManager&) = delete;
    ProfilerManager& operator=(const ProfilerManager&) = delete;

    mutable std::mutex mutex_;
    size_t max_samples_{10000};
    std::unordered_map<std::string, std::vector<SampleData>> samples_;
};

inline ScopedTimer::~ScopedTimer() {
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_).count();
    ProfilerManager::instance().recordSample(label_, elapsed, data_size_);
}

// 方便使用的宏
#define PROFILE_SCOPE(name) \
    lslidar_ch_driver::platform::ScopedTimer _prof_##__LINE__(name)

#define PROFILE_SCOPE_DATA(name, size) \
    lslidar_ch_driver::platform::ScopedTimer _prof_##__LINE__(name, size)

} // namespace platform
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_PERF_PROFILER_H
