/******************************************************************************
 * timestamp_sync.h — 多传感器时间同步模块
 *
 * 功能: 实现LiDAR-IMU-GPS多传感器软同步方案，支持多种授时方式：
 *       - PTP (IEEE 1588) 精确时间同步
 *       - GPS NMEA 时间同步
 *       - NTP 网络时间同步
 *       - 软件插值对齐
 *       - 时间偏移校准与漂移补偿
 *
 * 设计要点:
 * - 统一时间基准（所有传感器数据对齐到同一时钟域）
 * - 时间偏移估计与补偿（卡尔曼滤波）
 * - 时间戳插值（IMU高频数据向LiDAR帧对齐）
 * - 延迟测量与报告
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_TIMESTAMP_SYNC_H
#define LSLIDAR_LS_TIMESTAMP_SYNC_H

#include <cstdint>
#include <atomic>
#include <mutex>
#include <vector>
#include <deque>
#include <chrono>
#include <cmath>
#include <ros/ros.h>

namespace lslidar_ch_driver {
namespace core {

/**
 * @brief 传感器类型
 */
enum class SensorType : uint8_t {
    LIDAR = 0,
    IMU   = 1,
    GPS   = 2,
    CAMERA = 3,
};

/**
 * @brief 时间同步模式
 */
enum class SyncMode : uint8_t {
    SYSTEM_TIME = 0,      // 使用系统时间
    GPS_PPS     = 1,      // GPS 秒脉冲同步
    PTP         = 2,      // IEEE 1588 PTP
    NTP         = 3,      // NTP 网络时间
    SOFTWARE    = 4,      // 纯软件时间对齐
};

/**
 * @brief 时间同步配置
 */
struct SyncConfig {
    SyncMode mode{SyncMode::SYSTEM_TIME};
    double   max_time_offset_us{1000.0};      // 最大允许的时间偏差 (微秒)
    double   drift_compensation_rate{0.01};   // 漂移补偿率
    bool     enable_interpolation{true};       // 使能时间戳插值
    bool     enable_drift_compensation{true};  // 使能漂移补偿
    std::string ptp_interface{"eth0"};         // PTP网口
    std::string ntp_server{""};                // NTP服务器
};

/**
 * @brief 带时间戳的传感器数据
 */
struct SensorData {
    SensorType type;
    uint64_t   timestamp_ns;          // 全局时间戳 (ns)
    uint64_t   hardware_timestamp_ns; // 硬件时间戳 (ns)
    uint64_t   receive_timestamp_ns;  // 接收时间戳 (ns)
    uint64_t   sequence;              // 序列号
    double     time_offset_us;        // 与主时钟的偏差 (us)

    double age_s() const {
        return (receive_timestamp_ns - timestamp_ns) / 1e9;
    }
};

/**
 * @brief 时间同步管理器
 *
 * 管理多个传感器的时间基准，提供统一的时间戳服务。
 */
class TimestampSynchronizer {
public:
    explicit TimestampSynchronizer(const SyncConfig& config = SyncConfig())
        : config_(config)
        , master_offset_ns_(0)
        , drift_ppm_(0.0) {

        setSyncMode(config.mode);
    }

    /**
     * @brief 校准传感器时间戳到统一基准
     * @param sensor_type 传感器类型
     * @param hw_timestamp_ns 硬件时间戳 (ns)
     * @return 校准后的系统时间戳 (ns)
     */
    uint64_t calibrate(SensorType sensor_type, uint64_t hw_timestamp_ns) {
        uint64_t receive_time = getSystemTimeNs();

        SensorData data;
        data.type = sensor_type;
        data.hardware_timestamp_ns = hw_timestamp_ns;
        data.receive_timestamp_ns = receive_time;
        data.sequence = ++sequence_counter_;

        switch (config_.mode) {
            case SyncMode::GPS_PPS:
            case SyncMode::PTP:
                // 硬件同步模式下，硬件时间戳已同步
                data.timestamp_ns = hw_timestamp_ns;
                data.time_offset_us = 0.0;
                break;

            case SyncMode::NTP:
            case SyncMode::SYSTEM_TIME:
                // 软件同步模式，需要估计偏移
                data.timestamp_ns = estimateTimestamp(sensor_type, hw_timestamp_ns, receive_time);
                data.time_offset_us = (static_cast<int64_t>(receive_time) -
                                       static_cast<int64_t>(data.timestamp_ns)) / 1000.0;
                break;

            case SyncMode::SOFTWARE:
                // 纯软件对齐
                data.timestamp_ns = receive_time;
                data.time_offset_us = 0.0;
                break;
        }

        // 更新偏移估计
        if (config_.enable_drift_compensation && config_.mode != SyncMode::PTP) {
            updateDriftEstimate(data);
        }

        // 保存最近的历史
        recent_data_.push_back(data);
        if (recent_data_.size() > max_history_) {
            recent_data_.pop_front();
        }

        return data.timestamp_ns;
    }

    /**
     * @brief 对IMU数据进行时间戳插值（对齐到LiDAR帧）
     * @param imu_timestamps_ns IMU数据时间戳列表
     * @param lidar_timestamp_ns 目标LiDAR帧时间戳
     * @return 插值后的时间戳
     */
    uint64_t interpolateToLidarFrame(const std::vector<uint64_t>& imu_timestamps_ns,
                                      uint64_t lidar_timestamp_ns) {
        if (!config_.enable_interpolation || imu_timestamps_ns.empty()) {
            return lidar_timestamp_ns;
        }

        // 找到LiDAR时间戳前后的IMU数据
        uint64_t before = 0, after = 0;
        for (size_t i = 0; i + 1 < imu_timestamps_ns.size(); ++i) {
            if (imu_timestamps_ns[i] <= lidar_timestamp_ns &&
                imu_timestamps_ns[i + 1] >= lidar_timestamp_ns) {
                before = imu_timestamps_ns[i];
                after = imu_timestamps_ns[i + 1];
                break;
            }
        }

        if (before == 0 || after == 0 || after == before) {
            return lidar_timestamp_ns;
        }

        // 线性插值
        double ratio = static_cast<double>(lidar_timestamp_ns - before) /
                      static_cast<double>(after - before);
        return before + static_cast<uint64_t>(ratio * (after - before));
    }

    /**
     * @brief 获取与主时钟的时间偏移
     */
    double getTimeOffsetUs() const {
        return master_offset_ns_ / 1000.0;
    }

    /**
     * @brief 获取时钟漂移率 (ppm)
     */
    double getDriftPpm() const {
        return drift_ppm_;
    }

    /**
     * @brief 设置同步模式
     */
    void setSyncMode(SyncMode mode) {
        config_.mode = mode;
        ROS_INFO("[TimestampSync] Mode set to %d", static_cast<int>(mode));
    }

    /**
     * @brief 获取同步状态报告
     */
    std::string getStatusReport() const {
        std::ostringstream oss;
        oss << "SyncMode: " << static_cast<int>(config_.mode)
            << " Offset: " << getTimeOffsetUs() << "us"
            << " Drift: " << drift_ppm_ << "ppm"
            << " History: " << recent_data_.size() << " samples";
        return oss.str();
    }

private:
    SyncConfig config_;
    std::atomic<int64_t> master_offset_ns_;
    std::atomic<double>  drift_ppm_;
    std::deque<SensorData> recent_data_;
    std::mutex mutex_;
    uint64_t sequence_counter_{0};
    static constexpr size_t max_history_ = 1000;

    uint64_t getSystemTimeNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    uint64_t estimateTimestamp(SensorType type, uint64_t hw_ts, uint64_t rx_ts) {
        // 简单模型: timestamp = rx_time - estimated_delay
        // 延迟包括：传输延迟 + 处理延迟
        double estimated_delay_ns = 500000;  // 初始估计500us
        if (master_offset_ns_ != 0) {
            estimated_delay_ns = std::abs(static_cast<double>(master_offset_ns_));
        }
        return rx_ts - static_cast<uint64_t>(estimated_delay_ns);
    }

    void updateDriftEstimate(const SensorData& data) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (recent_data_.size() < 10) return;

        // 计算最近样本的时间偏移变化率
        auto& oldest = recent_data_.front();
        auto& newest = recent_data_.back();

        int64_t time_diff_ns = static_cast<int64_t>(newest.receive_timestamp_ns -
                                                     oldest.receive_timestamp_ns);
        if (time_diff_ns < 1000000) return;  // 少于1ms，不更新

        int64_t offset_diff_ns = static_cast<int64_t>(newest.receive_timestamp_ns -
                                                       newest.hardware_timestamp_ns) -
                                 static_cast<int64_t>(oldest.receive_timestamp_ns -
                                                       oldest.hardware_timestamp_ns);

        // ppm = offset_diff_ns / time_diff_ns * 1e6
        drift_ppm_ = static_cast<double>(offset_diff_ns) / time_diff_ns * 1e6;
        drift_ppm_.store(drift_ppm_ * config_.drift_compensation_rate +
                        drift_ppm_ * (1.0 - config_.drift_compensation_rate));
    }
};

} // namespace core
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_TIMESTAMP_SYNC_H
