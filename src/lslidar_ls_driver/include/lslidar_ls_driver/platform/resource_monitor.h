/******************************************************************************
 * resource_monitor.h — 嵌入式资源监控模块
 *
 * 功能: 实时监控CPU使用率、内存占用、IO吞吐、网络延迟等关键指标，
 *       在资源受限的嵌入式平台（如Jetson）上评估系统健康状态。
 *
 * 使用方法:
 *   ResourceMonitor monitor(1.0);  // 每秒采样一次
 *   monitor.start();
 *   // ... 运行系统 ...
 *   auto report = monitor.getReport();
 *   ROS_INFO("CPU: %.1f%%, Memory: %.1fMB", report.cpu_usage, report.memory_usage_mb);
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_RESOURCE_MONITOR_H
#define LSLIDAR_LS_RESOURCE_MONITOR_H

#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <chrono>
#include <fstream>
#include <sstream>
#include <numeric>
#include <ros/ros.h>

namespace lslidar_ch_driver {
namespace platform {

/**
 * @brief 资源使用报告
 */
struct ResourceReport {
    double cpu_usage_percent{0.0};         // CPU使用率 (%)
    double memory_usage_mb{0.0};           // 内存使用 (MB)
    double memory_percent{0.0};            // 内存使用率 (%)
    double disk_read_mbps{0.0};            // 磁盘读取 (MB/s)
    double disk_write_mbps{0.0};           // 磁盘写入 (MB/s)
    double network_rx_kbps{0.0};           // 网络接收 (KB/s)
    double network_tx_kbps{0.0};           // 网络发送 (KB/s)
    double temperature_c{0.0};             // CPU温度 (°C, 仅Linux)
    double process_cpu_percent{0.0};       // 本进程CPU使用率
    double process_memory_mb{0.0};         // 本进程内存 (MB)
    int    thread_count{0};                // 本进程线程数
    int    fd_count{0};                    // 本进程文件描述符数

    std::string toString() const {
        std::ostringstream oss;
        oss << "CPU: " << cpu_usage_percent << "% (proc:" << process_cpu_percent << "%) "
            << "MEM: " << memory_usage_mb << "MB (" << memory_percent << "%) "
            << "TEMP: " << temperature_c << "C "
            << "THR: " << thread_count << " FD:" << fd_count;
        return oss.str();
    }
};

/**
 * @brief 历史资源数据
 */
struct ResourceHistory {
    std::vector<double> cpu_samples;
    std::vector<double> memory_samples;
    std::vector<double> latency_samples;     // 处理延迟样本 (ms)
    ros::Time start_time;

    void addSample(double cpu, double mem, double latency_ms) {
        cpu_samples.push_back(cpu);
        memory_samples.push_back(mem);
        latency_samples.push_back(latency_ms);
    }

    double cpuAvg() const {
        if (cpu_samples.empty()) return 0.0;
        return std::accumulate(cpu_samples.begin(), cpu_samples.end(), 0.0) / cpu_samples.size();
    }

    double cpuMax() const {
        if (cpu_samples.empty()) return 0.0;
        return *std::max_element(cpu_samples.begin(), cpu_samples.end());
    }

    double memAvg() const {
        if (memory_samples.empty()) return 0.0;
        return std::accumulate(memory_samples.begin(), memory_samples.end(), 0.0) / memory_samples.size();
    }

    double latencyAvg() const {
        if (latency_samples.empty()) return 0.0;
        return std::accumulate(latency_samples.begin(), latency_samples.end(), 0.0) / latency_samples.size();
    }

    double latencyP99() const {
        if (latency_samples.empty()) return 0.0;
        auto sorted = latency_samples;
        std::sort(sorted.begin(), sorted.end());
        size_t idx = static_cast<size_t>(sorted.size() * 0.99);
        return sorted[std::min(idx, sorted.size() - 1)];
    }

    size_t sampleCount() const { return cpu_samples.size(); }
};

/**
 * @brief 资源监控器
 *
 * 在后台线程定期采样系统资源使用情况，可随时获取报告。
 */
class ResourceMonitor {
public:
    /**
     * @param interval_sec 采样间隔（秒）
     * @param history_size 保留的历史样本数
     */
    explicit ResourceMonitor(double interval_sec = 1.0, size_t history_size = 3600)
        : interval_sec_(interval_sec)
        , max_history_(history_size)
        , running_(false)
        , page_size_(sysconf(_SC_PAGESIZE))
        , clock_ticks_(sysconf(_SC_CLK_TCK)) {

        // 获取本进程PID
        char pid_path[32];
        snprintf(pid_path, sizeof(pid_path), "/proc/%d/stat", getpid());
        pid_ = getpid();
    }

    ~ResourceMonitor() { stop(); }

    void start() {
        if (running_.exchange(true)) return;
        // 获取初始CPU时间
        prev_cpu_time_ = readProcCpuTime();
        prev_sys_time_ = readSysCpuTime();
        worker_ = std::thread(&ResourceMonitor::sampleLoop, this);
        ROS_INFO("[ResourceMonitor] Started (interval=%.1fs)", interval_sec_);
    }

    void stop() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
    }

    ResourceReport getReport() {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_;
    }

    ResourceHistory getHistory() {
        std::lock_guard<std::mutex> lock(mutex_);
        return history_;
    }

    /**
     * @brief 记录处理延迟
     */
    void recordLatency(double latency_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        history_.latency_samples.push_back(latency_ms);
        if (history_.latency_samples.size() > max_history_) {
            history_.latency_samples.erase(history_.latency_samples.begin());
        }
    }

    bool isRunning() const { return running_; }

private:
    double       interval_sec_;
    size_t       max_history_;
    std::atomic<bool> running_;
    std::thread  worker_;
    ResourceReport  current_;
    ResourceHistory history_;
    mutable std::mutex mutex_;

    pid_t pid_{0};
    long  page_size_{4096};
    long  clock_ticks_{100};

    // 上一次采样的CPU时间
    uint64_t prev_cpu_time_{0};
    uint64_t prev_sys_time_{0};

    void sampleLoop() {
        while (running_) {
            sample();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(interval_sec_ * 1000)));
        }
    }

    void sample() {
        std::lock_guard<std::mutex> lock(mutex_);

        // 整体CPU
        current_.cpu_usage_percent = readCpuUsage();

        // 本进程CPU
        current_.process_cpu_percent = readProcCpuUsage();

        // 内存
        readMemoryInfo(current_.memory_usage_mb, current_.memory_percent,
                       current_.process_memory_mb);

        // 本进程信息
        current_.thread_count = readProcThreadCount();
        current_.fd_count = readProcFdCount();

        // 温度（如可用）
        current_.temperature_c = readCpuTemperature();

        // 记录历史
        history_.addSample(current_.cpu_usage_percent,
                           current_.memory_usage_mb,
                           history_.latency_samples.empty() ? 0.0 : history_.latency_samples.back());
        if (history_.cpu_samples.size() > max_history_) {
            history_.cpu_samples.erase(history_.cpu_samples.begin());
            history_.memory_samples.erase(history_.memory_samples.begin());
        }
    }

    double readCpuUsage() {
        uint64_t sys_time = readSysCpuTime();
        double delta = static_cast<double>(sys_time - prev_sys_time_);
        double usage = 0.0;
        if (delta > 0) {
            usage = 100.0 * static_cast<double>(sys_time - prev_sys_time_) / delta;
        }
        prev_sys_time_ = sys_time;
        return std::min(usage, 100.0);
    }

    double readProcCpuUsage() {
        uint64_t proc_time = readProcCpuTime();
        uint64_t sys_time = readSysCpuTime();
        double delta_proc = static_cast<double>(proc_time - prev_cpu_time_);
        double delta_sys = static_cast<double>(sys_time - prev_sys_time_);
        prev_cpu_time_ = proc_time;
        if (delta_sys < 1e-6) return 0.0;
        return std::min(100.0 * delta_proc / delta_sys, 100.0);
    }

    uint64_t readSysCpuTime() {
        std::ifstream stat("/proc/stat");
        std::string line;
        if (!std::getline(stat, line)) return 0;
        std::istringstream iss(line);
        std::string cpu;
        uint64_t user, nice, sys, idle, iowait, irq, softirq, steal;
        iss >> cpu >> user >> nice >> sys >> idle >> iowait >> irq >> softirq >> steal;
        return user + nice + sys + idle + iowait + irq + softirq + steal;
    }

    uint64_t readProcCpuTime() {
        std::ifstream stat("/proc/" + std::to_string(pid_) + "/stat");
        std::string line;
        if (!std::getline(stat, line)) return 0;
        // utime (14) + stime (15)
        std::istringstream iss(line);
        std::string dummy;
        for (int i = 0; i < 13; ++i) iss >> dummy;
        uint64_t utime, stime;
        iss >> utime >> stime;
        return (utime + stime) * 10000000ULL / clock_ticks_;
    }

    void readMemoryInfo(double& total_mb, double& total_percent, double& proc_mb) {
        // 系统内存
        std::ifstream meminfo("/proc/meminfo");
        std::string line;
        double mem_total_kb = 0, mem_avail_kb = 0;
        while (std::getline(meminfo, line)) {
            if (line.find("MemTotal:") == 0)
                sscanf(line.c_str(), "MemTotal: %lf kB", &mem_total_kb);
            if (line.find("MemAvailable:") == 0)
                sscanf(line.c_str(), "MemAvailable: %lf kB", &mem_avail_kb);
        }
        total_mb = (mem_total_kb - mem_avail_kb) / 1024.0;
        total_percent = mem_total_kb > 0 ? (mem_total_kb - mem_avail_kb) / mem_total_kb * 100.0 : 0.0;

        // 进程内存
        std::ifstream status("/proc/" + std::to_string(pid_) + "/status");
        while (std::getline(status, line)) {
            if (line.find("VmRSS:") == 0) {
                double rss_kb;
                sscanf(line.c_str(), "VmRSS: %lf kB", &rss_kb);
                proc_mb = rss_kb / 1024.0;
                break;
            }
        }
    }

    int readProcThreadCount() {
        std::ifstream status("/proc/" + std::to_string(pid_) + "/status");
        std::string line;
        while (std::getline(status, line)) {
            if (line.find("Threads:") == 0) {
                int threads;
                sscanf(line.c_str(), "Threads: %d", &threads);
                return threads;
            }
        }
        return 0;
    }

    int readProcFdCount() {
        std::string fd_path = "/proc/" + std::to_string(pid_) + "/fd";
        DIR* dir = opendir(fd_path.c_str());
        if (!dir) return 0;
        int count = 0;
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) count++;
        closedir(dir);
        return count - 2;  // 减去 . 和 ..
    }

    double readCpuTemperature() {
        std::ifstream temp("/sys/class/thermal/thermal_zone0/temp");
        if (!temp.is_open()) return 0.0;
        int millicelsius;
        temp >> millicelsius;
        return millicelsius / 1000.0;
    }
};

} // namespace platform
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_RESOURCE_MONITOR_H
