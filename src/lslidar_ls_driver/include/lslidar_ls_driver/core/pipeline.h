/******************************************************************************
 * pipeline.h — 多线程流水线框架
 *
 * 【这是实现多线程Pipeline的核心框架文件】
 *
 * 本框架实现了一个通用的多级流水线处理架构:
 *
 *   ┌─────────┐   队列   ┌─────────┐   队列   ┌─────────┐
 *   │ Stage 1 │ ───────→ │ Stage 2 │ ───────→ │ Stage 3 │
 *   │ (滤波)  │          │ (分割)   │          │ (聚类)   │
 *   └─────────┘          └─────────┘          └─────────┘
 *      线程1                线程2                线程3
 *
 * 每个Stage:
 *   - 运行在独立线程中（互不干扰）
 *   - 通过 ThreadSafeQueue 接收输入数据
 *   - 处理完成后将结果传递给下一个Stage
 *   - 拥有独立的性能统计（耗时/帧率）
 *
 * 关键设计:
 *   - 线程安全队列: mutex + condition_variable 实现生产者-消费者模型
 *   - 背压控制: 队列满时生产者自动阻塞，防止内存溢出
 *   - 性能追踪: 每个Stage自动统计处理耗时
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_PIPELINE_H
#define LSLIDAR_LS_PIPELINE_H

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <atomic>
#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace lslidar_ch_driver {

// ============================================================================
// 线程安全的有界队列 (Thread-Safe Bounded Queue)
//
// 【多线程编程的核心组件】
//
// 实现了经典的生产者-消费者模型:
//   生产者(前一个Stage): 调用 push() 放入数据
//   消费者(当前Stage):   调用 pop() 取出数据
//
// 关键特性:
//   1. 线程安全: 用 mutex + condition_variable 保护队列操作
//   2. 有界队列: max_size_ 限制队列长度，防止内存溢出
//   3. 背压控制: 队列满时 push() 会阻塞等待，直到消费者取走数据
//   4. 优雅关闭: shutdown() 唤醒所有等待线程
//
// 类比:
//   想象一个传送带，前端放产品（push），后端取产品（pop），
//   传送带长度固定（max_size），满了前端就停下来等。
// ============================================================================
template<typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_size = 64)
        : max_size_(max_size), shutdown_(false) {}

    /**
     * push — 生产者放入数据
     *
     * 如果队列已满，当前线程会阻塞在这里等待
     * 直到消费者 pop() 腾出空间
     */
    bool push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        // wait() 会释放锁并让线程休眠，直到条件满足
        // 条件: 队列未满 或 已关闭
        not_full_.wait(lock, [this]() {
            return queue_.size() < max_size_ || shutdown_;
        });
        if (shutdown_) return false;
        queue_.push(std::move(item));
        not_empty_.notify_one();  // 唤醒一个等待的消费者
        return true;
    }

    /**
     * pop — 消费者取出数据
     *
     * 如果队列为空，当前线程会阻塞在这里等待
     * 直到生产者 push() 放入新数据
     *
     * @param item [out] 取出的数据
     * @param timeout_ms 超时时间(毫秒)，-1=无限等待
     * @return true=成功取出, false=超时或已关闭
     */
    bool pop(T& item, int timeout_ms = -1) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (timeout_ms > 0) {
            if (!not_empty_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                     [this]() { return !queue_.empty() || shutdown_; })) {
                return false;  // 超时
            }
        } else {
            not_empty_.wait(lock, [this]() { return !queue_.empty() || shutdown_; });
        }
        if (shutdown_ && queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        not_full_.notify_one();  // 唤醒一个等待的生产者
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    void shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        not_full_.notify_all();   // 唤醒所有等待的生产者
        not_empty_.notify_all();  // 唤醒所有等待的消费者
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) queue_.pop();
        shutdown_ = false;
    }

private:
    mutable std::mutex mutex_;            // 互斥锁，保护队列操作
    std::condition_variable not_full_;    // 队列未满条件（生产者等待）
    std::condition_variable not_empty_;   // 队列非空条件（消费者等待）
    std::queue<T> queue_;                 // 实际数据队列
    size_t max_size_;                     // 队列最大容量
    bool shutdown_;                       // 关闭标志
};

// ============================================================================
// 流水线性能指标
// ============================================================================
struct PipelineMetrics {
    std::string stage_name;
    double      avg_process_time_ms{0.0};      // 平均处理耗时
    double      max_process_time_ms{0.0};      // 最大处理耗时
    double      min_process_time_ms{1e9};      // 最小处理耗时
    size_t      total_processed{0};            // 已处理帧数
    double      fps{0.0};                      // 当前帧率
    size_t      queue_size{0};                 // 当前队列长度
    uint64_t    last_process_time_us{0};       // 上次耗时

    void update(double process_time_ms) {
        avg_process_time_ms = avg_process_time_ms * 0.9 + process_time_ms * 0.1;  // 指数移动平均
        max_process_time_ms = std::max(max_process_time_ms, process_time_ms);
        min_process_time_ms = std::min(min_process_time_ms, process_time_ms);
        total_processed++;
        last_process_time_us = static_cast<uint64_t>(process_time_ms * 1000.0);
    }
};

// ============================================================================
// 带时间戳的点云数据包
// ============================================================================
struct StampedPointCloud {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;  // 点云数据
    ros::Time stamp;                              // 时间戳
    uint64_t  frame_id{0};                        // 帧序号
    double    acquisition_time{0.0};              // 采集时间

    StampedPointCloud()
        : cloud(new pcl::PointCloud<pcl::PointXYZI>()) {}
};

// ============================================================================
// 流水线Stage基类 — 每个Stage是一个独立的处理节点
//
// 【继承此类实现具体的处理逻辑】
//
// 子类只需实现 process() 方法:
//   输入: 一帧点云数据
//   输出: 处理后的点云数据 (或 nullptr 表示跳过)
//
// 框架会自动:
//   - 创建独立线程运行处理循环
//   - 从输入队列取出数据
//   - 调用 process() 处理
//   - 将结果传递给下一个Stage
//   - 统计处理耗时
// ============================================================================
class PipelineStage {
public:
    using Ptr = std::shared_ptr<PipelineStage>;

    explicit PipelineStage(const std::string& name, size_t queue_size = 32)
        : name_(name), enabled_(true), input_queue_(queue_size), metrics_{name_} {}

    virtual ~PipelineStage() { stop(); }

    /**
     * 启动本Stage的处理线程
     * 线程会循环调用 processLoop()，直到 stop() 被调用
     */
    virtual bool start() {
        if (worker_.joinable()) return true;
        running_ = true;
        worker_ = std::thread(&PipelineStage::processLoop, this);
        ROS_INFO("[Pipeline] Stage '%s' started.", name_.c_str());
        return true;
    }

    /** 停止本Stage的处理线程 */
    virtual void stop() {
        running_ = false;
        input_queue_.shutdown();  // 唤醒等待中的线程
        if (worker_.joinable()) {
            worker_.join();  // 等待线程结束
        }
        ROS_INFO("[Pipeline] Stage '%s' stopped.", name_.c_str());
    }

    /** 向本Stage投递数据（由前一个Stage调用） */
    bool enqueue(StampedPointCloud::Ptr data) {
        if (!enabled_) return false;
        return input_queue_.push(std::move(data));
    }

    /** 设置下一个Stage（流水线链接） */
    void setNextStage(PipelineStage::Ptr next) {
        next_stage_ = next;
    }

    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
    const PipelineMetrics& getMetrics() const { return metrics_; }
    std::string getName() const { return name_; }
    size_t getQueueSize() const { return input_queue_.size(); }

protected:
    /**
     * 【子类必须实现此方法】
     * 处理一帧数据，返回处理结果
     * 返回 nullptr 表示跳过此帧（不传递给下一个Stage）
     */
    virtual StampedPointCloud::Ptr process(StampedPointCloud::Ptr data) = 0;

    std::string                  name_;        // Stage名称
    std::atomic<bool>            running_{false};   // 运行标志
    std::atomic<bool>            enabled_{true};    // 启用标志
    ThreadSafeQueue<StampedPointCloud::Ptr> input_queue_;  // 输入队列
    std::thread                  worker_;      // 工作线程
    PipelineMetrics              metrics_;     // 性能统计
    PipelineStage::Ptr           next_stage_;  // 下一个Stage

private:
    /**
     * 处理循环 — 在独立线程中运行
     *
     * 循环逻辑:
     *   1. 从输入队列取出一帧数据（阻塞等待，超时100ms重试）
     *   2. 调用子类的 process() 方法处理
     *   3. 将结果传递给下一个Stage
     *   4. 记录耗时
     *   5. 重复
     */
    void processLoop() {
        while (running_) {
            StampedPointCloud::Ptr data;
            // 从队列取数据，超时100ms（避免shutdown时卡死）
            if (!input_queue_.pop(data, 100)) {
                continue;  // 超时，检查running_标志后重试
            }

            auto start_time = std::chrono::steady_clock::now();

            try {
                // 调用子类实现的处理逻辑
                auto result = process(std::move(data));
                // 将结果传递给下一个Stage
                if (result && next_stage_) {
                    next_stage_->enqueue(std::move(result));
                }
            } catch (const std::exception& e) {
                ROS_ERROR("[Pipeline] Stage '%s' exception: %s",
                          name_.c_str(), e.what());
            }

            // 记录耗时
            auto elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start_time).count();
            metrics_.update(elapsed);
        }
    }
};

// ============================================================================
// 流水线控制器 — 管理所有Stage的生命周期
//
// 用法:
//   PipelineController pipeline;
//   pipeline.addStage(filter_stage);    // 添加 Stage
//   pipeline.addStage(seg_stage);
//   pipeline.addStage(cluster_stage);
//   pipeline.start();                    // 启动所有 Stage
//   pipeline.feed(data);                 // 向第一个 Stage 注入数据
//   pipeline.printMetricsReport();       // 打印性能报告
//   pipeline.stop();                     // 停止所有 Stage
// ============================================================================
class PipelineController {
public:
    PipelineController() : running_(false) {}

    void addStage(PipelineStage::Ptr stage) {
        stages_.push_back(stage);
    }

    /**
     * 链接所有Stage — 按添加顺序将每个Stage的输出连接到下一个Stage的输入
     * Stage1.output → Stage2.input → Stage2.output → Stage3.input → ...
     */
    void link() {
        for (size_t i = 0; i + 1 < stages_.size(); ++i) {
            stages_[i]->setNextStage(stages_[i + 1]);
        }
    }

    void start() {
        link();
        for (auto& stage : stages_) {
            stage->start();  // 每个Stage启动自己的处理线程
        }
        running_ = true;
        ROS_INFO("[Pipeline] Pipeline started with %zu stages.", stages_.size());
    }

    void stop() {
        running_ = false;
        for (auto& stage : stages_) {
            stage->stop();  // 每个Stage停止自己的处理线程
        }
        ROS_INFO("[Pipeline] Pipeline stopped.");
    }

    /** 向流水线第一个Stage注入数据 */
    bool feed(StampedPointCloud::Ptr data) {
        if (!running_ || stages_.empty()) return false;
        return stages_.front()->enqueue(std::move(data));
    }

    /** 获取所有Stage的性能指标 */
    std::vector<PipelineMetrics> getAllMetrics() const {
        std::vector<PipelineMetrics> all;
        for (const auto& stage : stages_) {
            all.push_back(stage->getMetrics());
        }
        return all;
    }

    /** 打印性能报告（每个Stage的耗时/帧率/队列长度） */
    void printMetricsReport() const {
        ROS_INFO("========== Pipeline Performance Report ==========");
        for (const auto& stage : stages_) {
            const auto& m = stage->getMetrics();
            ROS_INFO("  [%s] total=%zu  avg=%.2fms  max=%.2fms  min=%.2fms  queue=%zu",
                     stage->getName().c_str(),
                     m.total_processed, m.avg_process_time_ms,
                     m.max_process_time_ms, m.min_process_time_ms,
                     stage->getQueueSize());
        }
        ROS_INFO("================================================");
    }

private:
    std::vector<PipelineStage::Ptr> stages_;   // 所有Stage列表
    std::atomic<bool>               running_;  // 运行标志
};

} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_PIPELINE_H
