/******************************************************************************
 * lslidar_ls_driver_enhanced.h — 增强版激光雷达驱动
 *
 * 功能: 在原有LslidarChDriver基础上，集成多线程Pipeline、点云处理算法、
 *       资源监控、性能分析、多传感器同步等高级特性。
 *
 * 架构:
 *   UDP Capture -> Packet Parser -> Pipeline -> ROS Publication
 *       (独立线程)    (独立线程)     (多级流水线)
 *
 * 亮点:
 * - 多级流水线架构，各级独立线程
 * - 非阻塞 UDP/epoll 输入与显式文件描述符生命周期
 * - 点云滤波/分割/聚类/分类完整管线 (C++)
 * - 嵌入式资源监控 (CPU/内存/IO)
 * - 性能Profiler (延迟/P99/帧率)
 * - 多传感器时间同步
 * - Jetson交叉编译支持
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_DRIVER_ENHANCED_H
#define LSLIDAR_LS_DRIVER_ENHANCED_H

#include <lslidar_ls_driver/lslidar_ls_driver.h>
#include <lslidar_ls_driver/core/pipeline.h>
#include <lslidar_ls_driver/core/data_capture.h>
#include <lslidar_ls_driver/core/timestamp_sync.h>
#include <lslidar_ls_driver/algorithm/obstacle_detection.h>
#include <lslidar_ls_driver/platform/resource_monitor.h>
#include <lslidar_ls_driver/platform/perf_profiler.h>

#include <memory>
#include <atomic>
#include <cstdio>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

namespace lslidar_ch_driver {

/**
 * @brief Pipeline Stage 1: 数据预处理 (滤波)
 */
class PreFilterStage : public PipelineStage {
public:
    using Ptr = std::shared_ptr<PreFilterStage>;

    PreFilterStage()
        : PipelineStage("PreFilter", 32) {
        algorithm::FilterParams params;
        params.voxel_leaf_size = 0.15;
        params.enable_statistical_outlier = true;
        params.sor_mean_k = 20;
        params.range_min = 0.5;
        params.range_max = 200.0;
        filter_ = std::make_unique<algorithm::PointCloudFilter>(params);
    }

protected:
    StampedPointCloud::Ptr process(StampedPointCloud::Ptr data) override {
        PROFILE_SCOPE("PreFilter");
        if (!data || !data->cloud || data->cloud->empty()) return nullptr;
        auto filtered = filter_->filter(data->cloud);
        auto result = std::make_shared<StampedPointCloud>();
        result->cloud = filtered;
        result->stamp = data->stamp;
        result->frame_id = data->frame_id;
        result->frame_sequence = data->frame_sequence;
        result->acquisition_time = data->acquisition_time;
        return result;
    }

private:
    std::unique_ptr<algorithm::PointCloudFilter> filter_;
};

/**
 * @brief Pipeline Stage 2: 感知处理 (地面分割 + 障碍物检测)
 */
class PerceptionStage : public PipelineStage {
public:
    using Ptr = std::shared_ptr<PerceptionStage>;

    PerceptionStage()
        : PipelineStage("Perception", 16) {
        algorithm::PerceptionParams params;
        params.ground_seg_params.distance_threshold = 0.3;
        params.ground_seg_params.max_iterations = 100;
        params.cluster_params.cluster_tolerance = 0.5;
        params.cluster_params.min_cluster_size = 20;
        params.cluster_params.max_cluster_size = 25000;
        detector_ = std::make_unique<algorithm::ObstacleDetector>(params);
    }

    void setDetectionResultPub(const ros::Publisher& pub) {
        marker_pub_ = pub;
    }

    const algorithm::DetectionResult& getLastResult() const { return last_result_; }

protected:
    StampedPointCloud::Ptr process(StampedPointCloud::Ptr data) override {
        PROFILE_SCOPE("Perception");
        if (!data || !data->cloud || data->cloud->empty()) return nullptr;
        last_result_ = detector_->detect(data->cloud);
        if (marker_pub_.getNumSubscribers() > 0) {
            publishMarkers(last_result_, data->frame_id);
        }
        auto result = std::make_shared<StampedPointCloud>();
        result->stamp = data->stamp;
        result->frame_id = data->frame_id;
        result->frame_sequence = data->frame_sequence;
        result->cloud = last_result_.colored_cloud;
        return result;
    }

private:
    std::unique_ptr<algorithm::ObstacleDetector> detector_;
    algorithm::DetectionResult last_result_;
    ros::Publisher marker_pub_;

    void publishMarkers(const algorithm::DetectionResult& result,
                        const std::string& frame_id) {
        visualization_msgs::MarkerArray markers;
        int id = 0;
        for (const auto& obs : result.obstacles) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "obstacles";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = obs.bbox.center.x();
            marker.pose.position.y = obs.bbox.center.y();
            marker.pose.position.z = obs.bbox.center.z();
            marker.scale.x = obs.bbox.dimensions.x();
            marker.scale.y = obs.bbox.dimensions.y();
            marker.scale.z = obs.bbox.dimensions.z();
            switch (obs.type) {
                case algorithm::ObstacleType::VEHICLE:
                    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; break;
                case algorithm::ObstacleType::PERSON:
                    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; break;
                case algorithm::ObstacleType::BUILDING:
                    marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 1.0; break;
                default:
                    marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; break;
            }
            marker.color.a = 0.6;
            marker.lifetime = ros::Duration(0.5);
            markers.markers.push_back(marker);
        }
        // 盲降区标记
        if (result.safety_zone.landing_zone.is_suitable) {
            visualization_msgs::Marker lz_marker;
            lz_marker.header.frame_id = frame_id;
            lz_marker.header.stamp = ros::Time::now();
            lz_marker.ns = "landing_zone";
            lz_marker.id = 0;
            lz_marker.type = visualization_msgs::Marker::CYLINDER;
            lz_marker.action = visualization_msgs::Marker::ADD;
            lz_marker.pose.position.x = result.safety_zone.landing_zone.center_x;
            lz_marker.pose.position.y = result.safety_zone.landing_zone.center_y;
            lz_marker.pose.position.z = result.safety_zone.landing_zone.max_slope_deg;
            lz_marker.scale.x = result.safety_zone.landing_zone.radius * 2;
            lz_marker.scale.y = result.safety_zone.landing_zone.radius * 2;
            lz_marker.scale.z = 0.1;
            lz_marker.color.g = 1.0; lz_marker.color.a = 0.3;
            lz_marker.lifetime = ros::Duration(1.0);
            markers.markers.push_back(lz_marker);
        }
        marker_pub_.publish(markers);
    }
};

/**
 * @brief Pipeline Stage 3: 可视化输出
 */
class VisualizationStage : public PipelineStage {
public:
    using Ptr = std::shared_ptr<VisualizationStage>;
    VisualizationStage() : PipelineStage("Visualization", 8) {}
    void setCloudPub(const ros::Publisher& pub) { cloud_pub_ = pub; }

protected:
    StampedPointCloud::Ptr process(StampedPointCloud::Ptr data) override {
        PROFILE_SCOPE("Visualization");
        if (!data || !data->cloud || data->cloud->empty()) return nullptr;
        if (cloud_pub_.getNumSubscribers() > 0) {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*data->cloud, msg);
            msg.header.frame_id = data->frame_id;
            msg.header.stamp = data->stamp;
            cloud_pub_.publish(msg);
        }
        return data;
    }

private:
    ros::Publisher cloud_pub_;
};

/**
 * @brief 增强版激光雷达驱动
 */
class EnhancedLslidarDriver {
public:
    EnhancedLslidarDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), resource_monitor_(0.5) {}

    ~EnhancedLslidarDriver() { shutdown(); }

    bool initialize() {
        loadParameters();
        base_driver_ = std::make_unique<LslidarChDriver>(nh_, pnh_);
        std::fprintf(stderr, "[EnhancedInit] Calling base initialize\n");
        if (!base_driver_->initialize()) {
            ROS_ERROR("Failed to initialize base LiDAR driver.");
            return false;
        }
        std::fprintf(stderr, "[EnhancedInit] Base initialize returned\n");
        std::fprintf(stderr, "[EnhancedInit] Before pipeline stages\n");
        setupPipeline();
        ROS_INFO("[EnhancedInit] Pipeline stages ready");
        createPublishers();
        ROS_INFO("[EnhancedInit] Publishers ready");
        resource_monitor_.start();
        ROS_INFO("[EnhancedInit] Resource monitor ready");
        platform::ProfilerManager::instance().setMaxSamples(10000);
        diag_timer_ = nh_.createTimer(ros::Duration(10.0),
            &EnhancedLslidarDriver::diagnosticCallback, this);

        ROS_INFO("===== Enhanced LiDAR Driver Initialized =====");
        ROS_INFO("Pipeline: Filter -> Perception -> Visualize");
        ROS_INFO("Resource Monitor: every 0.5 sec");
        return true;
    }

    bool spin() {
        pipeline_.start();
        while (ros::ok()) {
            bool ret = base_driver_->polling();
            StampedPointCloud::Ptr data = std::make_shared<StampedPointCloud>();
            {
                std::lock_guard<std::mutex> lock(base_driver_->pc_mutex_);
                if (base_driver_->point_cloud_xyzirt_pub_->empty()) {
                    if (!ret) break;
                    ros::spinOnce();
                    continue;
                }
                data->cloud->reserve(base_driver_->point_cloud_xyzirt_pub_->size());
                for (const auto& pt : base_driver_->point_cloud_xyzirt_pub_->points) {
                    pcl::PointXYZI p;
                    p.x = pt.x; p.y = pt.y; p.z = pt.z;
                    p.intensity = pt.intensity;
                    data->cloud->push_back(p);
                }
                data->frame_id = base_driver_->frame_id;
            }
            data->stamp = ros::Time::now();
            data->frame_sequence = ++frame_sequence_;
            pipeline_.feed(data);

            // 记录资源监控延迟
            resource_monitor_.recordLatency(
                (ros::Time::now() - data->stamp).toSec() * 1000.0);

            ros::spinOnce();
        }
        pipeline_.stop();
        return true;
    }

    void shutdown() {
        pipeline_.stop();
        resource_monitor_.stop();
        ROS_INFO("Enhanced LiDAR Driver shutdown.");
    }

private:
    ros::NodeHandle nh_, pnh_;
    std::unique_ptr<LslidarChDriver> base_driver_;
    PipelineController pipeline_;
    PreFilterStage::Ptr filter_stage_;
    PerceptionStage::Ptr perception_stage_;
    VisualizationStage::Ptr vis_stage_;
    ros::Publisher obstacle_marker_pub_, processed_cloud_pub_, diag_pub_;
    platform::ResourceMonitor resource_monitor_;
    ros::Timer diag_timer_;
    std::string frame_id_;
    uint64_t frame_sequence_{0};

    void loadParameters() {
        pnh_.param<std::string>("frame_id", frame_id_, "laser_link");
    }

    void setupPipeline() {
        std::fprintf(stderr, "[EnhancedInit] Constructing PreFilterStage\n");
        filter_stage_ = std::make_shared<PreFilterStage>();
        std::fprintf(stderr, "[EnhancedInit] Constructing PerceptionStage\n");
        perception_stage_ = std::make_shared<PerceptionStage>();
        std::fprintf(stderr, "[EnhancedInit] Constructing VisualizationStage\n");
        vis_stage_ = std::make_shared<VisualizationStage>();
        std::fprintf(stderr, "[EnhancedInit] Linking stages\n");
        pipeline_.addStage(filter_stage_);
        pipeline_.addStage(perception_stage_);
        pipeline_.addStage(vis_stage_);
    }

    void createPublishers() {
        obstacle_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "obstacle_markers", 10);
        processed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "processed_point_cloud", 10);
        diag_pub_ = nh_.advertise<std_msgs::String>("driver_diagnostics", 10);
        perception_stage_->setDetectionResultPub(obstacle_marker_pub_);
        vis_stage_->setCloudPub(processed_cloud_pub_);
    }

    void diagnosticCallback(const ros::TimerEvent&) {
        pipeline_.printMetricsReport();
        platform::ProfilerManager::instance().printReport();
        auto report = resource_monitor_.getReport();
        ROS_INFO("[Resource] %s", report.toString().c_str());
        std_msgs::String diag_msg;
        std::ostringstream oss;
        oss << "CPU:" << report.cpu_usage_percent << "% "
            << "MEM:" << report.memory_usage_mb << "MB "
            << "TEMP:" << report.temperature_c << "C "
            << "THR:" << report.thread_count;
        diag_msg.data = oss.str();
        diag_pub_.publish(diag_msg);
    }
};

} // namespace lslidar_ch_driver
#endif // LSLIDAR_LS_DRIVER_ENHANCED_H
