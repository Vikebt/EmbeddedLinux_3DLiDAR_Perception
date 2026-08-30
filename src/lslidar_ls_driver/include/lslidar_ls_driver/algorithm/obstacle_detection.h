/******************************************************************************
 * obstacle_detection.h — 障碍物检测算法模块
 *
 * 功能: 整合滤波-地面分割-聚类-分类的完整点云感知管线，
 *       提供统一的检测结果输出（障碍物列表、安全区域评估等）。
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_OBSTACLE_DETECTION_H
#define LSLIDAR_LS_OBSTACLE_DETECTION_H

#include <lslidar_ls_driver/algorithm/filter.h>
#include <lslidar_ls_driver/algorithm/ground_segmentation.h>
#include <lslidar_ls_driver/algorithm/clustering.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace lslidar_ch_driver {
namespace algorithm {

/**
 * @brief 感知管线参数
 */
struct PerceptionParams {
    FilterParams      filter_params;
    GroundSegParams   ground_seg_params;
    ClusterParams     cluster_params;
};

/**
 * @brief 障碍物检测结果
 */
struct DetectionResult {
    std::vector<Cluster> obstacles;
    GroundSegResult      ground_info;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr colored_cloud;  // 地面=灰, 障碍物=彩色

    // 安全区评估
    struct SafetyZone {
        bool   is_safe{false};
        double clear_radius{0.0};      // 无障碍半径 (米)
        double nearest_obstacle_dist{1e9};

        // 盲降区检测
        struct LandingZone {
            double center_x{0.0}, center_y{0.0};
            double radius{0.0};
            double max_slope_deg{0.0};
            double roughness{0.0};
            bool   is_suitable{false};
        };
        LandingZone landing_zone;
    };
    SafetyZone safety_zone;

    // 性能统计
    double filter_time_ms{0.0};
    double seg_time_ms{0.0};
    double cluster_time_ms{0.0};
    double total_time_ms{0.0};
};

/**
 * @brief 完整障碍物检测管线
 *
 * 整合三个子模块：
 *   RawCloud -> Filter -> GroundSeg -> Clustering -> Classification
 */
class ObstacleDetector {
public:
    explicit ObstacleDetector(const PerceptionParams& params = PerceptionParams())
        : filter_(params.filter_params)
        , ground_seg_(params.ground_seg_params)
        , clustering_(params.cluster_params) {}

    /**
     * @brief 执行完整检测管线
     */
    DetectionResult detect(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

        DetectionResult result;
        auto start_total = std::chrono::steady_clock::now();

        // Stage 1: 滤波
        auto start = std::chrono::steady_clock::now();
        result.filtered_cloud = filter_.filter(cloud);
        result.filter_time_ms = elapsedMs(start);
        ROS_DEBUG("[Detection] Filter: %zu -> %zu points (%.1fms)",
                  cloud->size(), result.filtered_cloud->size(), result.filter_time_ms);

        // Stage 2: 地面分割
        start = std::chrono::steady_clock::now();
        result.ground_info = ground_seg_.segment(result.filtered_cloud);
        result.seg_time_ms = elapsedMs(start);
        ROS_DEBUG("[Detection] GroundSeg: ground=%zu, nonground=%zu (%.1fms)",
                  result.ground_info.ground_cloud->size(),
                  result.ground_info.nonground_cloud->size(),
                  result.seg_time_ms);

        // Stage 3: 聚类
        start = std::chrono::steady_clock::now();
        result.obstacles = clustering_.cluster(result.ground_info.nonground_cloud);
        result.cluster_time_ms = elapsedMs(start);
        ROS_DEBUG("[Detection] Clustering: %zu obstacles (%.1fms)",
                  result.obstacles.size(), result.cluster_time_ms);

        // Stage 4: 安全区评估
        result.safety_zone = evaluateSafetyZone(result);

        // 生成着色点云
        result.colored_cloud = generateColoredCloud(result);

        result.total_time_ms = elapsedMs(start_total);
        return result;
    }

    // 各子模块参数访问
    PointCloudFilter& filter() { return filter_; }
    GroundSegmentation& groundSeg() { return ground_seg_; }
    Clustering& clusterAlgo() { return clustering_; }

    void setParams(const PerceptionParams& params) {
        filter_.setParams(params.filter_params);
        ground_seg_.setParams(params.ground_seg_params);
        clustering_.setParams(params.cluster_params);
    }

private:
    PointCloudFilter     filter_;
    GroundSegmentation   ground_seg_;
    Clustering           clustering_;

    double elapsedMs(const std::chrono::steady_clock::time_point& start) {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    }

    DetectionResult::SafetyZone evaluateSafetyZone(const DetectionResult& result) {
        DetectionResult::SafetyZone zone;

        // 找最近的障碍物
        double min_dist = 1e9;
        for (const auto& obs : result.obstacles) {
            double dist = obs.centroid.norm();
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        zone.nearest_obstacle_dist = min_dist;
        zone.clear_radius = (min_dist > 50.0) ? 50.0 : min_dist * 0.8;
        zone.is_safe = min_dist > 5.0;

        // 盲降区分析
        if (result.ground_info.has_ground) {
            zone.landing_zone.max_slope_deg = result.ground_info.ground_slope_deg;
            zone.landing_zone.roughness = result.ground_info.ground_plane_error;
            zone.landing_zone.is_suitable =
                zone.landing_zone.max_slope_deg < 8.0 &&   // 坡度 < 8度
                zone.landing_zone.roughness < 0.1 &&       // 粗糙度 < 0.1m
                zone.is_safe;                               // 无障碍物
            zone.landing_zone.radius = zone.clear_radius > 2.0 ? 2.0 : zone.clear_radius;
        }

        return zone;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr generateColoredCloud(
        const DetectionResult& result) {

        auto colored = pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
        colored->reserve(result.filtered_cloud->size());

        // 地面点标记为灰色 (使用强度值编码: 地面=100, 障碍物=按类型)
        for (const auto& pt : result.ground_info.ground_cloud->points) {
            pcl::PointXYZI p = pt;
            p.intensity = 100.0f;  // 地面标记
            colored->push_back(p);
        }

        // 障碍物按类型标记强度
        for (const auto& cluster : result.obstacles) {
            float intensity = 200.0f + static_cast<float>(cluster.type) * 30.0f;
            for (const auto& pt : cluster.cloud->points) {
                pcl::PointXYZI p = pt;
                p.intensity = intensity;
                colored->push_back(p);
            }
        }

        return colored;
    }
};

} // namespace algorithm
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_OBSTACLE_DETECTION_H
