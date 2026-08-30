/******************************************************************************
 * pipeline_processor.h — 完整的点云处理管线（从Python迁移）
 *
 * 【这是算法层的核心文件，将Python脚本中的处理逻辑精确迁移到C++】
 *
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║  管线流程图 (数据从左到右流动):                                    ║
 * ║                                                                   ║
 * ║  原始点云 → [体素滤波] → [RANSAC地面分割] → [DBSCAN聚类] → 着色   ║
 * ║  (5万点)     (降采样)     (分离地面和非地面)  (识别障碍物)   输出   ║
 * ║                  ↓                                                ║
 * ║              [2.5D栅格着陆区检测]                                  ║
 * ║              (从地面点中找安全着陆点)                               ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 *
 * 从以下Python文件迁移:
 *   - ThreadRecog.py → RANSAC + DBSCAN + 包围盒
 *   - LandRecog2.py  → 2.5D栅格着陆区检测
 *   - obstacle_recog.py → 障碍物分类规则
 *
 * 参数默认值与Python版保持一致，确保输出结果一致。
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_PIPELINE_PROCESSOR_H
#define LSLIDAR_LS_PIPELINE_PROCESSOR_H

/* 引入四个算法模块 */
#include <lslidar_ls_driver/algorithm/filter.h>              // 体素滤波
#include <lslidar_ls_driver/algorithm/ground_segmentation.h>  // RANSAC地面分割
#include <lslidar_ls_driver/algorithm/clustering.h>           // DBSCAN聚类
#include <lslidar_ls_driver/algorithm/landing_zone.h>         // 2.5D着陆区检测

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

namespace lslidar_ch_driver {
namespace algorithm {

/**
 * @brief 管线处理结果 — 包含所有阶段的输出数据
 *
 * 当调用 process() 处理一帧点云后，返回此结构体，包含:
 *   - 滤波后的点云
 *   - 地面点云和非地面点云（分割结果）
 *   - 聚类出的障碍物列表（每个障碍物含中心、尺寸、类型）
 *   - 着色点云（地面=灰色，障碍物=按类型着色）
 *   - 着陆区检测结果
 *   - 各阶段耗时统计
 */
struct PipelineOutput {
    /* 滤波后点云 */
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;

    /* 地面分割结果 */
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud;    // 地面点
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_cloud; // 非地面点（障碍物候选）

    /* 聚类结果 — 每个Cluster含中心坐标、边界盒、障碍物类型 */
    std::vector<Cluster> obstacles;

    /* 着色点云 — 地面点(intensity=100), 障碍物点(intensity=类型*30+200) */
    pcl::PointCloud<pcl::PointXYZI>::Ptr colored_cloud;

    /* 着陆区检测结果 */
    LandingZoneResult landing_zone;

    /* 各阶段耗时(ms) */
    double filter_time_ms{0.0};
    double ground_seg_time_ms{0.0};
    double cluster_time_ms{0.0};
    double landing_zone_time_ms{0.0};
    double total_time_ms{0.0};
};

/**
 * @brief 点云处理管线 — 整合四个算法模块，与Python逻辑完全一致
 *
 * 用法:
 *   PipelineProcessor processor;    // 创建处理器（参数已内置）
 *   auto output = processor.process(cloud);  // 处理一帧
 *
 * 参数在构造函数中设置，与Python版保持一致:
 *   - 体素大小: 1.0m (对应 Python voxel_down_sample(voxel_size=1.0))
 *   - RANSAC阈距: 1.6m (对应 ThreadRecog: ground_seg_distance_threshold)
 *   - DBSCAN半径: 5.0m (对应 ThreadRecog: nonground_cluster_eps)
 *   - 栅格分辨率: 1.5m (对应 LandRecog2: grid_resolution)
 */
class PipelineProcessor {
public:
    PipelineProcessor() {
        // ═══════════════════════════════════════════════════════════
        // 1. 体素滤波参数
        //    对应Python: point.voxel_down_sample(voxel_size=1.0)
        //    作用: 将空间划分成1m×1m×1m的立方体，每个立方体内只保留一个点
        //    效果: 点数从5万减少到3千，减少97%计算量
        // ═══════════════════════════════════════════════════════════
        filter_params_.enable_voxel_grid = true;
        filter_params_.voxel_leaf_size = 1.0;          // 体素边长1米
        filter_params_.range_min = 0.5;                 // 去掉0.5m内的近点（雷达噪声）
        filter_params_.range_max = 200.0;               // 去掉200m外的远点
        filter_params_.enable_height_filter = true;
        filter_params_.height_min = -5.0;               // 地面以下5米
        filter_params_.height_max = 20.0;               // 地面以上20米
        filter_ = std::make_unique<PointCloudFilter>(filter_params_);

        // ═══════════════════════════════════════════════════════════
        // 2. RANSAC地面分割参数
        //    对应Python: pcd.segment_plane(distance_threshold=1.6, num_iterations=15)
        //    原理: 随机选3点拟合平面 → 计算所有点到平面距离 →
        //         距离<1.6m的点视为"地面" → 重复15次取最优
        //    法向量约束: 要求地面法向量Z分量>0.3（防止误识别垂直墙面）
        // ═══════════════════════════════════════════════════════════
        ground_seg_params_.distance_threshold = 1.6;    // RANSAC距离阈值
        ground_seg_params_.max_iterations = 15;          // 最大迭代次数
        ground_seg_params_.ground_normal_z_min = 0.3;    // 地面法向量Z分量最小值
        ground_seg_ = std::make_unique<GroundSegmentation>(ground_seg_params_);

        // ═══════════════════════════════════════════════════════════
        // 3. DBSCAN聚类参数
        //    对应Python: nonground_pcd.cluster_dbscan(eps=5.0, min_points=5)
        //    原理: 以5m为半径，把距离<5m的点聚成一类
        //    注意: eps=5m看似大，但这是体素滤波后的点云，点间距本就增大
        //    效果: 将非地面点聚成一个个障碍物簇
        // ═══════════════════════════════════════════════════════════
        cluster_params_.cluster_tolerance = 5.0;         // 聚类半径
        cluster_params_.min_cluster_size = 5;             // 最小聚类点数
        cluster_params_.max_cluster_size = 50000;
        clustering_ = std::make_unique<Clustering>(cluster_params_);

        // ═══════════════════════════════════════════════════════════
        // 4. 2.5D着陆区检测参数
        //    对应Python LandRecog2.py 的 _create_25d_grid_stats 和 _find_best_landing_zone
        //    原理: 将地面点投影到水平栅格 → 统计每个栅格的高度标准差 →
        //         筛选平坦栅格(std<0.08m) → 用圆形掩模搜索安全区域
        // ═══════════════════════════════════════════════════════════
        lz_params_.grid_resolution = 1.5;                // 栅格分辨率1.5米
        lz_params_.landing_zone_radius = 1.5;            // 着陆区半径1.5米
        lz_params_.max_cell_std_dev = 0.08;              // 栅格高度标准差<8cm
        lz_params_.max_cell_range = 0.15;                // 栅格高度范围<15cm
        lz_params_.max_zone_planarity_std_dev = 0.10;    // 区域平面性<10cm
        lz_params_.min_points_per_cell = 5;              // 每栅格至少5个点
        landing_zone_ = std::make_unique<LandingZoneDetector>(lz_params_);
    }

    /**
     * @brief 处理一帧点云 — 这是管线的核心入口函数
     *
     * 调用流程:
     *   1. 体素滤波: 降低点云密度
     *   2. RANSAC地面分割: 分离地面点和非地面点
     *   3. DBSCAN聚类: 将非地面点聚成障碍物
     *   4. 2.5D着陆区检测: 在地面点中找安全着陆区
     *   5. 生成着色点云: 地面=灰色, 障碍物=按类型着色
     *
     * @param input_cloud 输入原始点云
     * @return PipelineOutput 处理结果
     */
    PipelineOutput process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
        PipelineOutput output;
        auto total_start = std::chrono::steady_clock::now();

        if (!input_cloud || input_cloud->empty()) return output;

        // ─────────────────────────────────────────────────────────
        // Stage 1: 体素滤波 (Voxel Grid Downsampling)
        //
        // 原理: 把空间切成1m×1m×1m的小立方体
        //        每个立方体内的所有点用一个重心点代替
        // 效果: 5万点 → 3千点，后续所有算法都更快
        // ─────────────────────────────────────────────────────────
        auto start = std::chrono::steady_clock::now();
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(input_cloud);
        voxel.setLeafSize(1.0, 1.0, 1.0);  // 体素大小1m
        output.filtered_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
        voxel.filter(*output.filtered_cloud);
        output.filter_time_ms = elapsedMs(start);

        if (output.filtered_cloud->size() < 10) return output;

        // ─────────────────────────────────────────────────────────
        // Stage 2: RANSAC 地面分割 (Ground Segmentation)
        //
        // 原理: 随机选3个点确定一个平面，计算所有点到该平面的距离，
        //        距离<1.6m的点视为"地面"，重复15次取内点最多的平面
        // 效果: 输出 ground_cloud (地面) + nonground_cloud (非地面)
        // ─────────────────────────────────────────────────────────
        start = std::chrono::steady_clock::now();
        auto seg_result = ground_seg_->segment(output.filtered_cloud);
        output.ground_cloud = seg_result.ground_cloud;
        output.nonground_cloud = seg_result.nonground_cloud;
        output.ground_seg_time_ms = elapsedMs(start);

        // ─────────────────────────────────────────────────────────
        // Stage 3: DBSCAN 聚类 (Clustering)
        //
        // 原理: 对非地面点做欧式聚类，以eps=5m为半径，
        //        把距离<5m的点聚成一类，每类就是一个障碍物
        // 效果: 输出 obstacles 列表，每个含:
        //        - 中心坐标 (centroid)
        //        - 三维包围盒 (bounding box)
        //        - 障碍物类型 (vehicle/person/building/pole/tree)
        // ─────────────────────────────────────────────────────────
        start = std::chrono::steady_clock::now();
        output.obstacles = clustering_->cluster(output.nonground_cloud);
        output.cluster_time_ms = elapsedMs(start);

        // ─────────────────────────────────────────────────────────
        // Stage 4: 2.5D 栅格着陆区检测 (Landing Zone Detection)
        //
        // 原理: 将地面点投影到1.5m分辨率的2D栅格，
        //        每个栅格统计高度标准差，标准差<8cm的视为平坦，
        //        用圆形掩模(半径1.5m)搜索连续平坦区域，选最近的
        // 效果: 输出着陆区中心坐标、距离、平面度
        // ─────────────────────────────────────────────────────────
        start = std::chrono::steady_clock::now();
        output.landing_zone = landing_zone_->detect(output.ground_cloud);
        output.landing_zone_time_ms = elapsedMs(start);

        // ─────────────────────────────────────────────────────────
        // Stage 5: 生成着色点云
        //
        // 地面点 → intensity=100 (Python GUI 显示为灰色)
        // 障碍物点 → intensity=类型*30+200 (按类型着色)
        //   车辆=200, 行人=230, 建筑=260, 电线杆=290, 树木=320
        // ─────────────────────────────────────────────────────────
        output.colored_cloud = generateColoredCloud(output);

        output.total_time_ms = elapsedMs(total_start);
        return output;
    }

    /* 获取各算法模块的引用，可用于动态调整参数 */
    PointCloudFilter& filter() { return *filter_; }
    GroundSegmentation& groundSeg() { return *ground_seg_; }
    Clustering& clusterAlgo() { return *clustering_; }
    LandingZoneDetector& landingZone() { return *landing_zone_; }

private:
    FilterParams           filter_params_;     // 滤波参数
    GroundSegParams        ground_seg_params_; // 地面分割参数
    ClusterParams          cluster_params_;    // 聚类参数
    LandingZoneParams      lz_params_;         // 着陆区参数

    std::unique_ptr<PointCloudFilter>      filter_;        // 滤波器
    std::unique_ptr<GroundSegmentation>    ground_seg_;    // 地面分割器
    std::unique_ptr<Clustering>            clustering_;    // 聚类器
    std::unique_ptr<LandingZoneDetector>   landing_zone_;  // 着陆区检测器

    /* 计算时间差（毫秒） */
    double elapsedMs(const std::chrono::steady_clock::time_point& start) {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    }

    /**
     * 生成着色点云 — 将地面和障碍物合并，用intensity编码类型
     *
     * intensity值约定:
     *   100 = 地面点 (Python GUI 显示为灰色)
     *   200 = 未知障碍物 (黄色)
     *   230 = 车辆 (红色)
     *   260 = 行人 (绿色)
     *   290 = 建筑 (蓝色)
     *   320 = 电线杆 (橙色)
     *   350 = 树木 (深绿)
     *
     * Python GUI (sensors_collect_v3.py) 根据intensity值区间着色
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr generateColoredCloud(
        const PipelineOutput& output) {

        auto colored = pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>());

        size_t reserve_size = (output.ground_cloud ? output.ground_cloud->size() : 0)
                            + (output.nonground_cloud ? output.nonground_cloud->size() : 0);
        colored->reserve(reserve_size);

        // 地面点 → intensity=100
        if (output.ground_cloud) {
            for (const auto& pt : output.ground_cloud->points) {
                pcl::PointXYZI p = pt;
                p.intensity = 100.0f;
                colored->push_back(p);
            }
        }

        // 障碍物点 → intensity=按类型着色
        for (const auto& cluster : output.obstacles) {
            float base_intensity = 200.0f + static_cast<float>(cluster.type) * 30.0f;
            if (cluster.cloud) {
                for (const auto& pt : cluster.cloud->points) {
                    pcl::PointXYZI p = pt;
                    p.intensity = base_intensity;
                    colored->push_back(p);
                }
            }
        }

        return colored;
    }
};

/**
 * @brief 将障碍物聚类结果转换为ROS MarkerArray
 *
 * 每个障碍物生成一个半透明立方体Marker，位置和大小等于包围盒，
 * 颜色按障碍物类型设置:
 *   车辆=红, 行人=绿, 建筑=蓝, 电线杆=橙, 树木=深绿
 *
 * Python GUI 订阅 /obstacle_markers 话题后，用pyqtgraph绘制这些边界盒
 */
inline visualization_msgs::MarkerArray obstaclesToMarkers(
    const std::vector<Cluster>& obstacles,
    const std::string& frame_id) {

    visualization_msgs::MarkerArray markers;
    int id = 0;

    for (const auto& obs : obstacles) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacles";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // 障碍物中心位置
        marker.pose.position.x = obs.bbox.center.x();
        marker.pose.position.y = obs.bbox.center.y();
        marker.pose.position.z = obs.bbox.center.z();

        // 障碍物尺寸（最小0.1m防止看不见）
        marker.scale.x = std::max(obs.bbox.dimensions.x(), 0.1f);
        marker.scale.y = std::max(obs.bbox.dimensions.y(), 0.1f);
        marker.scale.z = std::max(obs.bbox.dimensions.z(), 0.1f);

        // 按障碍物类型设置颜色
        switch (obs.type) {
            case ObstacleType::VEHICLE:  // 车辆=红色
                marker.color.r = 1.0f; marker.color.g = 0.2f; marker.color.b = 0.2f; break;
            case ObstacleType::PERSON:   // 行人=绿色
                marker.color.r = 0.2f; marker.color.g = 1.0f; marker.color.b = 0.2f; break;
            case ObstacleType::BUILDING: // 建筑=蓝色
                marker.color.r = 0.5f; marker.color.g = 0.5f; marker.color.b = 1.0f; break;
            case ObstacleType::POLE:     // 电线杆=橙色
                marker.color.r = 1.0f; marker.color.g = 0.8f; marker.color.b = 0.0f; break;
            case ObstacleType::TREE:     // 树木=深绿
                marker.color.r = 0.0f; marker.color.g = 0.8f; marker.color.b = 0.0f; break;
            default:                     // 其他=黄色
                marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; break;
        }
        marker.color.a = 0.6f;  // 半透明
        marker.lifetime = ros::Duration(0.3f);  // 0.3秒后自动消失（需要持续刷新）

        markers.markers.push_back(marker);
    }

    return markers;
}

/**
 * @brief 将着陆区转换为ROS Marker（绿色圆盘）
 *
 * Python GUI 订阅 /landing_zone_marker 后，
 * 用pyqtgraph绘制绿色圆盘表示安全着陆区
 */
inline visualization_msgs::Marker landingZoneToMarker(
    const LandingZoneResult& lz,
    const std::string& frame_id) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "landing_zone";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = lz.center_x;
    marker.pose.position.y = lz.center_y;
    marker.pose.position.z = lz.center_z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 3.0;  // 直径3米
    marker.scale.y = 3.0;
    marker.scale.z = 0.1;  // 高度0.1米（扁平圆盘）

    marker.color.g = 1.0f;  // 绿色
    marker.color.a = 0.3f;  // 半透明
    marker.lifetime = ros::Duration(1.0);

    return marker;
}

} // namespace algorithm
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_PIPELINE_PROCESSOR_H
