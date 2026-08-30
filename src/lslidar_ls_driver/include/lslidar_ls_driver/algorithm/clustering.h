/******************************************************************************
 * clustering.h — 点云聚类算法模块
 *
 * 功能: 实现基于欧式聚类(DBSCAN)的点云分割，计算聚类包围盒
 *       和基本几何属性，支持障碍物/车辆的区分和分类。
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_CLUSTERING_H
#define LSLIDAR_LS_CLUSTERING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace lslidar_ch_driver {
namespace algorithm {

/**
 * @brief 聚类参数
 */
struct ClusterParams {
    double cluster_tolerance{0.5};          // 聚类半径 (米)
    int    min_cluster_size{20};             // 最小聚类点数
    int    max_cluster_size{25000};          // 最大聚类点数
    bool   enable_multi_level{false};        // 多尺度聚类
    double cluster_tolerance_fine{0.3};      // 精细聚类半径
    int    min_cluster_size_coarse{50};      // 粗聚类最小点数
};

/**
 * @brief 聚类边界框
 */
struct BoundingBox {
    Eigen::Vector3f center{Eigen::Vector3f::Zero()};
    Eigen::Vector3f dimensions{Eigen::Vector3f::Zero()};   // width, depth, height
    Eigen::Vector3f min_point{Eigen::Vector3f::Zero()};
    Eigen::Vector3f max_point{Eigen::Vector3f::Zero()};
    float           min_height{0.0f};
    float           max_height{0.0f};
    float           orientation{0.0f};                      // 朝向角(度)
};

/**
 * @brief 障碍物类型枚举
 */
enum class ObstacleType : uint8_t {
    UNKNOWN     = 0,
    VEHICLE     = 1,    // 车辆
    PERSON      = 2,    // 行人
    BICYCLE     = 3,    // 自行车
    BUILDING    = 4,    // 建筑物
    POLE        = 5,    // 电线杆
    TREE        = 6,    // 树木
    TERRAIN     = 7,    // 地形
};

/**
 * @brief 聚类结果 - 单个障碍物
 */
struct Cluster {
    std::vector<int>           point_indices;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    BoundingBox                bbox;
    ObstacleType               type{ObstacleType::UNKNOWN};
    int                        id{-1};
    int                        num_points{0};
    float                      confidence{0.0f};
    Eigen::Vector3f            centroid{Eigen::Vector3f::Zero()};
    float                      avg_height{0.0f};
    float                      point_density{0.0f};          // points/m^3

    // 类型名称
    std::string typeName() const {
        switch (type) {
            case ObstacleType::VEHICLE:  return "Vehicle";
            case ObstacleType::PERSON:   return "Person";
            case ObstacleType::BICYCLE:  return "Bicycle";
            case ObstacleType::BUILDING: return "Building";
            case ObstacleType::POLE:     return "Pole";
            case ObstacleType::TREE:     return "Tree";
            case ObstacleType::TERRAIN:  return "Terrain";
            default:                     return "Unknown";
        }
    }
};

/**
 * @brief 聚类器
 *
 * 对非地面点云进行欧式聚类，计算每个障碍物的边界框和类型。
 */
class Clustering {
public:
    explicit Clustering(const ClusterParams& params = ClusterParams())
        : params_(params) {}

    /**
     * @brief 执行聚类
     * @param cloud 输入点云（非地面点）
     * @return 聚类列表
     */
    std::vector<Cluster> cluster(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

        std::vector<Cluster> clusters;

        if (!cloud || cloud->empty()) return clusters;

        // 创建KD-Tree搜索
        auto tree = pcl::search::KdTree<pcl::PointXYZI>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(cloud);

        // 欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(params_.cluster_tolerance);
        ec.setMinClusterSize(params_.min_cluster_size);
        ec.setMaxClusterSize(params_.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 构建每个聚类的详细信息
        for (size_t i = 0; i < cluster_indices.size(); ++i) {
            Cluster cluster;
            cluster.id = static_cast<int>(i);
            cluster.point_indices = cluster_indices[i].indices;
            cluster.num_points = static_cast<int>(cluster.point_indices.size());

            // 提取聚类点云
            cluster.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
                new pcl::PointCloud<pcl::PointXYZI>());
            cluster.cloud->reserve(cluster.num_points);
            for (const auto& idx : cluster.point_indices) {
                cluster.cloud->push_back(cloud->points[idx]);
            }

            // 计算边界框
            cluster.bbox = computeBoundingBox(cluster.cloud);

            // 计算质心
            cluster.centroid = computeCentroid(cluster.cloud);

            // 计算平均高度
            cluster.avg_height = computeAvgHeight(cluster.cloud);

            // 计算点密度
            cluster.point_density = computePointDensity(cluster);

            // 识别障碍物类型
            cluster.type = classifyObstacle(cluster);

            clusters.push_back(cluster);
        }

        return clusters;
    }

    void setParams(const ClusterParams& params) { params_ = params; }
    const ClusterParams& getParams() const { return params_; }

private:
    ClusterParams params_;

    BoundingBox computeBoundingBox(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

        BoundingBox bbox;
        if (cloud->empty()) return bbox;

        bbox.min_point = cloud->points[0].getVector3fMap();
        bbox.max_point = cloud->points[0].getVector3fMap();

        for (const auto& pt : cloud->points) {
            bbox.min_point = bbox.min_point.cwiseMin(pt.getVector3fMap());
            bbox.max_point = bbox.max_point.cwiseMax(pt.getVector3fMap());
        }

        bbox.center = (bbox.min_point + bbox.max_point) * 0.5f;
        bbox.dimensions = bbox.max_point - bbox.min_point;
        bbox.min_height = bbox.min_point.z();
        bbox.max_height = bbox.max_point.z();

        // 简单朝向估计（使用PCA）
        if (cloud->size() >= 5) {
            bbox.orientation = estimateOrientation(cloud);
        }

        return bbox;
    }

    float estimateOrientation(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();

        for (const auto& pt : cloud->points) {
            mean += pt.getVector3fMap();
        }
        mean /= static_cast<float>(cloud->size());

        for (const auto& pt : cloud->points) {
            Eigen::Vector3f diff = pt.getVector3fMap() - mean;
            cov += diff * diff.transpose();
        }
        cov /= static_cast<float>(cloud->size() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        Eigen::Vector3f eigenvec = solver.eigenvectors().col(2);  // 主方向

        return std::atan2(eigenvec.y(), eigenvec.x()) * 180.0f / M_PI;
    }

    Eigen::Vector3f computeCentroid(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& pt : cloud->points) {
            centroid += pt.getVector3fMap();
        }
        return centroid / static_cast<float>(cloud->size());
    }

    float computeAvgHeight(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

        double sum_z = 0.0;
        for (const auto& pt : cloud->points) {
            sum_z += pt.z;
        }
        return static_cast<float>(sum_z / cloud->size());
    }

    float computePointDensity(const Cluster& cluster) {
        float volume = cluster.bbox.dimensions.prod();
        if (volume < 0.01f) volume = 0.01f;
        return static_cast<float>(cluster.num_points) / volume;
    }

    /**
     * @brief 基于几何特征分类障碍物
     */
    ObstacleType classifyObstacle(const Cluster& cluster) {
        const auto& dim = cluster.bbox.dimensions;
        float len = dim.x();   // 长度
        float wid = dim.y();   // 宽度
        float hgt = dim.z();   // 高度

        // 确保各维度有序
        float max_dim = std::max({len, wid, hgt});
        float mid_dim = std::max(std::min(len, wid),
                                  std::min(std::max(len, wid), hgt));
        float min_dim = std::min({len, wid, hgt});

        // 建筑物：很大且高
        if (max_dim > 15.0f && hgt > 5.0f) {
            return ObstacleType::BUILDING;
        }

        // 车辆：1.5~5m长，0.5~2.5m宽，0.5~2m高
        if (len > 1.5f && len < 10.0f &&
            wid > 0.5f && wid < 3.0f &&
            hgt > 0.5f && hgt < 3.5f &&
            cluster.num_points > 50) {
            return ObstacleType::VEHICLE;
        }

        // 行人：窄且高
        if (max_dim < 1.2f && hgt > 1.0f && hgt < 2.2f &&
            min_dim < 0.8f && cluster.num_points > 10) {
            return ObstacleType::PERSON;
        }

        // 电线杆/树干：细长
        if (max_dim > 2.0f && min_dim < 0.5f && max_dim / (min_dim + 0.1f) > 4.0f) {
            return ObstacleType::POLE;
        }

        // 树木：体量大、点密度高
        if (hgt > 2.0f && wid > 1.0f &&
            cluster.point_density > 100.0f) {
            return ObstacleType::TREE;
        }

        // 地形
        if (hgt < 0.5f && max_dim > 5.0f) {
            return ObstacleType::TERRAIN;
        }

        return ObstacleType::UNKNOWN;
    }
};

} // namespace algorithm
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_CLUSTERING_H
