/******************************************************************************
 * ground_segmentation.h — 地面分割算法模块
 *
 * 功能: 使用RANSAC平面拟合实现地面点云分割，支持多种地面模型
 *       （平面/斜面），并评估地面平坦度。
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_GROUND_SEGMENTATION_H
#define LSLIDAR_LS_GROUND_SEGMENTATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <ros/ros.h>

namespace lslidar_ch_driver {
namespace algorithm {

/**
 * @brief 地面分割参数
 */
struct GroundSegParams {
    double distance_threshold{0.25};       // RANSAC距离阈值 (米)
    int    max_iterations{100};             // 最大迭代次数
    double ground_normal_z_min{0.7};        // 地面法向量Z分量最小值 (过滤非水平面)
    bool   enable_multi_plane{false};       // 是否启用多平面分割
    int    max_planes{3};                   // 最大平面数量
    // 坡度适应
    bool   enable_slope_adaptation{true};
    double max_slope_angle_deg{15.0};       // 最大坡度角度
};

/**
 * @brief 地面分割结果
 */
struct GroundSegResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_cloud;
    pcl::ModelCoefficients::Ptr         coefficients;
    bool                                 has_ground{false};
    double                               ground_height{0.0};
    double                               ground_plane_error{0.0};
    double                               ground_slope_deg{0.0};  // 地面坡度

    GroundSegResult()
        : ground_cloud(new pcl::PointCloud<pcl::PointXYZI>())
        , nonground_cloud(new pcl::PointCloud<pcl::PointXYZI>())
        , coefficients(new pcl::ModelCoefficients()) {}
};

/**
 * @brief 地面分割器类
 *
 * 使用RANSAC进行地面平面分割，并评估地面质量指标。
 */
class GroundSegmentation {
public:
    explicit GroundSegmentation(const GroundSegParams& params = GroundSegParams())
        : params_(params) {}

    /**
     * @brief 执行地面分割
     * @param cloud 输入点云
     * @return 分割结果（地面/非地面点云、平面系数、质量指标）
     */
    GroundSegResult segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        GroundSegResult result;

        if (!cloud || cloud->empty()) return result;

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(params_.distance_threshold);
        seg.setMaxIterations(params_.max_iterations);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *result.coefficients);

        if (inliers->indices.empty()) {
            // 未检测到地面，全部视为非地面
            *result.nonground_cloud = *cloud;
            return result;
        }

        // 检查地面法向量是否朝上
        if (!isValidGroundPlane(result.coefficients)) {
            *result.nonground_cloud = *cloud;
            return result;
        }

        result.has_ground = true;
        result.ground_height = computeGroundHeight(result.coefficients);

        // 计算地面坡度
        result.ground_slope_deg = computeSlopeAngle(result.coefficients);

        // 提取地面点
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.filter(*result.ground_cloud);

        // 提取非地面点
        extract.setNegative(true);
        extract.filter(*result.nonground_cloud);

        // 计算地面平面拟合误差
        result.ground_plane_error = computePlaneError(cloud, inliers, result.coefficients);

        // 多平面分割（可选）
        if (params_.enable_multi_plane && !result.nonground_cloud->empty()) {
            auto additional = segmentAdditionalPlanes(result.nonground_cloud);
            // Merge additional ground points
            *result.ground_cloud += *additional.ground_cloud;
            // Update nonground cloud
            pcl::PointCloud<pcl::PointXYZI> temp;
            pcl::ExtractIndices<pcl::PointXYZI> ext2;
            ext2.setInputCloud(result.nonground_cloud);
            ext2.setIndices(inliers);
            ext2.setNegative(true);
            ext2.filter(temp);
            *result.nonground_cloud = temp;
        }

        return result;
    }

    void setParams(const GroundSegParams& params) { params_ = params; }
    const GroundSegParams& getParams() const { return params_; }

private:
    GroundSegParams params_;

    /**
     * @brief 检查一个平面是否是有效的地面
     * 条件：法向量Z分量足够大（接近水平地面）
     */
    bool isValidGroundPlane(const pcl::ModelCoefficients::Ptr& coeff) {
        if (!coeff) return false;
        // 平面方程: ax + by + cz + d = 0
        double a = coeff->values[0];
        double b = coeff->values[1];
        double c = coeff->values[2];
        double norm = std::sqrt(a * a + b * b + c * c);
        if (norm < 1e-6) return false;

        double normal_z = std::abs(c) / norm;

        if (params_.enable_slope_adaptation) {
            double max_z = std::cos(params_.max_slope_angle_deg * M_PI / 180.0);
            return normal_z >= max_z;
        }

        return normal_z >= params_.ground_normal_z_min;
    }

    double computeGroundHeight(const pcl::ModelCoefficients::Ptr& coeff) {
        // 计算平面在原点处的高度: z = -d/c (当 a*x + b*y + c*z + d = 0, x=y=0)
        if (std::abs(coeff->values[2]) < 1e-6) return 0.0;
        return -coeff->values[3] / coeff->values[2];
    }

    double computeSlopeAngle(const pcl::ModelCoefficients::Ptr& coeff) {
        double a = coeff->values[0];
        double b = coeff->values[1];
        double c = coeff->values[2];
        double norm = std::sqrt(a * a + b * b + c * c);
        if (norm < 1e-6) return 0.0;
        // 坡度 = 90 - 法向量与Z轴的夹角
        double cos_theta = std::abs(c) / norm;
        return 90.0 - std::acos(cos_theta) * 180.0 / M_PI;
    }

    double computePlaneError(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const pcl::PointIndices::Ptr& inliers,
        const pcl::ModelCoefficients::Ptr& coeff) {

        if (!coeff || inliers->indices.empty()) return 0.0;

        double a = coeff->values[0];
        double b = coeff->values[1];
        double c = coeff->values[2];
        double d = coeff->values[3];
        double norm = std::sqrt(a * a + b * b + c * c);
        if (norm < 1e-6) return 0.0;

        double sum_error = 0.0;
        for (const auto& idx : inliers->indices) {
            const auto& pt = cloud->points[idx];
            double dist = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / norm;
            sum_error += dist;
        }
        return sum_error / inliers->indices.size();
    }

    GroundSegResult segmentAdditionalPlanes(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        GroundSegResult result;
        auto remaining = cloud;

        for (int i = 0; i < params_.max_planes - 1; ++i) {
            auto partial = segment(remaining);
            if (!partial.has_ground) break;
            *result.ground_cloud += *partial.ground_cloud;
            remaining = partial.nonground_cloud;
        }

        *result.nonground_cloud = *remaining;
        return result;
    }
};

} // namespace algorithm
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_GROUND_SEGMENTATION_H
