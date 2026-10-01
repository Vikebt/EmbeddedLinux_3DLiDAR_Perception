#ifndef LSLIDAR_POINT_CLOUD_FILTER_H
#define LSLIDAR_POINT_CLOUD_FILTER_H

#include <cmath>
#include <memory>

#include <pcl/common/point_tests.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lslidar_ch_driver {
namespace algorithm {

struct FilterParams {
    bool enable_voxel_grid{true};
    float voxel_leaf_size{0.15F};
    bool enable_statistical_outlier{false};
    int sor_mean_k{20};
    double sor_std_dev_thresh{1.0};
    bool enable_radius_filter{false};
    double radius_search{0.5};
    int radius_min_neighbors{6};
    float range_min{0.5F};
    float range_max{200.0F};
    bool enable_height_filter{false};
    float height_min{-5.0F};
    float height_max{20.0F};
    bool enable_crop_box{false};
    float crop_min_x{-50.0F};
    float crop_max_x{50.0F};
    float crop_min_y{-50.0F};
    float crop_max_y{50.0F};
    float crop_min_z{-5.0F};
    float crop_max_z{20.0F};
};

class PointCloudFilter {
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZI>;
    using CloudPtr = Cloud::Ptr;

    explicit PointCloudFilter(const FilterParams& params = FilterParams())
        : params_(params) {}

    CloudPtr filter(const CloudPtr& input) const {
        CloudPtr selected(new Cloud());
        if (!input) return selected;
        selected->reserve(input->size());

        const float min_sq = params_.range_min * params_.range_min;
        const float max_sq = params_.range_max * params_.range_max;
        for (const auto& point : input->points) {
            if (!pcl::isFinite(point)) continue;
            const float range_sq = point.x * point.x + point.y * point.y + point.z * point.z;
            if (range_sq < min_sq || range_sq > max_sq) continue;
            if (params_.enable_height_filter &&
                (point.z < params_.height_min || point.z > params_.height_max)) continue;
            if (params_.enable_crop_box &&
                (point.x < params_.crop_min_x || point.x > params_.crop_max_x ||
                 point.y < params_.crop_min_y || point.y > params_.crop_max_y ||
                 point.z < params_.crop_min_z || point.z > params_.crop_max_z)) continue;
            selected->push_back(point);
        }
        selected->header = input->header;

        CloudPtr current = selected;
        if (params_.enable_voxel_grid && params_.voxel_leaf_size > 0.0F && !current->empty()) {
            CloudPtr output(new Cloud());
            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(current);
            voxel.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size,
                              params_.voxel_leaf_size);
            voxel.filter(*output);
            current = output;
        }
        if (params_.enable_statistical_outlier &&
            current->size() > static_cast<std::size_t>(params_.sor_mean_k)) {
            CloudPtr output(new Cloud());
            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
            sor.setInputCloud(current);
            sor.setMeanK(params_.sor_mean_k);
            sor.setStddevMulThresh(params_.sor_std_dev_thresh);
            sor.filter(*output);
            current = output;
        }
        if (params_.enable_radius_filter && !current->empty()) {
            CloudPtr output(new Cloud());
            pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius;
            radius.setInputCloud(current);
            radius.setRadiusSearch(params_.radius_search);
            radius.setMinNeighborsInRadius(params_.radius_min_neighbors);
            radius.filter(*output);
            current = output;
        }
        return current;
    }

    void setParams(const FilterParams& params) { params_ = params; }
    const FilterParams& params() const { return params_; }

private:
    FilterParams params_;
};

}  // namespace algorithm
}  // namespace lslidar_ch_driver
#endif
