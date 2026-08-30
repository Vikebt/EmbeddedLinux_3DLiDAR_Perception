/******************************************************************************
 * landing_zone.h — 2.5D栅格着陆区检测算法
 *
 * 功能: 从 LandRecog2.py (Python) 迁移到 C++
 *       使用2.5D栅格地图评估地面平坦度，搜索最佳安全着陆区域。
 *
 * 算法步骤:
 *   1. 将3D点云投影到2D栅格，每个栅格统计点云Z值分布
 *   2. 筛选平坦栅格 (std_dev < threshold, range < threshold)
 *   3. 使用圆形掩模搜索候选着陆区
 *   4. 选择离原点最近的有效着陆区
 *
 * 作者: 周聪 (从LandRecog2.py迁移)
 * 日期: 2025
 *****************************************************************************/

#ifndef LSLIDAR_LS_LANDING_ZONE_H
#define LSLIDAR_LS_LANDING_ZONE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cstdint>

namespace lslidar_ch_driver {
namespace algorithm {

/**
 * @brief 着陆区检测参数（与LandRecog2.py保持一致）
 */
struct LandingZoneParams {
    double grid_resolution{1.5};             // 栅格分辨率(米)
    double landing_zone_radius{1.5};         // 着陆区半径(米)
    double max_cell_std_dev{0.08};           // 最大栅格高度标准差(米)
    double max_cell_range{0.15};             // 最大栅格高度范围(米)
    double max_zone_planarity_std_dev{0.10}; // 最大区域平面性标准差
    int    min_points_per_cell{5};           // 栅格最少点数
};

/**
 * @brief 栅格统计信息
 */
struct CellStats {
    int    count{0};
    double min_z{1e9};
    double max_z{-1e9};
    double avg_z{0.0};
    double std_dev_z{0.0};
    double range_z{0.0};
};

/**
 * @brief 着陆区结果
 */
struct LandingZoneResult {
    bool   found{false};
    double center_x{0.0};
    double center_y{0.0};
    double center_z{0.0};
    double distance{0.0};         // 到原点的距离
    double planarity{0.0};        // 平面性标准差
    int    num_flat_cells{0};     // 平坦栅格数
    int    num_candidates{0};     // 候选区域数

    std::string toString() const {
        if (!found) return "No landing zone found";
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "LandingZone: center=(%.2f, %.2f, %.2f) dist=%.2fm planarity=%.3f",
                 center_x, center_y, center_z, distance, planarity);
        return std::string(buf);
    }
};

/**
 * @brief 2.5D栅格着陆区检测器
 *
 * 精确迁移自 LandRecog2.py
 */
class LandingZoneDetector {
public:
    explicit LandingZoneDetector(const LandingZoneParams& params = LandingZoneParams())
        : params_(params) {

        // 预计算圆形掩模
        cells_in_radius_ = static_cast<int>(
            std::ceil(params_.landing_zone_radius / params_.grid_resolution));
        circular_mask_ = createCircularMask(cells_in_radius_);
    }

    /**
     * @brief 执行着陆区检测
     * @param cloud 输入点云（应包含地面点）
     * @return 检测结果
     */
    LandingZoneResult detect(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        LandingZoneResult result;

        if (!cloud || cloud->empty()) return result;

        // Step 1: 创建2.5D栅格统计地图
        auto grid_stats = create25DGridStats(cloud);
        if (grid_stats.empty()) return result;

        // Step 2: 筛选平坦栅格
        auto flat_cells = filterFlatCells(grid_stats);
        result.num_flat_cells = static_cast<int>(flat_cells.size());
        if (flat_cells.empty()) return result;

        // Step 3: 搜索着陆区
        auto zones = findLandingZones(flat_cells);
        result.num_candidates = static_cast<int>(zones.size());
        if (zones.empty()) return result;

        // Step 4: 选择离原点最近的着陆区
        auto best = std::min_element(zones.begin(), zones.end(),
            [](const LandingZoneResult& a, const LandingZoneResult& b) {
                return a.distance < b.distance;
            });

        result = *best;
        result.found = true;
        return result;
    }

    void setParams(const LandingZoneParams& params) { params_ = params; }
    const LandingZoneParams& getParams() const { return params_; }

private:
    LandingZoneParams params_;
    int cells_in_radius_;
    std::vector<std::vector<bool>> circular_mask_;

    // 栅格索引 key: (ix << 32) | iy
    using GridKey = int64_t;
    static GridKey makeKey(int ix, int iy) {
        return (static_cast<int64_t>(ix) << 32) | static_cast<uint32_t>(iy);
    }

    /**
     * @brief 创建圆形掩模（与LandRecog2.py一致）
     */
    std::vector<std::vector<bool>> createCircularMask(int radius) {
        int diameter = 2 * radius + 1;
        std::vector<std::vector<bool>> mask(diameter, std::vector<bool>(diameter, false));

        for (int y = -radius; y <= radius; ++y) {
            for (int x = -radius; x <= radius; ++x) {
                if (x * x + y * y <= radius * radius) {
                    mask[y + radius][x + radius] = true;
                }
            }
        }
        return mask;
    }

    /**
     * @brief 创建2.5D栅格统计地图（与LandRecog2._create_25d_grid_stats()一致）
     */
    std::unordered_map<GridKey, CellStats> create25DGridStats(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {

        std::unordered_map<GridKey, std::vector<double>> cell_z_values;

        // 收集每个栅格的Z值
        for (const auto& pt : cloud->points) {
            int ix = static_cast<int>(std::floor(pt.x / params_.grid_resolution));
            int iy = static_cast<int>(std::floor(pt.y / params_.grid_resolution));
            cell_z_values[makeKey(ix, iy)].push_back(pt.z);
        }

        // 计算统计量
        std::unordered_map<GridKey, CellStats> grid_stats;
        for (auto& [key, z_vals] : cell_z_values) {
            if (static_cast<int>(z_vals.size()) < params_.min_points_per_cell) {
                continue;
            }

            CellStats stats;
            stats.count = static_cast<int>(z_vals.size());

            double sum = 0.0, sum_sq = 0.0;
            stats.min_z = *std::min_element(z_vals.begin(), z_vals.end());
            stats.max_z = *std::max_element(z_vals.begin(), z_vals.end());

            for (double z : z_vals) {
                sum += z;
                sum_sq += z * z;
            }

            stats.avg_z = sum / z_vals.size();
            stats.range_z = stats.max_z - stats.min_z;
            stats.std_dev_z = std::sqrt(
                (sum_sq / z_vals.size()) - (stats.avg_z * stats.avg_z));

            grid_stats[key] = stats;
        }

        return grid_stats;
    }

    /**
     * @brief 筛选平坦栅格（与LandRecog2逻辑一致）
     */
    std::unordered_map<GridKey, CellStats> filterFlatCells(
        const std::unordered_map<GridKey, CellStats>& grid_stats) {

        std::unordered_map<GridKey, CellStats> flat;
        for (const auto& [key, stats] : grid_stats) {
            if (stats.std_dev_z < params_.max_cell_std_dev &&
                stats.range_z < params_.max_cell_range) {
                flat[key] = stats;
            }
        }
        return flat;
    }

    /**
     * @brief 搜索候选着陆区（与LandRecog2._find_best_landing_zone()一致）
     */
    std::vector<LandingZoneResult> findLandingZones(
        const std::unordered_map<GridKey, CellStats>& flat_cells) {

        std::vector<LandingZoneResult> valid_zones;
        int radius = cells_in_radius_;

        // 获取掩模偏移量
        std::vector<std::pair<int, int>> mask_offsets;
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (circular_mask_[dy + radius][dx + radius]) {
                    mask_offsets.emplace_back(dx, dy);
                }
            }
        }

        for (const auto& [center_key, center_stats] : flat_cells) {
            int cx = static_cast<int>(center_key >> 32);
            int cy = static_cast<int>(center_key & 0xFFFFFFFF);

            double center_x = (cx + 0.5) * params_.grid_resolution;
            double center_y = (cy + 0.5) * params_.grid_resolution;

            // 检查圆形区域内所有栅格是否都是平坦的
            bool valid = true;
            std::vector<double> avg_z_list;

            for (const auto& [dx, dy] : mask_offsets) {
                GridKey key = makeKey(cx + dx, cy + dy);
                auto it = flat_cells.find(key);
                if (it == flat_cells.end()) {
                    valid = false;
                    break;
                }
                avg_z_list.push_back(it->second.avg_z);
            }

            if (!valid) continue;

            // 检查区域平面性
            if (avg_z_list.size() > 1) {
                double sum = 0.0, sum_sq = 0.0;
                for (double z : avg_z_list) {
                    sum += z;
                    sum_sq += z * z;
                }
                double mean = sum / avg_z_list.size();
                double planarity = std::sqrt(
                    (sum_sq / avg_z_list.size()) - (mean * mean));

                if (planarity < params_.max_zone_planarity_std_dev) {
                    LandingZoneResult zone;
                    zone.center_x = center_x;
                    zone.center_y = center_y;
                    zone.center_z = mean;
                    zone.distance = std::sqrt(center_x * center_x + center_y * center_y);
                    zone.planarity = planarity;
                    valid_zones.push_back(zone);
                }
            }
        }

        return valid_zones;
    }
};

} // namespace algorithm
} // namespace lslidar_ch_driver

#endif // LSLIDAR_LS_LANDING_ZONE_H
