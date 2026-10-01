#include <gtest/gtest.h>
#include <lslidar_ls_driver/algorithm/filter.h>

using lslidar_ch_driver::algorithm::FilterParams;
using lslidar_ch_driver::algorithm::PointCloudFilter;

TEST(FilterTest, HandlesNullAndEmptyCloud) {
    PointCloudFilter filter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr null_cloud;
    EXPECT_TRUE(filter.filter(null_cloud)->empty());
    auto empty = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    EXPECT_TRUE(filter.filter(empty)->empty());
}

TEST(FilterTest, AppliesRangeAndHeightBounds) {
    FilterParams params;
    params.enable_voxel_grid = false;
    params.range_min = 1.0F;
    params.range_max = 10.0F;
    params.enable_height_filter = true;
    params.height_min = -1.0F;
    params.height_max = 2.0F;
    PointCloudFilter filter(params);

    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointXYZI point;
    point.x = 2.0F; point.y = 0.0F; point.z = 0.0F; cloud->push_back(point);
    point.x = 0.1F; cloud->push_back(point);
    point.x = 2.0F; point.z = 3.0F; cloud->push_back(point);

    auto output = filter.filter(cloud);
    ASSERT_EQ(output->size(), 1U);
    EXPECT_FLOAT_EQ(output->front().x, 2.0F);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
