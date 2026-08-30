#include <gtest/gtest.h>
#include <lslidar_ls_driver/algorithm/ground_segmentation.h>

using namespace lslidar_ch_driver::algorithm;

TEST(GroundSegTest, EmptyCloud) {
    GroundSegmentation seg;
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    auto result = seg.segment(cloud);
    EXPECT_FALSE(result.has_ground);
    EXPECT_TRUE(result.nonground_cloud->empty());
}

TEST(GroundSegTest, SimpleGround) {
    GroundSegmentation seg;
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    
    // 添加地面点 (z=0平面)
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 10; ++j) {
            pcl::PointXYZI pt;
            pt.x = (i % 10) * 0.5;
            pt.y = (i / 10) * 0.5;
            pt.z = 0.0;
            pt.intensity = 50;
            cloud->push_back(pt);
        }
    }
    // 添加非地面点
    for (int i = 0; i < 10; ++i) {
        pcl::PointXYZI pt;
        pt.x = 1.0; pt.y = 0.0; pt.z = 1.5;
        cloud->push_back(pt);
    }

    auto result = seg.segment(cloud);
    EXPECT_TRUE(result.has_ground);
    EXPECT_GT(result.ground_cloud->size(), 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
