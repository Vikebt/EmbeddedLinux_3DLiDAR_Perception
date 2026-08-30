#include <gtest/gtest.h>
#include <lslidar_ls_driver/algorithm/clustering.h>

using namespace lslidar_ch_driver::algorithm;

TEST(ClusteringTest, EmptyCloud) {
    Clustering clusterer;
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    auto clusters = clusterer.cluster(cloud);
    EXPECT_TRUE(clusters.empty());
}

TEST(ClusteringTest, SingleCluster) {
    Clustering clusterer;
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    
    // 添加一个密集聚类
    for (int i = 0; i < 50; ++i) {
        pcl::PointXYZI pt;
        pt.x = 1.0 + (rand() % 100) / 1000.0;
        pt.y = 2.0 + (rand() % 100) / 1000.0;
        pt.z = 0.5 + (rand() % 100) / 1000.0;
        cloud->push_back(pt);
    }

    auto clusters = clusterer.cluster(cloud);
    EXPECT_GE(clusters.size(), 1);

    if (!clusters.empty()) {
        EXPECT_NEAR(clusters[0].centroid.x(), 1.05, 0.1);
        EXPECT_NEAR(clusters[0].centroid.y(), 2.05, 0.1);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
