#include <gtest/gtest.h>

#include <lslidar_ls_driver/lslidar_ls_driver.h>

#include <csignal>

// The input library refers to the executable's signal flag, as the existing
// driver nodes do. This test intentionally never opens a LiDAR input.
volatile sig_atomic_t flag = 1;

TEST(DriverLifecycle, UninitializedDriverCanBeDestroyed) {
    ros::M_string remappings;
    ros::init(remappings, "driver_lifecycle_test");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    {
        lslidar_ch_driver::LslidarChDriver driver(nh, private_nh);
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
