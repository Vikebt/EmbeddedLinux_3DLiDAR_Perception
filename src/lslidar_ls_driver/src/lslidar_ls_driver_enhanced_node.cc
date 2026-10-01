/******************************************************************************
 * lslidar_ls_driver_enhanced_node.cc — 增强版激光雷达驱动节点
 *
 * 功能: 启动增强版驱动，支持多级Pipeline处理、资源监控、性能分析。
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#include <lslidar_ls_driver/driver/lslidar_ls_driver_enhanced.h>
#include <csignal>

volatile sig_atomic_t flag = 1;

static void signalHandler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        flag = 0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lslidar_ls_driver_enhanced_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("===========================================================");
    ROS_INFO("  LS3 LiDAR Enhanced Driver - Version 2.0");
    ROS_INFO("  Author: Zhou Cong");
    ROS_INFO("  Pipeline: Filter -> GroundSeg -> Clustering -> Visualize");
    ROS_INFO("  Features: Multi-thread, Resource Monitor, Profiler");
    ROS_INFO("===========================================================");

    lslidar_ch_driver::EnhancedLslidarDriver driver(nh, private_nh);
    if (!driver.initialize()) {
        ROS_ERROR("Failed to initialize enhanced LiDAR driver.");
        return 1;
    }

    ROS_INFO("Enhanced driver started. Processing point cloud pipeline...");
    driver.spin();

    ROS_INFO("Enhanced driver shutting down...");
    driver.shutdown();
    return 0;
}
