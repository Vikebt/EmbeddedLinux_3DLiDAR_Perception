/******************************************************************************
 * lslidar_ls_driver_enhanced_node.cc — 增强版激光雷达驱动节点
 *
 * 功能: 启动增强版驱动，支持多级Pipeline处理、资源监控、性能分析。
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#include <lslidar_ls_driver/driver/lslidar_ls_driver_enhanced.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

volatile sig_atomic_t flag = 1;

static void signalHandler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        flag = 0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lslidar_ls_driver_enhanced_node",
              ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.setParam("ready", false);

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::atomic<bool> stop_watcher{false};
    std::thread shutdown_watcher([&stop_watcher] {
        while (!stop_watcher.load() && flag && ros::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!flag) {
            ros::shutdown();
        }
    });

    ROS_INFO("===========================================================");
    ROS_INFO("  LS3 LiDAR Enhanced Driver - Version 2.0");
    ROS_INFO("  Author: Zhou Cong");
    ROS_INFO("  Pipeline: Filter -> GroundSeg -> Clustering -> Visualize");
    ROS_INFO("  Features: Multi-thread, Resource Monitor, Profiler");
    ROS_INFO("===========================================================");

    lslidar_ch_driver::EnhancedLslidarDriver driver(nh, private_nh);
    if (!driver.initialize()) {
        const int exit_code = flag ? 1 : 0;
        if (flag) {
            ROS_ERROR("Failed to initialize enhanced LiDAR driver.");
        }
        stop_watcher.store(true);
        shutdown_watcher.join();
        return exit_code;
    }

    private_nh.setParam("ready", true);
    ROS_INFO("Enhanced driver started. Processing point cloud pipeline...");
    driver.spin();

    private_nh.setParam("ready", false);
    ROS_INFO("Enhanced driver shutting down...");
    driver.shutdown();
    stop_watcher.store(true);
    shutdown_watcher.join();
    return 0;
}
