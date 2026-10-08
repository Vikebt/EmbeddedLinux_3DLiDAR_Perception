/******************************************************************************
 * lslidar_ls_pipeline_node.cc — 点云处理管线节点
 *
 * 【这是整个项目最核心的新增文件】
 *
 * 功能: 订阅雷达原始点云，执行完整感知管线，发布处理结果。
 *       实现 C++ 做实时处理, Python GUI 只做可视化的架构。
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  本节点在系统中的位置:                                       ║
 * ║                                                              ║
 * ║  雷达硬件 ──UDP──→ 驱动节点 ──ROS话题──→ 【本节点】──→ Python GUI ║
 * ║                    (进程1)              (进程2)          (进程3) ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * 订阅的话题:
 *   /lslidar_point_cloud  (sensor_msgs/PointCloud2)  ← 驱动节点发布的原始点云
 *
 * 发布的话题:
 *   /processed_point_cloud   (sensor_msgs/PointCloud2)    → 着色后点云
 *   /obstacle_markers        (visualization_msgs/MarkerArray) → 障碍物边界盒
 *   /landing_zone_marker     (visualization_msgs/Marker)  → 着陆区
 *   /pipeline_diagnostics    (std_msgs/String)            → 性能诊断
 *   /pipeline_fps            (std_msgs/Float32)           → 帧率
 *
 * 处理管线 (从Python算法迁移而来):
 *   Step 1: 体素下采样 voxel_size=1.0    ← 对应原 ThreadRecog.py 中 voxel_down_sample
 *   Step 2: RANSAC地面分割 dist=1.6      ← 对应原 ThreadRecog.py 中 segment_ground_ransac
 *   Step 3: DBSCAN聚类 eps=5.0           ← 对应原 ThreadRecog.py 中 cluster_nonground_points
 *   Step 4: 2.5D着陆区检测               ← 对应原 LandRecog2.py 中 _find_best_landing_zone
 *
 * 编译: catkin_make (集成在CMakeLists.txt中)
 * 运行: rosrun lslidar_ls_driver lslidar_ls_pipeline_node
 *        或通过 lslidar_ls1550_pipeline.launch 统一启动
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/* 引入管线处理器头文件 — 封装了所有算法模块 */
#include <lslidar_ls_driver/algorithm/pipeline_processor.h>

#include <memory>
#include <chrono>
#include <sstream>
#include <csignal>
#include <thread>

using namespace lslidar_ch_driver::algorithm;

// ============================================================================
// 全局变量 — ROS 发布者和管线处理器
// ============================================================================
volatile sig_atomic_t g_running = 1;    // 运行标志，收到SIGINT时置0

/* 五个发布者，对应五个话题 */
ros::Publisher g_processed_cloud_pub;   // 发布着色点云
ros::Publisher g_obstacle_markers_pub;  // 发布障碍物3D边界盒
ros::Publisher g_landing_zone_pub;      // 发布着陆区
ros::Publisher g_diagnostics_pub;       // 发布性能诊断信息
ros::Publisher g_fps_pub;               // 发布管线帧率

/* 管线处理器 — 包含滤波、分割、聚类、着陆区检测四个模块 */
std::unique_ptr<PipelineProcessor> g_pipeline;

/* 帧率统计结构体 */
struct {
    std::chrono::steady_clock::time_point last_time;
    int frame_count{0};
    double fps{0.0};
} g_fps_stats;

// ============================================================================
// 信号处理函数 — 收到 Ctrl+C 时设置退出标志
// ============================================================================
void signalHandler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        g_running = 0;
    }
}

// ============================================================================
// 点云回调 — 每收到一帧原始点云，执行一次管线处理
//
// 这是本节点的核心函数，当驱动节点发布一帧点云到 /lslidar_point_cloud
// ROS 话题时，此函数被自动调用。
//
// 调用链: ROS话题触发 → 本函数 → g_pipeline->process(cloud) → 发布结果
// ============================================================================
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!g_running) return;

    auto total_start = std::chrono::steady_clock::now();

    // ────────────────────────────────────────────────────────────
    // Step 0: 将ROS消息转为PCL点云格式
    // ROS话题传输的是PointCloud2二进制消息，需要转成PCL的PointCloud对象
    // ────────────────────────────────────────────────────────────
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    // ────────────────────────────────────────────────────────────
    // Step 1~4: 执行完整管线处理
    // pipeline_processor.h 中定义了完整管线:
    //   体素滤波 → RANSAC地面分割 → DBSCAN聚类 → 2.5D着陆区检测
    // 这一步是CPU密集型计算，耗时约45ms
    // ────────────────────────────────────────────────────────────
    auto output = g_pipeline->process(cloud);

    // ────────────────────────────────────────────────────────────
    // Step 5: 发布着色点云
    // colored_cloud 中: 地面点(intensity=100), 障碍物点(intensity=按类型)
    // Python GUI 订阅此话题后，根据intensity值着色显示
    // ────────────────────────────────────────────────────────────
    if (output.colored_cloud && !output.colored_cloud->empty()) {
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*output.colored_cloud, out_msg);
        out_msg.header.frame_id = msg->header.frame_id;  // 保持坐标系一致
        out_msg.header.stamp = msg->header.stamp;         // 保持时间戳一致
        g_processed_cloud_pub.publish(out_msg);
    }

    // ────────────────────────────────────────────────────────────
    // Step 6: 发布障碍物边界盒
    // obstaclesToMarkers() 将聚类结果转成ROS Marker
    // 每个障碍物是一个半透明立方体，按类型着色（红=车、绿=人、蓝=建筑）
    // ────────────────────────────────────────────────────────────
    auto markers = obstaclesToMarkers(output.obstacles, msg->header.frame_id);
    g_obstacle_markers_pub.publish(markers);

    // ────────────────────────────────────────────────────────────
    // Step 7: 发布着陆区标记
    // 如果找到了符合条件的安全着陆区，发布绿色圆盘标记
    // ────────────────────────────────────────────────────────────
    if (output.landing_zone.found) {
        auto lz_marker = landingZoneToMarker(
            output.landing_zone, msg->header.frame_id);
        g_landing_zone_pub.publish(lz_marker);
    }

    // ────────────────────────────────────────────────────────────
    // Step 8: 发布诊断信息
    // 格式: "PIPELINE|total=45ms|filter=5ms|groundseg=12ms|..."
    // Python GUI 读取此信息显示在"处理结果"面板上
    // ────────────────────────────────────────────────────────────
    std::ostringstream diag;
    diag << "PIPELINE|"
         << "total=" << output.total_time_ms << "ms|"
         << "filter=" << output.filter_time_ms << "ms|"
         << "groundseg=" << output.ground_seg_time_ms << "ms|"
         << "cluster=" << output.cluster_time_ms << "ms|"
         << "lz=" << output.landing_zone_time_ms << "ms|"
         << "points_in=" << cloud->size() << "|"
         << "points_out="
         << (output.colored_cloud ? output.colored_cloud->size() : 0) << "|"
         << "obstacles=" << output.obstacles.size() << "|"
         << "landing=" << (output.landing_zone.found ? "FOUND" : "NONE");

    std_msgs::String diag_msg;
    diag_msg.data = diag.str();
    g_diagnostics_pub.publish(diag_msg);

    // ────────────────────────────────────────────────────────────
    // Step 9: 帧率统计（每秒更新一次）
    // Python GUI 读取 /pipeline_fps 显示实时帧率
    // ────────────────────────────────────────────────────────────
    g_fps_stats.frame_count++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(
        now - g_fps_stats.last_time).count();

    if (elapsed >= 1.0) {
        g_fps_stats.fps = g_fps_stats.frame_count / elapsed;
        g_fps_stats.frame_count = 0;
        g_fps_stats.last_time = now;

        /* 打印到终端，方便调试时观察 */
        ROS_INFO("[Pipeline] FPS: %.1f | Total: %.1fms | Obs: %zu | LZ: %s",
                 g_fps_stats.fps, output.total_time_ms,
                 output.obstacles.size(),
                 output.landing_zone.found ? "YES" : "NO");

        /* 发布到话题 */
        std_msgs::Float32 fps_msg;
        fps_msg.data = g_fps_stats.fps;
        g_fps_pub.publish(fps_msg);
    }
}

// ============================================================================
// 主函数 — 节点入口
//
// 运行时序:
//   1. 初始化ROS节点
//   2. 创建管线处理器（内部初始化所有算法模块和参数）
//   3. 创建5个话题发布者
//   4. 订阅 /lslidar_point_cloud，注册 pointCloudCallback 回调函数
//   5. 进入 ros::spin() 循环，等待话题消息到来
//   6. 每收到一帧点云 → 调用回调函数 → 处理 → 发布结果
//   7. 收到Ctrl+C → 退出
// ============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "lslidar_ls_pipeline_node",
              ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.setParam("ready", false);

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("===============================================================");
    ROS_INFO("  LS3 LiDAR Pipeline Node - C++ Real-time Processing");
    ROS_INFO("  Pipeline: VoxelFilter -> RANSAC GroundSeg -> DBSCAN Cluster");
    ROS_INFO("         -> 2.5D Landing Zone Detection");
    ROS_INFO("  Published topics:");
    ROS_INFO("    /processed_point_cloud  (colored point cloud)");
    ROS_INFO("    /obstacle_markers       (3D bounding boxes)");
    ROS_INFO("    /landing_zone_marker    (safe landing zone)");
    ROS_INFO("    /pipeline_diagnostics   (performance info)");
    ROS_INFO("===============================================================");

    // 创建管线处理器（算法参数已在构造函数中设置，与Python版一致）
    g_pipeline = std::make_unique<PipelineProcessor>();

    // 创建5个ROS发布者
    g_processed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
        "processed_point_cloud", 5);    // queue_size=5，超过丢弃旧的
    g_obstacle_markers_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "obstacle_markers", 5);
    g_landing_zone_pub = nh.advertise<visualization_msgs::Marker>(
        "landing_zone_marker", 5);
    g_diagnostics_pub = nh.advertise<std_msgs::String>(
        "pipeline_diagnostics", 10);
    g_fps_pub = nh.advertise<std_msgs::Float32>(
        "pipeline_fps", 10);

    // 订阅原始点云话题 — 当驱动节点发布点云时，pointCloudCallback会被调用
    std::string input_topic = "lslidar_point_cloud";
    pnh.param<std::string>("input_topic", input_topic, "lslidar_point_cloud");
    ros::Subscriber cloud_sub = nh.subscribe(
        input_topic, 5, pointCloudCallback);

    // 初始化FPS统计
    g_fps_stats.last_time = std::chrono::steady_clock::now();

    ROS_INFO("Pipeline node started. Subscribing to: %s", input_topic.c_str());
    pnh.setParam("ready", true);

    while (ros::ok() && g_running) {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    pnh.setParam("ready", false);
    ros::shutdown();
    ROS_INFO("Pipeline node shutdown.");
    return 0;
}
