/******************************************************************************
 * benchmark_pipeline.cpp — 流水线性能基准测试
 *
 * 功能: 对点云处理管线进行压测，评估在不同参数配置下各阶段的
 *       延迟、吞吐量、资源使用情况，生成性能报告。
 *
 * 编译:
 *   g++ -std=c++17 -O3 benchmark_pipeline.cpp -o bench_pipeline \
 *       -lpcl_common -lpcl_filters -lpcl_segmentation -lpcl_search
 *
 * 运行:
 *   ./bench_pipeline --input pointcloud.pcd --iterations 1000
 *
 * 作者: 周聪
 * 日期: 2025
 *****************************************************************************/

#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 引入算法模块
#include <lslidar_ls_driver/algorithm/filter.h>
#include <lslidar_ls_driver/algorithm/ground_segmentation.h>
#include <lslidar_ls_driver/algorithm/clustering.h>
#include <lslidar_ls_driver/algorithm/obstacle_detection.h>

using namespace lslidar_ch_driver::algorithm;

struct BenchmarkResult {
    std::string test_name;
    int iterations;
    double avg_time_ms;
    double min_time_ms;
    double max_time_ms;
    double p50_ms;
    double p95_ms;
    double p99_ms;
    double throughput_hz;
    size_t input_points;
    size_t output_points;
};

void printResults(const std::vector<BenchmarkResult>& results) {
    std::cout << "\n========== Benchmark Results ==========\n";
    std::cout << std::left << std::setw(35) << "Test"
              << std::right << std::setw(10) << "Avg(ms)"
              << std::setw(10) << "Min(ms)"
              << std::setw(10) << "Max(ms)"
              << std::setw(10) << "P99(ms)"
              << std::setw(10) << "Hz"
              << std::setw(10) << "Points" << "\n";
    std::cout << std::string(95, '-') << "\n";
    for (const auto& r : results) {
        std::cout << std::left << std::setw(35) << r.test_name
                  << std::right << std::setw(10) << std::fixed << std::setprecision(2) << r.avg_time_ms
                  << std::setw(10) << r.min_time_ms
                  << std::setw(10) << r.max_time_ms
                  << std::setw(10) << r.p99_ms
                  << std::setw(10) << r.throughput_hz
                  << std::setw(10) << r.input_points << "\n";
    }
    std::cout << "========================================\n";
}

BenchmarkResult benchmarkPipeline(
    const std::string& name,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const PerceptionParams& params,
    int iterations) {

    ObstacleDetector detector(params);
    std::vector<double> times;
    times.reserve(iterations);

    // 预热
    for (int i = 0; i < 10; ++i) {
        detector.detect(cloud);
    }

    // 正式测试
    DetectionResult last_result;
    for (int i = 0; i < iterations; ++i) {
        auto start = std::chrono::steady_clock::now();
        last_result = detector.detect(cloud);
        auto elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        times.push_back(elapsed);
    }

    // 统计
    std::sort(times.begin(), times.end());
    double sum = 0;
    for (double t : times) sum += t;

    BenchmarkResult r;
    r.test_name = name;
    r.iterations = iterations;
    r.avg_time_ms = sum / iterations;
    r.min_time_ms = times.front();
    r.max_time_ms = times.back();
    r.p50_ms = times[times.size() * 0.5];
    r.p95_ms = times[times.size() * 0.95];
    r.p99_ms = times[times.size() * 0.99];
    r.throughput_hz = 1000.0 / r.avg_time_ms;
    r.input_points = cloud->size();
    r.output_points = last_result.filtered_cloud->size();

    return r;
}

int main(int argc, char** argv) {
    std::string input_file = "test_cloud.pcd";
    int iterations = 100;

    // 简单参数解析
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) input_file = argv[++i];
        if (arg == "--iterations" && i + 1 < argc) iterations = std::stoi(argv[++i]);
    }

    // 加载或生成测试点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile(input_file, *cloud) == -1) {
        std::cout << "Cannot load " << input_file << ". Generating synthetic cloud...\n";
        // 生成合成点云: 10000个点随机分布在[-20, 20] x [-20, 20] x [-2, 5]
        cloud->reserve(10000);
        for (int i = 0; i < 10000; ++i) {
            pcl::PointXYZI pt;
            pt.x = (rand() % 4000 - 2000) / 100.0;
            pt.y = (rand() % 4000 - 2000) / 100.0;
            pt.z = (rand() % 700 - 200) / 100.0;
            pt.intensity = (rand() % 255);
            cloud->push_back(pt);
        }
    }
    std::cout << "Benchmark cloud: " << cloud->size() << " points\n\n";

    std::vector<BenchmarkResult> results;

    // 测试1: 默认参数
    {
        PerceptionParams params;
        auto r = benchmarkPipeline("Default Params", cloud, params, iterations);
        results.push_back(r);
    }

    // 测试2: 嵌入式优化参数 (较大体素+较少迭代)
    {
        PerceptionParams params;
        params.filter_params.voxel_leaf_size = 0.3;
        params.ground_seg_params.max_iterations = 50;
        auto r = benchmarkPipeline("Embedded Optimized", cloud, params, iterations);
        results.push_back(r);
    }

    // 测试3: 高精度参数
    {
        PerceptionParams params;
        params.filter_params.voxel_leaf_size = 0.05;
        params.filter_params.enable_voxel_grid = true;
        params.ground_seg_params.max_iterations = 200;
        auto r = benchmarkPipeline("High Precision", cloud, params, iterations);
        results.push_back(r);
    }

    // 测试4: 仅滤波
    {
        PerceptionParams params;
        params.ground_seg_params.max_iterations = 0;  // 不执行
        // 设置聚类参数使其不生效
        params.cluster_params.min_cluster_size = 100000;  // 不聚类
        auto r = benchmarkPipeline("Filter Only", cloud, params, iterations);
        results.push_back(r);
    }

    printResults(results);

    return 0;
}
