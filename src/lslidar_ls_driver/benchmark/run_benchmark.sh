#!/bin/bash
# =============================================================================
# 性能基准测试运行脚本
#
# 使用方法:
#   chmod +x run_benchmark.sh
#   ./run_benchmark.sh                    # 运行全部测试
#   ./run_benchmark.sh --quick            # 快速测试 (100次)
#   ./run_benchmark.sh --full             # 完整测试 (1000次)
#   ./run_benchmark.sh --input cloud.pcd  # 使用自定义点云
#
# 作者: 周聪
# 日期: 2025
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build_benchmark"

ITERATIONS=500
INPUT_CLOUD=""

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --quick) ITERATIONS=100; shift ;;
        --full)  ITERATIONS=1000; shift ;;
        --input) INPUT_CLOUD="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo "============================================"
echo "  LS3 LiDAR Performance Benchmark"
echo "  Iterations: $ITERATIONS"
echo "============================================"

# 编译benchmark
echo "[1/3] Building benchmark..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake "$PROJECT_DIR" -DCMAKE_BUILD_TYPE=Release 2>/dev/null
make benchmark_pipeline -j$(nproc) 2>/dev/null

if [ $? -ne 0 ]; then
    echo "Building benchmark directly..."
    g++ -std=c++14 -O3 -march=native "$SCRIPT_DIR/benchmark_pipeline.cpp" \
        -o "$BUILD_DIR/benchmark_pipeline" \
        -I"$PROJECT_DIR/include" \
        $(pkg-config --cflags --libs pcl_common pcl_filters pcl_segmentation pcl_search pcl_io 2>/dev/null) \
        -lboost_system
fi

if [ ! -f "$BUILD_DIR/benchmark_pipeline" ]; then
    echo "ERROR: Failed to build benchmark."
    exit 1
fi

# 运行测试
echo "[2/3] Running benchmark..."
BENCH_ARGS="--iterations $ITERATIONS"
[ -n "$INPUT_CLOUD" ] && BENCH_ARGS="$BENCH_ARGS --input $INPUT_CLOUD"

$BUILD_DIR/benchmark_pipeline $BENCH_ARGS 2>&1 | tee "$BUILD_DIR/benchmark_results.txt"

# 生成CSV报告
echo "[3/3] Generating CSV report..."
echo "Test,Avg(ms),Min(ms),Max(ms),P99(ms),Hz,Points" > "$BUILD_DIR/benchmark_results.csv"
grep -E "^\s+\w+" "$BUILD_DIR/benchmark_results.txt" | awk '{
    printf "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n",
        $1, $2, $3, $4, $5, $6, $7
}' >> "$BUILD_DIR/benchmark_results.csv"

echo ""
echo "Results saved to:"
echo "  - $BUILD_DIR/benchmark_results.txt"
echo "  - $BUILD_DIR/benchmark_results.csv"
echo "============================================"
