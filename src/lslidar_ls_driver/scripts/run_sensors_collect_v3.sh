#!/bin/bash
# =============================================================================
# run_sensors_collect_v3.sh — LS3 LiDAR 多传感器采集系统启动脚本
#
# 功能: 自动检查ROS环境、硬件连接、依赖包，然后启动多传感器采集系统
#
# 使用方法:
#   ./run_sensors_collect_v3.sh                        # 默认启动
#   ./run_sensors_collect_v3.sh -c                     # 仅检查环境
#   ./run_sensors_collect_v3.sh -d 192.168.1.100       # 自定义雷达IP
#   ./run_sensors_collect_v3.sh --no-gui               # 无GUI模式(后台记录)
#   ./run_sensors_collect_v3.sh -h                     # 帮助信息
#
# 最佳实践:
#   1. 推荐通过 systemd 服务管理: sudo systemctl start ls3lidar
#   2. 日志可通过 journalctl -u ls3lidar 查看
#   3. 后台运行: nohup ./run_sensors_collect_v3.sh --no-gui &
#
# 作者: 周聪
# 日期: 2025
# =============================================================================

# ============================ 配置区 ========================================
# 以下路径请根据实际环境修改
# 也可以通过环境变量 LS3_WS_PATH 覆盖
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS_PATH="${LS3_WS_PATH:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"
DEFAULT_SAVE_PATH="/home/nuaa/Ayra/data_lidargpsimu/"
DEFAULT_DEVICE_IP="192.168.1.200"

# ============================ 颜色定义 ======================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# ============================ 日志函数 ======================================
print_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[OK]${NC} $1"; }
print_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error()   { echo -e "${RED}[ERROR]${NC} $1"; }
print_step()    { echo -e "${CYAN}[STEP]${NC} $1"; }
print_banner()  {
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════╗"
    echo "║       LS3 LiDAR 多传感器采集系统 v3.0           ║"
    echo "║       中航工业洛阳XX研究所 · 原理样机            ║"
    echo "╚══════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ============================ ROS环境检查 ===================================
check_ros_environment() {
    print_step "1/7 检查ROS环境..."

    if [ -z "$ROS_DISTRO" ]; then
        # 尝试自动查找ROS
        for ros_setup in /opt/ros/*/setup.bash; do
            if [ -f "$ros_setup" ]; then
                print_info "发现ROS环境: $ros_setup"
                source "$ros_setup"
                break
            fi
        done

        if [ -z "$ROS_DISTRO" ]; then
            print_error "ROS环境未设置!"
            print_info "请先source ROS环境，例如:"
            print_info "  source /opt/ros/noetic/setup.bash"
            return 1
        fi
    fi

    print_success "ROS $ROS_DISTRO 环境就绪"

    # 检查ROS Master是否在运行
    if rostopic list > /dev/null 2>&1; then
        print_success "ROS Master 正在运行"
    else
        print_warn "ROS Master 未运行，尝试启动 roscore..."
        roscore &
        ROSCORE_PID=$!
        sleep 2
        if kill -0 $ROSCORE_PID 2>/dev/null; then
            print_success "roscore 已启动 (PID: $ROSCORE_PID)"
        else
            print_error "roscore 启动失败，请手动启动: roscore &"
            return 1
        fi
    fi
    return 0
}

# ============================ 工作空间检查 ===================================
check_workspace() {
    print_step "2/7 检查工作空间..."

    local ws_path="${1:-$DEFAULT_WS_PATH}"

    if [ ! -d "$ws_path" ]; then
        print_error "工作空间不存在: $ws_path"
        print_info "请设置正确的路径: export LS3_WS_PATH=/path/to/workspace"
        return 1
    fi

    print_info "工作空间: $ws_path"

    # 检查是否已编译
    if [ -f "$ws_path/devel/setup.bash" ]; then
        source "$ws_path/devel/setup.bash"
        print_success "工作空间已编译"
    else
        print_warn "工作空间未编译，编译中..."
        cd "$ws_path"
        if catkin_make -DCMAKE_BUILD_TYPE=Release; then
            source "$ws_path/devel/setup.bash"
            print_success "编译完成"
        else
            print_error "编译失败，请检查编译错误"
            return 1
        fi
    fi

    # 验证驱动包可用
    if rospack find lslidar_ls_driver > /dev/null 2>&1; then
        print_success "ROS包 lslidar_ls_driver 可用"
    else
        print_error "ROS包 lslidar_ls_driver 未找到"
        return 1
    fi

    return 0
}

# ============================ 硬件检查 =======================================
check_hardware() {
    print_step "3/7 检查硬件设备..."

    local errors=0

    # 相机
    if [ -e "/dev/video0" ]; then
        print_success "相机: /dev/video0"
    else
        print_warn "相机未找到 /dev/video0 (不影响运行)"
    fi

    # GPS
    if [ -e "/dev/wheeltec_gps" ]; then
        print_success "GPS: /dev/wheeltec_gps"
    elif [ -e "/dev/ttyUSB0" ]; then
        print_warn "GPS: 未找到 /dev/wheeltec_gps, 但发现 /dev/ttyUSB0"
    else
        print_warn "GPS设备未找到 (不影响运行)"
    fi

    # IMU
    if [ -e "/dev/wheeltec_IMU" ]; then
        print_success "IMU: /dev/wheeltec_IMU"
    else
        print_warn "IMU设备未找到 (不影响运行)"
    fi

    return 0
}

# ============================ 网络检查 =======================================
check_network() {
    print_step "4/7 检查网络连接..."

    local lidar_ip="${1:-$DEFAULT_DEVICE_IP}"

    # 检查LiDAR网络接口
    if ip link show eth0 > /dev/null 2>&1; then
        local eth_ip=$(ip -4 addr show eth0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
        print_info "本机IP: ${eth_ip:-未配置}"
    fi

    # ping雷达
    if ping -c 1 -W 1 "$lidar_ip" > /dev/null 2>&1; then
        print_success "LiDAR 网络连接正常 ($lidar_ip)"
    else
        print_warn "无法连接到雷达 $lidar_ip"
        print_info "请检查: 网线连接 | IP配置 | 雷达电源"
        print_info "可用 'ip addr show eth0' 查看本机IP"
    fi

    return 0
}

# ============================ 数据目录 =======================================
check_data_directory() {
    print_step "5/7 检查数据保存目录..."

    local save_path="${1:-$DEFAULT_SAVE_PATH}"

    if [ ! -d "$save_path" ]; then
        mkdir -p "$save_path"
        if [ $? -eq 0 ]; then
            print_success "数据目录已创建: $save_path"
        else
            print_error "无法创建数据目录: $save_path"
            return 1
        fi
    else
        # 检查磁盘空间
        local avail=$(df -h "$save_path" | awk 'NR==2 {print $4}')
        print_success "数据目录: $save_path (可用空间: $avail)"
    fi

    return 0
}

# ============================ 依赖检查 =======================================
check_dependencies() {
    print_step "6/7 检查Python依赖..."

    local missing=0

    # 关键依赖
    local deps=("rospy" "cv2" "numpy" "PyQt5" "pyqtgraph" "open3d" "serial" "yaml")
    for dep in "${deps[@]}"; do
        if python3 -c "import $dep" 2>/dev/null; then
            :
        else
            print_warn "缺少Python包: $dep"
            missing=$((missing + 1))
        fi
    done

    if [ $missing -eq 0 ]; then
        print_success "Python依赖检查通过"
    else
        print_error "缺少 $missing 个Python包"
        print_info "安装命令: pip3 install pyqtgraph open3d pyyaml"
        return 1
    fi

    return 0
}

# ============================ 打印配置摘要 ===================================
print_summary() {
    print_step "7/7 启动配置摘要"

    local ws_path="${1:-$DEFAULT_WS_PATH}"
    local save_path="${2:-$DEFAULT_SAVE_PATH}"
    local device_ip="${3:-$DEFAULT_DEVICE_IP}"
    local gps_port="${4:-/dev/wheeltec_gps}"
    local imu_port="${5:-/dev/wheeltec_IMU}"
    local camera_index="${6:-0}"
    local enable_gui="${7:-true}"

    echo ""
    echo "╔══════════════════════════════════════════════════╗"
    echo "║                配置摘要                          ║"
    echo "╠══════════════════════════════════════════════════╣"
    printf "║  %-20s %-20s ║\n" "工作空间:" "$ws_path"
    printf "║  %-20s %-20s ║\n" "雷达IP:" "$device_ip"
    printf "║  %-20s %-20s ║\n" "数据保存:" "$save_path"
    printf "║  %-20s %-20s ║\n" "GUI模式:" "$enable_gui"
    printf "║  %-20s %-20s ║\n" "GPS串口:" "$gps_port"
    printf "║  %-20s %-20s ║\n" "IMU串口:" "$imu_port"
    printf "║  %-20s %-20s ║\n" "相机索引:" "$camera_index"
    printf "║  %-20s %-20s ║\n" "ROS版本:" "${ROS_DISTRO:-未知}"
    printf "║  %-20s %-20s ║\n" "系统时间:" "$(date)"
    echo "╚══════════════════════════════════════════════════╝"
}

# ============================ 启动系统 =======================================
start_system() {
    local ws_path="$1"
    local device_ip="$2"
    local save_path="$3"
    local gps_port="$4"
    local imu_port="$5"
    local camera_index="$6"
    local no_gui="$7"

    # 构建launch命令
    local launch_cmd="roslaunch lslidar_ls_driver lslidar_ls1550_pipeline.launch"
    launch_cmd="$launch_cmd device_ip:=$device_ip"
    launch_cmd="$launch_cmd save_path:=$save_path"
    launch_cmd="$launch_cmd gps_port:=$gps_port"
    launch_cmd="$launch_cmd imu_port:=$imu_port"
    launch_cmd="$launch_cmd camera_index:=$camera_index"

    if [ "$no_gui" = true ]; then
        launch_cmd="$launch_cmd gui:=false"
    fi

    print_info "执行: $launch_cmd"
    echo "──────────────────────────────────────────────"
    print_info "系统启动中... 按 Ctrl+C 停止"
    echo ""

    # 执行launch
    eval "$launch_cmd"
}

# ============================ 主函数 ========================================
main() {
    # 默认值
    local device_ip="$DEFAULT_DEVICE_IP"
    local save_path="$DEFAULT_SAVE_PATH"
    local ws_path="$DEFAULT_WS_PATH"
    local gps_port="/dev/wheeltec_gps"
    local imu_port="/dev/wheeltec_IMU"
    local camera_index="0"
    local check_only=false
    local no_gui=false

    # 解析参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -c|--check-only)
                check_only=true
                shift
                ;;
            --no-gui)
                no_gui=true
                shift
                ;;
            -d|--device-ip)
                device_ip="$2"; shift 2
                ;;
            -p|--save-path)
                save_path="$2"; shift 2
                ;;
            -w|--workspace)
                ws_path="$2"; shift 2
                ;;
            -g|--gps-port)
                gps_port="$2"; shift 2
                ;;
            -i|--imu-port)
                imu_port="$2"; shift 2
                ;;
            --camera-index)
                camera_index="$2"; shift 2
                ;;
            *)
                print_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # 打印Banner
    print_banner

    # 执行环境检查（允许部分失败）
    local check_failed=0

    check_ros_environment || ((check_failed++))
    check_workspace "$ws_path" || ((check_failed++))
    check_hardware || true  # 硬件检查允许失败
    check_network "$device_ip" || true  # 网络检查允许失败
    check_data_directory "$save_path" || ((check_failed++))
    check_dependencies || ((check_failed++))

    # 仅检查模式
    if [ "$check_only" = true ]; then
        echo ""
        if [ $check_failed -eq 0 ]; then
            print_success "环境检查通过！一切就绪。"
        else
            print_warn "环境检查完成，$check_failed 项存在问题（非致命）"
        fi
        exit $check_failed
    fi

    if [ $check_failed -gt 2 ]; then
        print_error "环境检查存在 $check_failed 项严重问题，请修复后重试"
        exit 1
    fi

    # 打印配置摘要
    local gui_text=$([ "$no_gui" = true ] && echo "禁用" || echo "启用")
    print_summary "$ws_path" "$save_path" "$device_ip" "$gps_port" "$imu_port" "$camera_index" "$gui_text"

    # 启动系统
    start_system "$ws_path" "$device_ip" "$save_path" "$gps_port" "$imu_port" "$camera_index" "$no_gui"

    print_success "系统已退出"
}

# ============================ 帮助信息 =======================================
show_help() {
    echo "LS3 LiDAR 多传感器采集系统 - 启动脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助"
    echo "  -c, --check-only        仅检查环境，不启动"
    echo "  --no-gui                不启动GUI (后台记录模式)"
    echo "  -d, --device-ip IP      雷达IP (默认: $DEFAULT_DEVICE_IP)"
    echo "  -p, --save-path PATH    数据保存路径"
    echo "  -w, --workspace PATH    工作空间路径"
    echo "  -g, --gps-port PORT     GPS串口"
    echo "  -i, --imu-port PORT     IMU串口"
    echo "  --camera-index INDEX    相机索引"
    echo ""
    echo "环境变量:"
    echo "  LS3_WS_PATH             工作空间路径 (覆盖默认)"
    echo "  LS3_LIDAR_IP            雷达IP地址"
    echo "  LS3_SAVE_PATH           数据保存路径"
    echo "  LS3_CONFIG_PATH         YAML配置文件路径"
    echo ""
    echo "示例:"
    echo "  $0                                          # 完整启动"
    echo "  $0 -c                                       # 仅检查"
    echo "  $0 -d 192.168.1.100 --no-gui                 # 自定义IP + 后台"
    echo "  LS3_WS_PATH=/opt/ros_ws $0                   # 自定义工作空间"
}

# ============================ 信号处理 =======================================
cleanup() {
    echo ""
    print_info "收到停止信号，正在退出..."
    # 杀死子进程 (roscore等)
    if [ -n "$ROSCORE_PID" ]; then
        kill "$ROSCORE_PID" 2>/dev/null
    fi
    exit 0
}

trap cleanup SIGINT SIGTERM

# ============================ 入口 =======================================
main "$@"
