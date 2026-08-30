#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
config_manager.py — 统一配置管理器

功能: 集中管理所有传感器的配置参数，支持从ROS参数、YAML文件、环境变量、
      命令行参数等多种来源加载配置，并提供类型安全和默认值保障。

使用方式:
    from config_manager import ConfigManager, load_config
    cfg = load_config()
    print(cfg.lidar_ip)
    print(cfg.save_path)

设计原则:
    - 单一数据源: 所有配置集中管理，消除散落在各文件中的硬编码
    - 分层覆盖: 命令行 > YAML文件 > ROS参数 > 默认值
    - 类型安全: 配置项带类型注解和校验

作者: 周聪
日期: 2025
"""

import os
import sys
import argparse
import yaml
from typing import Optional, Dict, Any
from dataclasses import dataclass, field, asdict
from pathlib import Path


# ============================================================================
# 数据类定义 — 配置的数据结构
# ============================================================================

@dataclass
class LidarConfig:
    """LiDAR传感器配置"""
    device_ip: str = "192.168.1.200"
    msop_port: int = 2368
    difop_port: int = 2369
    add_multicast: bool = False
    group_ip: str = "224.1.1.2"
    packet_rate: float = 22440.0
    use_time_service: bool = False
    time_service_mode: str = "gps"  # gps / ptp_l2 / ptp_udpv4 / ntp
    frame_id: str = "laser_link"
    min_range: float = 0.5
    max_range: float = 300.0
    scan_start_angle: int = -60
    scan_end_angle: int = 60
    pointcloud_topic: str = "lslidar_point_cloud"
    packet_loss_detection: bool = True


@dataclass
class CameraConfig:
    """相机配置"""
    index: int = 0
    width: int = 1920
    height: int = 1080
    fps: int = 20


@dataclass
class SerialPortConfig:
    """串口配置"""
    gps_port: str = "/dev/wheeltec_gps"
    gps_baud_rate: int = 9600
    imu_port: str = "/dev/wheeltec_IMU"
    imu_baud_rate: int = 460800


@dataclass
class PipelineConfig:
    """处理管线配置"""
    voxel_leaf_size: float = 0.15
    enable_voxel_grid: bool = True
    enable_height_filter: bool = True
    height_min: float = -2.0
    height_max: float = 10.0
    ground_seg_distance_threshold: float = 1.0
    ground_seg_ransac_n: int = 3
    ground_seg_num_iterations: int = 10
    cluster_eps: float = 2.0
    cluster_min_points: int = 15
    cluster_min_vis: int = 20


@dataclass
class LandingZoneConfig:
    """着陆区检测配置"""
    grid_resolution: float = 1.5
    zone_radius: float = 1.5
    max_cell_std_dev: float = 0.08
    max_cell_range: float = 0.15
    max_zone_planarity_std_dev: float = 0.10
    min_points_per_cell: int = 5


@dataclass
class SaveConfig:
    """数据保存配置"""
    base_path: str = "/home/nuaa/Ayra/data_lidargpsimu/"
    save_pointcloud: bool = True
    save_image: bool = True
    save_sensors: bool = True
    queue_size: int = 500


@dataclass
class AppConfig:
    """应用总配置"""
    lidar: LidarConfig = field(default_factory=LidarConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    serial: SerialPortConfig = field(default_factory=SerialPortConfig)
    pipeline: PipelineConfig = field(default_factory=PipelineConfig)
    landing_zone: LandingZoneConfig = field(default_factory=LandingZoneConfig)
    save: SaveConfig = field(default_factory=SaveConfig)

    # 通用
    enable_gui: bool = True
    enable_obstacle_detection: bool = True
    enable_landing_zone: bool = True
    log_level: str = "INFO"
    workspace_path: str = ""


# ============================================================================
# 配置加载器
# ============================================================================

_DEFAULT_CONFIG_PATHS = [
    # 按优先级从低到高
    os.path.join(os.path.dirname(__file__), "..", "config", "pipeline_config.yaml"),
    os.path.join(os.path.dirname(__file__), "config.yaml"),
    "/etc/ls3lidar/config.yaml",
]


def _merge_dict(base: dict, override: dict) -> dict:
    """递归合并两个字典"""
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _merge_dict(result[key], value)
        else:
            result[key] = value
    return result


def _load_yaml_config(yaml_path: Optional[str] = None) -> dict:
    """从YAML文件加载配置"""
    if yaml_path and os.path.exists(yaml_path):
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f) or {}

    # 搜索默认路径
    for path in _DEFAULT_CONFIG_PATHS:
        if os.path.exists(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}

    return {}


def _env_to_config() -> dict:
    """从环境变量加载配置（环境变量优先级最高）"""
    env_map = {
        "LS3_LIDAR_IP": ("lidar", "device_ip"),
        "LS3_MSOP_PORT": ("lidar", "msop_port"),
        "LS3_DIFOP_PORT": ("lidar", "difop_port"),
        "LS3_SAVE_PATH": ("save", "base_path"),
        "LS3_GPS_PORT": ("serial", "gps_port"),
        "LS3_IMU_PORT": ("serial", "imu_port"),
        "LS3_LOG_LEVEL": ("log_level"),
    }

    config = {}
    for env_key, config_path in env_map.items():
        value = os.environ.get(env_key)
        if value is None:
            continue

        # 类型转换
        if isinstance(config_path, tuple):
            section, key = config_path
            if section not in config:
                config[section] = {}
            try:
                config[section][key] = int(value)
            except ValueError:
                config[section][key] = value

    return config


def _dataclass_to_dict(obj):
    """递归将dataclass转换为字典"""
    result = {}
    for key, value in asdict(obj).items():
        if isinstance(value, (LidarConfig, CameraConfig, SerialPortConfig,
                              PipelineConfig, LandingZoneConfig, SaveConfig)):
            result[key] = _dataclass_to_dict(value)
        else:
            result[key] = value
    return result


def _dict_to_dataclass(data: dict, config: AppConfig) -> AppConfig:
    """将字典数据合并到dataclass配置中"""
    section_map = {
        "lidar": ("lidar", LidarConfig),
        "camera": ("camera", CameraConfig),
        "serial": ("serial", SerialPortConfig),
        "pipeline": ("pipeline", PipelineConfig),
        "landing_zone": ("landing_zone", LandingZoneConfig),
        "save": ("save", SaveConfig),
    }

    for section_name, (attr_name, cls) in section_map.items():
        section_data = data.get(section_name, {})
        if not section_data:
            continue
        current = getattr(config, attr_name)
        for key, value in section_data.items():
            if hasattr(current, key):
                setattr(current, key, value)

    return config


def load_config(yaml_path: Optional[str] = None,
                cli_args: Optional[Dict[str, Any]] = None) -> AppConfig:
    """
    加载配置（优先级: CLI参数 > 环境变量 > YAML文件 > 默认值）

    Args:
        yaml_path: YAML配置文件路径（可选）
        cli_args: 命令行参数字典（可选）

    Returns:
        AppConfig 配置对象
    """
    config = AppConfig()

    # 1. 加载YAML配置
    yaml_data = _load_yaml_config(yaml_path)
    config = _dict_to_dataclass(yaml_data, config)

    # 2. 环境变量覆盖
    env_data = _env_to_config()
    config = _dict_to_dataclass(env_data, config)

    # 3. CLI参数覆盖
    if cli_args:
        config = _dict_to_dataclass(cli_args, config)

    # 4. 尝试从ROS参数加载
    try:
        import rospy
        if rospy.has_param('~save_path'):
            config.save.base_path = rospy.get_param('~save_path')
        if rospy.has_param('~camera_index'):
            config.camera.index = rospy.get_param('~camera_index')
        if rospy.has_param('~gps_port'):
            config.serial.gps_port = rospy.get_param('~gps_port')
        if rospy.has_param('~imu_port'):
            config.serial.imu_port = rospy.get_param('~imu_port')
    except (ImportError, rospy.ROSException):
        pass

    return config


def create_argparser() -> argparse.ArgumentParser:
    """创建命令行参数解析器（与roscpp兼容的接口）"""
    parser = argparse.ArgumentParser(
        description="LS3 LiDAR 多传感器采集系统",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                          # 默认参数运行
  %(prog)s --device-ip 192.168.1.100               # 自定义雷达IP
  %(prog)s --no-gui --save-path /mnt/data/         # 无GUI后台记录
  %(prog)s --check-only                             # 仅检查环境
        """
    )

    # LiDAR参数
    lidar_group = parser.add_argument_group("LiDAR参数")
    lidar_group.add_argument("--device-ip", default=None, help="雷达IP地址")
    lidar_group.add_argument("--msop-port", type=int, default=None, help="数据端口")
    lidar_group.add_argument("--difop-port", type=int, default=None, help="设备端口")

    # 串口参数
    serial_group = parser.add_argument_group("串口参数")
    serial_group.add_argument("--gps-port", default=None, help="GPS串口设备")
    serial_group.add_argument("--imu-port", default=None, help="IMU串口设备")

    # 相机参数
    camera_group = parser.add_argument_group("相机参数")
    camera_group.add_argument("--camera-index", type=int, default=None, help="相机设备索引")

    # 保存参数
    save_group = parser.add_argument_group("数据保存")
    save_group.add_argument("-p", "--save-path", default=None, help="数据保存路径")

    # 模式选项
    mode_group = parser.add_argument_group("运行模式")
    mode_group.add_argument("--no-gui", action="store_true", help="不启动GUI")
    mode_group.add_argument("--check-only", action="store_true", help="仅检查环境")
    mode_group.add_argument("--config", default=None, help="YAML配置文件路径")
    mode_group.add_argument("-v", "--verbose", action="store_true", help="详细日志")

    return parser


def cli_to_dict(args: argparse.Namespace) -> dict:
    """将argparse结果转换为配置字典（跳过None值）"""
    result = {}
    mapping = {
        "device_ip": ("lidar", "device_ip"),
        "msop_port": ("lidar", "msop_port"),
        "difop_port": ("lidar", "difop_port"),
        "gps_port": ("serial", "gps_port"),
        "imu_port": ("serial", "imu_port"),
        "camera_index": ("camera", "index"),
        "save_path": ("save", "base_path"),
    }

    for cli_name, (section, key) in mapping.items():
        value = getattr(args, cli_name.replace("-", "_"), None)
        if value is not None:
            if section not in result:
                result[section] = {}
            result[section][key] = value

    if args.no_gui:
        result["enable_gui"] = False
    if args.check_only:
        result["check_only"] = True
    if args.verbose:
        result["log_level"] = "DEBUG"

    return result


# ============================================================================
# 便捷 API
# ============================================================================

def get_config_path() -> str:
    """获取配置文件的推荐路径"""
    # 优先级: 环境变量 > 默认工作空间路径 > 包内路径
    env_path = os.environ.get("LS3_CONFIG_PATH")
    if env_path:
        return env_path

    ws_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "config", "pipeline_config.yaml"
    )
    if os.path.exists(ws_path):
        return ws_path

    return os.path.join(os.path.dirname(__file__), "..", "config", "pipeline_config.yaml")


# 测试入口
if __name__ == "__main__":
    parser = create_argparser()
    args = parser.parse_args()
    cli_dict = cli_to_dict(args)
    config = load_config(cli_args=cli_dict)

    print("=" * 60)
    print("LS3 LiDAR 系统配置")
    print("=" * 60)
    print(f"LiDAR IP: {config.lidar.device_ip}")
    print(f"保存路径: {config.save.base_path}")
    print(f"GUI使能: {config.enable_gui}")
    print(f"障碍物检测: {config.enable_obstacle_detection}")
    print(f"着陆区检测: {config.enable_landing_zone}")
    print(f"日志级别: {config.log_level}")
    print("=" * 60)
