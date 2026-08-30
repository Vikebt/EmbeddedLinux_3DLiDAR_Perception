#!/usr/bin/env python3
"""
LS3 LiDAR Python 应用包

提供多传感器数据采集、点云处理、障碍物检测的Python应用层。

安装:
    pip install -e .     # 开发模式安装
    pip install .        # 生产模式安装

使用:
    # 作为模块导入
    from ls3lidar_app.config_manager import load_config
    from ls3lidar_app.sensors_collect_v3 import main

    # 命令行运行
    python -m ls3lidar_app.sensors_collect_v3 [options]
"""

from setuptools import setup, find_packages

setup(
    name="ls3lidar_app",
    version="3.0.0",
    description="LS3 LiDAR 多传感器数据采集与点云处理应用",
    author="Zhou Cong",
    author_email="your-email@example.com",
    packages=find_packages(),
    python_requires=">=3.6",
    install_requires=[
        "numpy>=1.17",
        "open3d>=0.12",
        "PyQt5>=5.12",
        "pyqtgraph>=0.11",
        "pyyaml>=5.1",
        "scipy>=1.3",
        "scikit-learn>=0.20",
        "opencv-python>=4.1",
        "pyserial>=3.4",
        "matplotlib>=3.1",
        "pandas>=1.0",
    ],
    extras_require={
        "dev": ["pytest", "pytest-cov", "flake8"],
        "ros": ["rospy", "sensor_msgs", "std_msgs"],
    },
    entry_points={
        "console_scripts": [
            "ls3lidar-collect=sensors_collect_v3:main",
        ],
    },
)
