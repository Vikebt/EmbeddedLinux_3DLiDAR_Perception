# 多传感器采集系统启动文件总结

本文档总结了为 `sensors_collect_v3.py` 脚本创建的所有启动和配置文件。

## 创建的文件列表

### 1. ROS Launch文件
- **文件路径**: `launch/sensors_collect_v3.launch`
- **功能**: ROS launch文件，用于启动雷达驱动和多传感器采集系统
- **主要参数**:
  - 雷达IP地址和端口配置
  - 相机参数（索引、分辨率、FPS）
  - GPS/IMU串口参数
  - 数据保存路径

### 2. 启动脚本

#### 2.1 完整启动脚本
- **文件路径**: `scripts/run_sensors_collect_v3.sh`
- **功能**: 功能完整的启动脚本，包含环境检查、硬件检测、参数配置等
- **特性**:
  - 全面的环境检查
  - 硬件设备检测
  - 网络连接测试
  - 命令行参数支持
  - 彩色输出和错误处理

#### 2.2 简化启动脚本
- **文件路径**: `scripts/start_sensors_collect_v3.sh`
- **功能**: 简化版启动脚本，用于快速启动系统
- **特性**:
  - 基本的ROS环境检查
  - 工作空间加载
  - 直接启动系统

#### 2.3 测试脚本
- **文件路径**: `scripts/test_sensors_collect_v3.sh`
- **功能**: 系统配置和依赖测试脚本
- **特性**:
  - 全面的系统测试
  - 依赖包检查
  - 硬件设备检测
  - 测试结果汇总

### 3. 配置文件
- **文件路径**: `config/sensors_collect_v3_config.yaml`
- **功能**: YAML格式的配置文件，包含所有系统参数
- **内容**:
  - 雷达参数配置
  - 相机参数配置
  - GPS/IMU参数配置
  - 算法参数配置
  - 显示参数配置

### 4. 文档文件
- **文件路径**: `README_sensors_collect_v3.md`
- **功能**: 详细的使用说明文档
- **内容**:
  - 系统特性介绍
  - 安装和配置说明
  - 使用方法和参数说明
  - 故障排除指南

## 使用方法

### 快速启动
```bash
# 使用简化脚本
./scripts/start_sensors_collect_v3.sh

# 使用完整脚本
./scripts/run_sensors_collect_v3.sh

# 使用ROS launch
roslaunch lslidar_ls_driver sensors_collect_v3.launch
```

### 系统测试
```bash
# 运行系统测试
./scripts/test_sensors_collect_v3.sh

# 仅检查环境（不启动系统）
./scripts/run_sensors_collect_v3.sh -c
```

### 自定义参数
```bash
# 使用自定义参数启动
./scripts/run_sensors_collect_v3.sh \
    -d 192.168.1.100 \
    -p /home/user/data/ \
    --camera-index 1
```

## 文件权限

所有脚本文件都已设置为可执行权限：
```bash
chmod +x scripts/*.sh
```

## 依赖要求

### 系统依赖
- Ubuntu 18.04/20.04
- ROS Noetic/Melodic
- Python 3.6+

### Python包依赖
```bash
sudo apt-get install python3-opencv python3-numpy python3-pyqt5 python3-serial
pip3 install pyqtgraph open3d
```

### 硬件要求
- 激光雷达 (LS1550)
- USB相机
- GPS模块 (wheeltec)
- IMU模块 (wheeltec)

## 注意事项

1. **权限设置**: 确保脚本文件具有执行权限
2. **环境变量**: 确保ROS环境已正确设置
3. **硬件连接**: 确保所有硬件设备正确连接
4. **网络配置**: 确保雷达网络连接正常
5. **数据目录**: 确保数据保存目录具有写权限

## 故障排除

如果遇到问题，请按以下顺序检查：

1. 运行测试脚本检查系统状态
2. 检查ROS环境和工作空间编译
3. 验证硬件设备连接
4. 检查网络连接和IP配置
5. 查看错误日志和输出信息

## 联系支持

如有问题或需要技术支持，请参考README文档或联系开发团队。
