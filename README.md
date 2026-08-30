# 三维智能感知机载激光雷达系统 · NVIDIA Jetson Orin NX

一套部署于 NVIDIA Jetson Orin NX 的机载三维 LiDAR 实时感知系统。项目覆盖高速 UDP 点云接收、ROS 驱动、C++ 多线程处理管线、地面分割、障碍物聚类、着陆区评估和可视化；重点是嵌入式 Linux 平台上的实时性、资源控制与可靠部署，而非离线算法演示工程。

> **平台**：NVIDIA Jetson Orin NX（ARM64 / Embedded Linux）
> **技术栈**：C++17、ROS/catkin、PCL、UDP/epoll、OpenMP、RViz、Python/PyQt 辅助工具
> **本人职责**：负责 LiDAR 数据链路、点云感知管线、嵌入式性能优化、ROS 集成、Jetson 交叉编译与现场部署。
> **项目资料**：[STAR 完整梳理](项目三_LS3激光雷达感知系统_STAR完整梳理.md)

## 项目背景

面向无人机避障与自主着陆场景，系统需要在 Orin NX 上持续接收雷神 LS1550 LiDAR 的高速 UDP 数据，并在有限 CPU、内存和功耗预算内输出可用于避障和着陆评估的点云结果。工程难点不只是完成点云算法，而是保证网络接收、算法处理、ROS 发布和可视化之间的速率匹配及异常隔离。

## 实现概览

```text
LS1550 LiDAR
     │ UDP / epoll
     v
数据接收与点云解析 ──> ROS PointCloud2 ──> 实时感知 Pipeline ──> RViz / Qt
Memory Pool                                  ROI / Voxel / SOR
丢包统计                                     地面分割 / 聚类 / 分类 / 着陆区
     │                                             │
     └────────── systemd 进程管理 / Jetson 资源监控 ─┘
```

| 层级 | 模块 | 职责 |
| --- | --- | --- |
| 输入层 | `input.cc` | UDP 接收、PCAP 输入与原始包读取 |
| 驱动层 | `lslidar_ls_driver` | 数据包解析、点云组帧、ROS 消息发布 |
| 核心层 | `core/` | 时间同步、数据采集、线程安全 Pipeline |
| 算法层 | `algorithm/` | 滤波、地面分割、聚类、障碍物/着陆区评估 |
| 平台层 | `platform/` | 性能统计、CPU/内存等运行状态监控 |

## 技术要点

- 使用非阻塞 UDP 与 `epoll` 处理高速传感器输入，并通过内存池减少频繁动态分配。
- 按“采集 -> 预处理 -> 感知 -> 发布/可视化”组织多线程 Pipeline，以有界队列承接上下游速率差异。
- 使用 PCL 完成 ROI 裁剪、体素滤波、离群点处理、RANSAC 地面分割、欧式聚类和几何规则分类。
- 提供着陆区平整度和障碍物安全距离评估，输出 ROS 点云和 Marker 供 RViz 检查。
- 面向 Orin NX 统一 C++17 与 ARMv8.2-A/Cortex-A78 交叉编译参数；移除了 AArch64 不支持的 ARM32 FPU 参数。
- 使用 launch、YAML 配置和 systemd 脚本解耦设备参数、启动流程与业务代码。

## 工程结构

```text
src/lslidar_ls_driver/
├── include/lslidar_ls_driver/
│   ├── algorithm/           点云感知算法
│   ├── core/                Pipeline、时间同步与数据采集
│   ├── driver/              增强驱动接口
│   └── platform/            资源与性能监控
├── src/                     ROS 节点、驱动与输入实现
├── config/                  LiDAR 与 Pipeline 配置
├── launch/                  ROS 启动文件
├── cmake/                   Jetson Orin NX 工具链
├── tests/                   单元/rostest 测试
├── benchmark/               性能基准工具
└── package.xml              ROS 包描述
```

## 构建与验证

在 Jetson Orin NX 的 Ubuntu/ROS 环境安装 ROS、PCL、libpcap 后：

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch lslidar_ls_driver lslidar_ls1550_enhanced.launch
```

交叉编译时使用 `src/lslidar_ls_driver/cmake/jetson_toolchain.cmake` 并提供目标 sysroot。`build/`、`devel/`、IDE 缓存和运行数据均不纳入版本控制，目标机部署前应依次验证 UDP 接收、点云发布、Pipeline 输出和资源占用。
