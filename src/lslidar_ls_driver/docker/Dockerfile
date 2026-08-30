# =============================================================================
# Dockerfile - LS3 LiDAR Enhanced Driver
#
# 构建:
#   docker build -t ls3lidar:latest -f docker/Dockerfile .
#
# 运行:
#   docker run --rm -it --net=host --privileged ls3lidar:latest
#
# 作者: 周聪
# 日期: 2025
# =============================================================================

FROM ros:melodic-ros-base-bionic AS base

# 安装依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libpcap-dev \
    libpcl-dev \
    pcl-tools \
    libgoogle-glog-dev \
    libeigen3-dev \
    libboost-all-dev \
    ros-melodic-pcl-ros \
    ros-melodic-tf2-eigen \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-visualization-msgs \
    ros-melodic-nav-msgs \
    ros-melodic-rviz \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
WORKDIR /catkin_ws
RUN mkdir -p src/lslidar_ls_driver

# 复制源码
COPY . src/lslidar_ls_driver/

# 编译
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    cd /catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

# 环境变量
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
ENV ROS_MASTER_URI=http://localhost:11311

# 入口
CMD ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && roslaunch lslidar_ls_driver lslidar_ls1550_enhanced.launch"]
