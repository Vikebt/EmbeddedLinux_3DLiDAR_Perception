cmake_minimum_required(VERSION 2.8.3)
project(lslidar_ls_driver)

# ===========================================================================
# 编译配置
# ===========================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message("cmake_build_type: " ${CMAKE_BUILD_TYPE})

# 可选: 启用ASAN (调试用)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")

set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    pcl_conversions
    nodelet
    dynamic_reconfigure
    message_generation
    message_runtime
    std_msgs
    sensor_msgs
    visualization_msgs
    nav_msgs
    tf2_ros
)

# ===========================================================================
# 消息和服务文件
# ===========================================================================
add_message_files(FILES LslidarLsPacket.msg)

add_service_files(FILES
    angle_distortion_correction.srv
    frame_rate.srv
    data_ip.srv
    destination_ip.srv
    data_port.srv
    dev_port.srv
    invalid_data.srv
    standby_mode.srv
    time_service.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp nodelet pcl_ros pcl_conversions std_msgs sensor_msgs
    visualization_msgs nav_msgs tf2_ros message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# ===========================================================================
# Module 1: 底层输入层 - UDP数据接收
# ===========================================================================
add_library(lslidar_ls_input
    src/input.cc
)
target_link_libraries(lslidar_ls_input
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
)

# ===========================================================================
# Module 2: 基础驱动层 - 点云解析与原始转换
# ===========================================================================
add_library(lslidar_ls_driver
    src/lslidar_ls_driver.cc
)
target_link_libraries(lslidar_ls_driver
    lslidar_ls_input
    ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ls_driver
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# ===========================================================================
# Module 3: 基础驱动节点
# ===========================================================================
add_executable(lslidar_ls_driver_node
    src/lslidar_ls_driver_node.cc
)
target_link_libraries(lslidar_ls_driver_node
    lslidar_ls_driver
    ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ls_driver_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# ===========================================================================
# Module 4: 增强版驱动节点 (Pipeline + 算法 + 监控)
# ===========================================================================
add_executable(lslidar_ls_driver_enhanced_node
    src/lslidar_ls_driver_enhanced_node.cc
)
target_link_libraries(lslidar_ls_driver_enhanced_node
    lslidar_ls_driver
    ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ls_driver_enhanced_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)


# ===========================================================================
# Module 5: Pipeline 处理节点 - 从Python迁移的实时感知管线
# 订阅 /lslidar_point_cloud → 滤波/地面分割/聚类/着陆区检测 → 发布结果
# ===========================================================================
add_executable(lslidar_ls_pipeline_node
    src/lslidar_ls_pipeline_node.cc
)
target_link_libraries(lslidar_ls_pipeline_node
    ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ls_pipeline_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# ===========================================================================
# 依赖链接（确保input库正确依赖消息生成）
# ===========================================================================
if (catkin_EXPORTED_TARGETS)
    add_dependencies(lslidar_ls_input ${catkin_EXPORTED_TARGETS})
endif()

# ===========================================================================
# 单元测试 (需要catkin中启用测试)
# ===========================================================================
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    catkin_add_gtest(test_pipeline tests/test_pipeline.cpp)
    catkin_add_gtest(test_filter tests/test_filter.cpp)
    catkin_add_gtest(test_ground_seg tests/test_ground_seg.cpp)
    catkin_add_gtest(test_clustering tests/test_clustering.cpp)
    foreach(test_target test_pipeline test_filter test_ground_seg test_clustering)
        if(TARGET ${test_target})
            target_link_libraries(${test_target} ${catkin_LIBRARIES})
        endif()
    endforeach()

    add_executable(test_bounded_queue tests/host/test_bounded_queue.cpp)
    target_link_libraries(test_bounded_queue pthread)
    add_test(NAME bounded_queue COMMAND test_bounded_queue)
    set_tests_properties(bounded_queue PROPERTIES TIMEOUT 5)
endif()

# ===========================================================================
# 安装
# ===========================================================================
install(TARGETS
    lslidar_ls_input
    lslidar_ls_driver
    lslidar_ls_driver_node
    lslidar_ls_driver_enhanced_node
    lslidar_ls_pipeline_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
