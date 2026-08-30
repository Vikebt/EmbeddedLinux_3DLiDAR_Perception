# =============================================================================
# NVIDIA Jetson 交叉编译工具链
#
# 使用方法:
#   cmake -DCMAKE_TOOLCHAIN_FILE=cmake/jetson_toolchain.cmake \
#         -DJETSON_ROOT=/path/to/aarch64-linux-gnu/sysroot ..
#
# 功能:
#   - 自动检测 aarch64 交叉编译器
#   - 配置 sysroot 路径
#   - 启用 NEON SIMD 优化
#   - 设置 CUDA 架构 (Jetson Nano/AGX Xavier/Orin)
#
# 作者: 周聪
# 日期: 2025
# =============================================================================

# 目标系统
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 交叉编译器前缀
set(CROSS_COMPILE aarch64-linux-gnu-)
find_program(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
find_program(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++)

if(NOT CMAKE_C_COMPILER)
    message(FATAL_ERROR "aarch64 cross compiler not found! Install: sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu")
endif()

# sysroot 路径 (需要包含目标平台的头文件和库)
if(DEFINED JETSON_ROOT)
    set(CMAKE_SYSROOT ${JETSON_ROOT})
    set(CMAKE_FIND_ROOT_PATH ${JETSON_ROOT})
else()
    message(WARNING "JETSON_ROOT not set. Using default sysroot.")
    set(CMAKE_SYSROOT /usr/aarch64-linux-gnu)
endif()

# 库搜索策略
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# AArch64 已内建 Advanced SIMD；-mfpu/-mfloat-abi 是 32 位 ARM 参数，
# 传给 aarch64-linux-gnu-g++ 会导致交叉编译失败。
# Jetson Orin NX uses Arm Cortex-A78AE (ARMv8.2-A).
add_compile_options(-march=armv8.2-a -mcpu=cortex-a78)

# CUDA 架构 (根据具体 Jetson 型号调整)
# Jetson Nano:    5.3 (Maxwell)
# Jetson TX2:     6.2 (Pascal)
# Jetson AGX Xavier: 7.2 (Volta)
# Jetson Orin:    8.7 (Ampere)
option(JETSON_ARCH "Jetson architecture version" "5.3")

# 链接选项
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt -lpthread")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -lrt -lpthread")

message(STATUS "Cross-compiling for Jetson (aarch64)")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  Sysroot: ${CMAKE_SYSROOT}")
message(STATUS "  Architecture: armv8-a + NEON")
message(STATUS "  CUDA Arch: ${JETSON_ARCH}")
