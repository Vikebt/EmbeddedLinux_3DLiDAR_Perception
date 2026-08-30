#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pathlib
import rospy
# rospkg is not explicitly used, but often useful for finding package paths
from pathlib import Path
from matplotlib import cm, pyplot as plt # 仅用于色彩映射表获取
import open3d as o3d
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import os
import glob
from collections import deque

# ROS 消息类型
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath # Renamed to RosPath to avoid conflict with pathlib.Path
import tf2_ros
import tf.transformations as tft # For quaternion and transformation matrix conversions
from visualization_msgs.msg import Marker, MarkerArray # 用于发布3D边界框
from scipy.spatial.transform import Rotation as R # 用于更方便地处理旋转
from nav_msgs.msg import OccupancyGrid # 用于发布2.5D栅格地图

# 导入工具函数
import utils

# ----------- 辅助函数 -----------

def get_sorted_files(directory: str, extension: str) -> list:
    """
    获取目录下特定扩展名的文件，并按文件名（通常代表时间戳）排序。
    Args:
        directory (str): 目标目录路径。
        extension (str): 文件扩展名 (例如 "ply", "csv")。
    Returns:
        list: 排序后的文件路径列表。
    """
    files = glob.glob(os.path.join(directory, f"*.{extension}"))
    files.sort() # 默认按字符串排序，适用于时间戳命名的文件
    return files

def load_imu_data_from_csv(csv_path: str) -> np.ndarray:
    """
    从CSV文件加载IMU数据，提取第一行姿态的四元数，并转换为旋转矩阵 R_G_I。
    R_G_I 表示从 IMU 本体系 (I) 到全局导航参考系 (G) 的旋转。
    Args:
        csv_path (str): IMU数据CSV文件的路径。
    Returns:
        np.ndarray: 3x3 旋转矩阵 (R_G_I)。
    """
    try:
        df = pd.read_csv(csv_path)
        if df.empty:
            rospy.logwarn(f"IMU CSV 文件 {csv_path} 为空。")
            return None # 或者抛出异常
        # CSV 列名假设为: timestamp,imu_q0,imu_q1,imu_q2,imu_q3,...
        # scipy.spatial.transform.Rotation.from_quat 需要 [x, y, z, w] 顺序的四元数。
        # 假设CSV中的 q0 是 w (标量部分), q1, q2, q3 是 x, y, z (矢量部分)。
        # 确保只取第一行数据作为该时间戳的IMU姿态
        q_xyzw = df[['imu_q1', 'imu_q2', 'imu_q3', 'imu_q0']].iloc[0].to_numpy()
        return Rotation.from_quat(q_xyzw).as_matrix() # 返回 R_G_I
    except FileNotFoundError:
        rospy.logerr(f"IMU CSV 文件未找到: {csv_path}")
        return None
    except pd.errors.EmptyDataError:
        rospy.logerr(f"IMU CSV 文件为空或格式错误: {csv_path}")
        return None
    except KeyError as e:
        rospy.logerr(f"IMU CSV 文件 {csv_path} 缺少必要的列: {e}")
        return None
    except Exception as e:
        rospy.logerr(f"加载IMU数据时发生未知错误 ({csv_path}): {e}")
        return None


def create_transform_matrix(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    根据旋转矩阵R和平移向量t创建4x4齐次变换矩阵。
    Args:
        R (np.ndarray): 3x3 旋转矩阵。
        t (np.ndarray): 3x1 或 1x3 平移向量。
    Returns:
        np.ndarray: 4x4 齐次变换矩阵。
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten() #确保t是1D数组
    return T

def o3d_pcd_to_ros_pcd2(o3d_pcd: o3d.geometry.PointCloud, frame_id: str, stamp: rospy.Time) -> PointCloud2:
    """
    将Open3D点云对象转换为ROS PointCloud2消息。
    Args:
        o3d_pcd (o3d.geometry.PointCloud): Open3D点云对象。
        frame_id (str): ROS消息头中的frame_id。
        stamp (rospy.Time): ROS消息头中的时间戳。
    Returns:
        PointCloud2: ROS PointCloud2 消息。
    """
    if o3d_pcd is None or o3d_pcd.is_empty() or not o3d_pcd.has_points():
        # rospy.logwarn_throttle(5.0, "o3d_pcd_to_ros_pcd2: 输入的Open3D点云为空或无效，将发布空点云消息。")
        header_empty = Header()
        header_empty.stamp = stamp
        header_empty.frame_id = frame_id
        return pc2.create_cloud_xyz32(header_empty, []) # 发布一个有效但空的PointCloud2


    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    
    points_xyz = np.asarray(o3d_pcd.points)
    
    if o3d_pcd.has_colors():
        colors_rgb_float = np.asarray(o3d_pcd.colors)
        if len(points_xyz) != len(colors_rgb_float):
            rospy.logwarn_throttle(5.0, "o3d_pcd_to_ros_pcd2: 点和颜色的数量不匹配，将发布无颜色的点云。")
            fields = [
                pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            ]
            ros_pcd = pc2.create_cloud(header, fields, points_xyz)
        else:
            # Open3D颜色是0-1的float, ROS PointCloud2的RGB通常是uint8或float32
            # 如果需要 uint8 (0-255), 则需要转换: colors_rgb_uint8 = (colors_rgb_float * 255).astype(np.uint8)
            # 然后将 r,g,b 字段打包成一个 uint32 rgb 字段，或者单独的 uint8 r,g,b 字段
            # 这里我们使用float32创建包含 x,y,z,r,g,b 字段的点云
            points_with_color = np.hstack((points_xyz, colors_rgb_float))
            fields = [
                pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name="r", offset=12, datatype=pc2.PointField.FLOAT32, count=1), # Red
                pc2.PointField(name="g", offset=16, datatype=pc2.PointField.FLOAT32, count=1), # Green
                pc2.PointField(name="b", offset=20, datatype=pc2.PointField.FLOAT32, count=1), # Blue
            ]
            ros_pcd = pc2.create_cloud(header, fields, points_with_color)
    else:
        fields = [
            pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]
        ros_pcd = pc2.create_cloud(header, fields, points_xyz)
    return ros_pcd


def color_pointcloud_by_z_inplace(pcd: o3d.geometry.PointCloud, colormap_name: str ='jet'):
    """
    为Open3D点云按Z轴高度赋色 (直接修改输入点云对象)。
    Args:
        pcd (o3d.geometry.PointCloud): 需要赋色的Open3D点云对象。
        colormap_name (str): Matplotlib的色彩映射表名称 (如 'jet', 'viridis', 'hsv')。
    """
    if pcd is None or pcd.is_empty() or not pcd.has_points():
        if pcd is not None and pcd.has_colors():
            pcd.colors = o3d.utility.Vector3dVector(np.array([]).reshape(-1,3))
        return
    
    points = np.asarray(pcd.points)
    z_values = points[:, 2]
    
    if len(z_values) == 0:
        if pcd.has_colors():
            pcd.colors = o3d.utility.Vector3dVector(np.array([]).reshape(-1,3))
        return

    z_min, z_max = np.min(z_values), np.max(z_values)
    
    if z_max == z_min: 
        normalized_z = np.full_like(z_values, 0.5) 
    else:
        normalized_z = (z_values - z_min) / (z_max - z_min)
    
    try:
        colormap = plt.get_cmap(colormap_name)
        colors_rgb = colormap(normalized_z)[:, :3] 
        pcd.colors = o3d.utility.Vector3dVector(colors_rgb)
    except ValueError:
        rospy.logwarn_throttle(10.0, f"无法获取名为 '{colormap_name}' 的色彩映射表。点云将不被着色。")
    except Exception as e:
        rospy.logerr(f"为点云着色时发生错误: {e}")

def segment_ground_ransac(pcd: o3d.geometry.PointCloud, 
                          distance_threshold: float, 
                          ransac_n: int, 
                          num_iterations: int) -> tuple:
    """
    使用RANSAC分割地面点云。
    返回: (ground_pcd, nonground_pcd)
    """
    if pcd is None or pcd.is_empty() or len(pcd.points) < ransac_n:
        # rospy.logdebug_throttle(5.0, "segment_ground_ransac: 输入点云为空或点数不足。")
        return o3d.geometry.PointCloud(), o3d.geometry.PointCloud(pcd) if pcd else o3d.geometry.PointCloud()
    try:
        # segment_plane 会找到最大的平面
        plane_model, inlier_indices = pcd.segment_plane(distance_threshold=distance_threshold,
                                                        ransac_n=ransac_n,
                                                        num_iterations=num_iterations)
        # 可选: 检查平面法向量是否大致朝上 (例如 Z 分量 > 0.7)
        # normal_z = plane_model[2]
        # if abs(normal_z) < 0.7: # 假设世界Z轴向上
        #     rospy.logdebug_throttle(5.0, f"RANSAC平面法向量Z分量 {normal_z:.2f} 过小，可能不是水平地面。将所有点视为非地面。")
        #     return o3d.geometry.PointCloud(), o3d.geometry.PointCloud(pcd)

    except RuntimeError as e:
        rospy.logwarn_throttle(5.0, f"地面分割RANSAC错误: {e}. 将所有点视为非地面。")
        return o3d.geometry.PointCloud(), o3d.geometry.PointCloud(pcd)
        
    ground_pcd = pcd.select_by_index(inlier_indices)
    nonground_pcd = pcd.select_by_index(inlier_indices, invert=True)
    return ground_pcd, nonground_pcd

def cluster_nonground_points(nonground_pcd: o3d.geometry.PointCloud, 
                             eps: float, 
                             min_points_dbscan: int, 
                             min_cluster_size_for_vis: int) -> o3d.geometry.PointCloud:
    """
    对非地面点云进行欧式聚类 (DBSCAN) 并为簇着色。
    返回: 带有簇颜色的点云。噪声点或过小的簇会被滤除。
    """
    if nonground_pcd is None or nonground_pcd.is_empty() or len(nonground_pcd.points) < min_points_dbscan:
        # rospy.logdebug_throttle(5.0, "cluster_nonground_points: 输入点云为空或点数不足以聚类。")
        return o3d.geometry.PointCloud()
    try:
        labels = np.array(nonground_pcd.cluster_dbscan(eps=eps, min_points=min_points_dbscan, print_progress=False))
    except RuntimeError as e:
        rospy.logwarn_throttle(5.0, f"DBSCAN聚类错误: {e}. 返回空聚类点云。")
        return o3d.geometry.PointCloud()

    max_label = labels.max() # 获取最大的簇ID (-1 表示噪声)
    clustered_points_list, clustered_colors_list = [], []
    
    if max_label >= 0: # 如果至少存在一个有效簇 (ID >= 0)
        points_np = np.asarray(nonground_pcd.points)
        try:
            colormap = plt.get_cmap('hsv') # 使用HSV色彩空间获取区分度较高的颜色
        except AttributeError: # Matplotlib < 3.5
            colormap = cm.get_cmap('hsv')
        
        for i in range(max_label + 1): # 遍历每个簇 (0 到 max_label)
            cluster_indices = np.where(labels == i)[0]
            if len(cluster_indices) >= min_cluster_size_for_vis: # 只处理足够大的簇
                cluster_points = points_np[cluster_indices]
                # 为簇生成颜色，确保 max_label > 0 时分母不为0
                color_idx_norm = float(i) / max(1, max_label) if max_label > 0 else 0.5 
                color = colormap(color_idx_norm)[:3] # 获取RGB
                
                clustered_points_list.append(cluster_points)
                clustered_colors_list.append(np.tile(color, (len(cluster_points), 1)))
    
    if not clustered_points_list: # 如果没有足够大的簇被保留
        # rospy.logdebug_throttle(5.0, "所有簇都太小或全是噪声点，返回空的聚类点云。")
        return o3d.geometry.PointCloud()

    final_points = np.vstack(clustered_points_list)
    final_colors = np.vstack(clustered_colors_list)
    
    pcd_clustered_colored = o3d.geometry.PointCloud()
    pcd_clustered_colored.points = o3d.utility.Vector3dVector(final_points)
    pcd_clustered_colored.colors = o3d.utility.Vector3dVector(final_colors)
    
    return pcd_clustered_colored

# ----------- 新增：目标识别相关 -----------
def get_axis_aligned_bounding_box_for_cluster(cluster_points: np.ndarray) -> o3d.geometry.AxisAlignedBoundingBox:
    """为给定的点簇计算轴对齐边界框"""
    if cluster_points.shape[0] == 0:
        return None
    pcd_cluster = o3d.geometry.PointCloud()
    pcd_cluster.points = o3d.utility.Vector3dVector(cluster_points)
    return pcd_cluster.get_axis_aligned_bounding_box()

def create_bounding_box_marker(bbox: o3d.geometry.AxisAlignedBoundingBox, 
                               obj_id: int, 
                               label: str, 
                               header: Header,
                               color: tuple = (0.0, 1.0, 0.0, 0.5)) -> Marker: # RGBA
    """创建一个表示3D边界框的ROS Marker"""
    marker = Marker()
    marker.header = header
    marker.ns = "detected_objects"
    marker.id = obj_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    center = bbox.get_center()
    extent = bbox.get_extent() # length, width, height (x, y, z extent)

    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]
    marker.pose.orientation.w = 1.0 # AABB, so no rotation relative to frame

    marker.scale.x = extent[0] if extent[0] > 0.01 else 0.01 # Ensure not zero
    marker.scale.y = extent[1] if extent[1] > 0.01 else 0.01
    marker.scale.z = extent[2] if extent[2] > 0.01 else 0.01

    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = float(color[3]) # Transparency

    marker.lifetime = rospy.Duration(0.5) # Marker持续时间 (秒), 根据发布频率调整

    # 添加文本标签 (可选, 在RViz中可能需要调整Marker显示选项才能看到)
    # text_marker = Marker()
    # text_marker.header = header
    # text_marker.ns = "object_labels"
    # text_marker.id = obj_id # 使用相同ID，但不同namespace
    # text_marker.type = Marker.TEXT_VIEW_FACING
    # text_marker.action = Marker.ADD
    # text_marker.pose.position.x = center[0]
    # text_marker.pose.position.y = center[1]
    # text_marker.pose.position.z = center[2] + extent[2] / 2 + 0.2 # 标签在包围盒上方
    # text_marker.text = label
    # text_marker.scale.z = 0.3 # 文本大小
    # text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0;
    # text_marker.lifetime = rospy.Duration(0.5)
    # (这里为了简化，不直接返回text_marker，但可以考虑单独发布或与bbox marker一起管理)

    return marker
# ----------- 新增：地面校正相关 -----------
def align_local_map_to_xy_plane_prior(local_map_pcd: o3d.geometry.PointCloud):
    # 使点云坐标系的地面与世界坐标系的地面平行
    # 绕x轴旋转-45°
    aligned_local_map_pcd = utils.rotate_point_cloud_around_x_axis(local_map_pcd, -45)
    # 绕y轴旋转-5°
    # aligned_local_map_pcd = utils.rotate_point_cloud_around_y_axis(aligned_local_map_pcd, -5)

    return aligned_local_map_pcd

def align_local_map_to_xy_plane(local_map_pcd: o3d.geometry.PointCloud,
                                ransac_dist_thresh: float = 0.1,
                                ransac_n: int = 3,
                                ransac_iters: int = 100,
                                normal_similarity_thresh: float = 1e-5) -> tuple: # 新增阈值
    """
    尝试将局部地图旋转，使其估计出的地面与XOY平面平行。
    Args:
        local_map_pcd (o3d.geometry.PointCloud): 输入的局部地图点云 (在世界坐标系W下)。
        ransac_dist_thresh, ransac_n, ransac_iters: RANSAC平面拟合参数。
        normal_similarity_thresh (float): 用于判断法向量是否已对齐或反平行的阈值。
    Returns:
        tuple: (aligned_local_map_pcd, alignment_transform, ground_plane_model)
    """
    if local_map_pcd is None or local_map_pcd.is_empty() or len(local_map_pcd.points) < ransac_n * 3 : # 稍微增加点数需求
        rospy.logdebug_throttle(5.0, "align_local_map: 输入点云不足以进行地面估计和校正。")
        return local_map_pcd, np.eye(4), None

    try:
        ground_plane_model, inlier_indices = local_map_pcd.segment_plane(
            distance_threshold=ransac_dist_thresh,
            ransac_n=ransac_n,
            num_iterations=ransac_iters
        )
        ground_normal_raw = np.array(ground_plane_model[:3]) # 未归一化的原始法向量
        rospy.logdebug(f"  align_local_map: 初始估计地面法向量: {ground_normal_raw.round(3)}")

        norm_ground_normal = np.linalg.norm(ground_normal_raw)
        if norm_ground_normal < 1e-6: # 检查原始法向量是否为零
             rospy.logwarn_throttle(5.0, "align_local_map: RANSAC估计的地面法向量为零或过小，无法校正。")
             return local_map_pcd, np.eye(4), ground_plane_model

        ground_normal_normalized = ground_normal_raw / norm_ground_normal

        # 确保地面法向量的Z分量为正，便于后续与 (0,0,1) 对齐
        if ground_normal_normalized[2] < 0:
            ground_normal_normalized = -ground_normal_normalized
            # 如果需要，也可以反转原始 ground_plane_model 的符号
            # ground_plane_model = [-x for x in ground_plane_model]

        target_normal = np.array([0.0, 0.0, 1.0])

        # 检查是否已经对齐
        dot_product = np.dot(ground_normal_normalized, target_normal)
        if np.abs(dot_product - 1.0) < normal_similarity_thresh:
            rospy.logdebug_throttle(5.0, "  align_local_map: 地面法向量已与目标Z轴对齐，无需旋转。")
            return local_map_pcd, np.eye(4), ground_plane_model # 无需旋转

        # 检查是否反平行 (理论上，因为我们强制 Z>0，这种情况不应该发生，但双重检查无妨)
        # if np.abs(dot_product + 1.0) < normal_similarity_thresh:
        #     rospy.logwarn_throttle(5.0, "  align_local_map: 地面法向量与目标Z轴反平行。")
        #     # 这种情况，align_vectors 仍会工作，但会产生警告。
        #     # 或者可以自定义一个旋转，例如绕X轴或Y轴旋转180度。
        #     # rotation_matrix = R.from_euler('x', 180, degrees=True).as_matrix()
        #     # pass # 让 align_vectors 处理

        # 计算旋转
        # align_vectors([vec_a], [vec_b]) finds rotation R such that R @ vec_a aligns with vec_b
        try:
            # The first argument is a list of vectors to be rotated.
            # The second argument is a list of target vectors.
            rotation, _ = R.align_vectors([ground_normal_normalized], [target_normal])
            rotation_matrix = rotation.as_matrix()
            rospy.logdebug(f"  align_local_map: 计算得到的对齐旋转矩阵:\n{rotation_matrix.round(3)}")
        except ValueError as e: # 例如，当向量无法对齐时（如一个为零）
            rospy.logwarn_throttle(5.0, f"align_local_map: scipy.align_vectors 错误: {e}. 将不进行旋转。")
            return local_map_pcd, np.eye(4), ground_plane_model


        map_center = local_map_pcd.get_center()
        T_align_rot_only = np.eye(4)
        T_align_rot_only[:3, :3] = rotation_matrix
        
        T_translate_to_origin = create_transform_matrix(np.eye(3), -map_center)
        T_translate_back = create_transform_matrix(np.eye(3), map_center)
        
        alignment_transform = T_translate_back @ T_align_rot_only @ T_translate_to_origin
        rospy.logdebug(f"  align_local_map: 最终对齐变换矩阵 (绕中心 {map_center.round(2)} 旋转):\n{alignment_transform.round(3)}")

        aligned_local_map_pcd = o3d.geometry.PointCloud(local_map_pcd)
        aligned_local_map_pcd.transform(alignment_transform)
        
        return aligned_local_map_pcd, alignment_transform, ground_plane_model

    except Exception as e:
        rospy.logerr_throttle(5.0, f"align_local_map: 校正局部地图时发生未知错误: {e}")
        return local_map_pcd, np.eye(4), None
# ----------- 新增：2.5D栅格地图构建相关 -----------
def create_25d_grid_map(points_W: np.ndarray, # 输入点云的XYZ坐标 (N,3)
                        current_lidar_pos_xy_W: np.ndarray, # 当前LiDAR在世界系下的XY位置 (用于地图中心)
                        grid_resolution: float, # 每个栅格的分辨率 (米/单元)
                        map_size_meters: float, # 地图的总边长 (米), 地图将是方形的
                        min_z_val: float = -2.0, # 用于归一化高度的最小Z值
                        max_z_val: float = 5.0   # 用于归一化高度的最大Z值
                        ) -> tuple: # (OccupancyGrid, origin_x_world, origin_y_world)
    """
    从给定的点云构建一个2.5D最大高度栅格地图。
    地图以 current_lidar_pos_xy_W 为中心。
    Args:
        points_W (np.ndarray): (N,3) NumPy数组，表示世界坐标系下的点云XYZ坐标。
        current_lidar_pos_xy_W (np.ndarray): (2,) NumPy数组，LiDAR在世界XY平面的位置。
        grid_resolution (float): 栅格地图的分辨率 (米/单元)。
        map_size_meters (float): 地图的边长 (米)，地图将以此为中心。
        min_z_val (float): 用于将高度值归一化到0-100的最小Z参考值。
        max_z_val (float): 用于将高度值归一化到0-100的最大Z参考值。
    Returns:
        tuple: (OccupancyGrid or None, float, float)
               - nav_msgs.OccupancyGrid 消息，如果成功。
               - 地图原点在世界坐标系中的X坐标。
               - 地图原点在世界坐标系中的Y坐标。
               如果点云为空或参数无效，则返回 (None, 0, 0)。
    """
    if points_W is None or points_W.shape[0] == 0 or grid_resolution <= 0 or map_size_meters <= 0:
        rospy.logdebug_throttle(5.0, "create_25d_grid_map: 输入点云为空或参数无效。")
        return None, 0.0, 0.0
    

    actual_z_min_local = np.min(points_W[:, 2])
    actual_z_max_local = np.max(points_W[:, 2])
    rospy.loginfo(f"  GridMap: Actual local Z range: [{actual_z_min_local:.2f}, {actual_z_max_local:.2f}]")
    rospy.loginfo(f"  GridMap: Using Z norm params: min_ref={min_z_val:.2f}, max_ref={max_z_val:.2f}")

    # 使用参数传入的范围进行归一化，但可以根据实际情况调整或给出警告
    # 如果希望更动态，可以将 min_z_norm 和 max_z_norm 设置为 actual_z_min_local 和 actual_z_max_local
    # 但通常我们想有一个固定的参考范围，除非场景高度变化极大
    min_z_for_norm = min_z_val
    max_z_for_norm = max_z_val

    if actual_z_max_local < min_z_for_norm or actual_z_min_local > max_z_for_norm:
        rospy.logwarn_throttle(10.0, f"  GridMap: Actual Z range [{actual_z_min_local:.2f}, {actual_z_max_local:.2f}] is outside or far from normalization range [{min_z_for_norm:.2f}, {max_z_for_norm:.2f}]. Grid map may appear flat or poorly scaled.")
        # 可以选择在这里动态调整 min_z_for_norm 和 max_z_for_norm
        # 例如: min_z_for_norm = actual_z_min_local - 0.1
        #       max_z_for_norm = actual_z_max_local + 0.1

    if max_z_for_norm <= min_z_for_norm: # 防止除零或负范围
        rospy.logwarn_throttle(5.0, f"  GridMap: Invalid normalization range max_z_ref ({max_z_for_norm:.2f}) <= min_z_ref ({min_z_for_norm:.2f}). Will result in a flat map.")
        # 强制一个小的范围，避免崩溃，但结果会是平的
        max_z_for_norm = min_z_for_norm + 0.1 


    # 确定地图的尺寸 (单元数)
    map_width_cells = int(np.ceil(map_size_meters / grid_resolution))
    map_height_cells = int(np.ceil(map_size_meters / grid_resolution))

    # 计算地图原点在世界坐标系中的位置
    # 地图数据[0,0]对应于地图的左下角
    # 地图中心在 (map_width_cells/2, map_height_cells/2)
    # 我们希望这个中心对应于 current_lidar_pos_xy_W
    map_origin_x_world = current_lidar_pos_xy_W[0] - (map_width_cells / 2.0) * grid_resolution
    map_origin_y_world = current_lidar_pos_xy_W[1] - (map_height_cells / 2.0) * grid_resolution

    # 初始化高度图 (存储每个单元的最大高度)
    # 使用一个特殊值（如负无穷）表示单元为空
    height_map_raw = np.full((map_height_cells, map_width_cells), -np.inf, dtype=np.float32)

    # 将世界坐标系下的点转换为栅格索引
    # (p_x - origin_x) / res = grid_idx_x
    grid_indices_x = np.floor((points_W[:, 0] - map_origin_x_world) / grid_resolution).astype(int)
    grid_indices_y = np.floor((points_W[:, 1] - map_origin_y_world) / grid_resolution).astype(int)
    points_z_values = points_W[:, 2]

    # 筛选出在栅格地图范围内的点
    valid_indices_mask = (
        (grid_indices_x >= 0) & (grid_indices_x < map_width_cells) &
        (grid_indices_y >= 0) & (grid_indices_y < map_height_cells)
    )
    
    valid_grid_x = grid_indices_x[valid_indices_mask]
    valid_grid_y = grid_indices_y[valid_indices_mask]
    valid_points_z = points_z_values[valid_indices_mask]

    if len(valid_grid_x) == 0:
        rospy.logdebug_throttle(5.0, "create_25d_grid_map: 没有点落在定义的栅格地图区域内。")
        # 仍然创建一个空的 OccupancyGrid，但数据为-1 (未知)
        grid_data = np.full(map_width_cells * map_height_cells, -1, dtype=np.int8)
    else:
        # 填充高度图：对于每个有效的点，更新其所在单元的最大高度
        # 使用 np.maximum.at 来高效地进行原地更新
        # 注意：(y,x) 顺序，因为图像/数组通常是 (row, col) -> (y, x)
        np.maximum.at(height_map_raw, (valid_grid_y, valid_grid_x), valid_points_z)

        # 将原始高度值归一化到0-100，并处理空单元
        # 空单元 (值为-inf) 应映射为-1 (未知)
        # 其他值裁剪并归一化
        
        occupancy_values = np.zeros_like(height_map_raw, dtype=np.int8)
        
        # 非空单元
        has_value_mask = height_map_raw > -np.inf
        
        # 裁剪Z值
        clipped_z = np.clip(height_map_raw[has_value_mask], min_z_for_norm, max_z_for_norm)
        
        # 归一化到0-1范围 (防止除零)
        if max_z_val > min_z_for_norm:
            normalized_z_01 = (clipped_z - min_z_for_norm) / (max_z_for_norm - min_z_for_norm)
        else: # 如果 max_z_for_norm == min_z_for_norm, 所有有效值都映射到50
            normalized_z_01 = np.full_like(clipped_z, 0.5)

        # 映射到0-100
        occupancy_values[has_value_mask] = (normalized_z_01 * 100).astype(np.int8)
        
        # 空单元标记为未知 (-1)
        occupancy_values[~has_value_mask] = -1
        
        # OccupancyGrid 的 data 是一维数组，行优先 (row-major order)
        grid_data = occupancy_values.flatten(order='C') # C-style (row-major)

    # 创建 OccupancyGrid 消息
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now() # 使用当前时间
    grid_msg.header.frame_id = "odom" # 栅格地图在 odom 坐标系下定义
    
    grid_msg.info.resolution = grid_resolution
    grid_msg.info.width = map_width_cells
    grid_msg.info.height = map_height_cells
    
    grid_msg.info.origin.position.x = map_origin_x_world
    grid_msg.info.origin.position.y = map_origin_y_world
    grid_msg.info.origin.position.z = 0.0 # 2.5D地图，通常原点Z为0
    grid_msg.info.origin.orientation.w = 1.0 # 无旋转

    grid_msg.data = grid_data.tolist() # 需要是 list of int8

    if grid_data.size > 0:
        rospy.loginfo(f"  Grid data stats: Min={np.min(grid_data)}, Max={np.max(grid_data)}, Mean={np.mean(grid_data):.2f}")
        unique_vals, counts = np.unique(grid_data, return_counts=True)
        rospy.loginfo(f"  Grid data unique values (up to 10): {dict(zip(unique_vals[:10], counts[:10]))}")

    return grid_msg, map_origin_x_world, map_origin_y_world

class LidarImuOdometryNode:
    """
    LiDAR-IMU里程计节点，通过融合LiDAR扫描和IMU数据来估计传感器运动。
    它处理一系列的PLY点云文件和对应的IMU数据(CSV格式)，
    使用IMU数据作为ICP的初始猜测，进行scan-to-map匹配。
    节点发布里程计、TF变换、路径以及处理后的局部地图点云。
    """
    def __init__(self):
        """
        初始化节点，加载参数，设置发布器和状态变量。
        """
        rospy.init_node('lidar_imu_odometry_node')
        rospy.loginfo("LiDAR-IMU Odometry Node (with Local Map Segmentation/Clustering) - Initializing...")

        # ----------- 加载ROS参数 -----------
        self.base_data_path_str = rospy.get_param('~base_data_path', '/home/nuaa/Ayra/data_lidargpsimu/20250417_103748')
        self.pointcloud_dir_name = rospy.get_param('~pointcloud_dir_name', 'pointcloud')
        self.sensors_dir_name = rospy.get_param('~sensors_dir_name', 'sensors')
        
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') 
        self.base_link_frame_id = rospy.get_param('~lidar_frame_id', 'base_link') 

        self.n_map_frames = rospy.get_param('~n_map_frames', 10) 
        # self.map_crop_radius = rospy.get_param('~map_crop_radius', 50.0) # 旧的，可能用于ICP目标裁剪
        self.local_map_crop_radius_for_processing = rospy.get_param('~local_map_crop_radius_for_processing', 30.0) # 新增：用于处理和发布的局部地图半径
        self.colormap_name_global_map = rospy.get_param('~colormap_name_global_map', 'viridis') # 为可选的全局Z轴着色地图用
        
        self.voxel_size_map_history = rospy.get_param('~voxel_size_map_history', 0.3) 
        # self.voxel_size_cropped_map = rospy.get_param('~voxel_size_cropped_map', 0.3) # 旧的，可能用于ICP目标裁剪
        self.voxel_size_scan = rospy.get_param('~voxel_size_scan', 0.15) 
        
        self.icp_max_correspondence_distance = rospy.get_param('~icp_max_correspondence_distance', 1.0) 
        self.icp_max_iterations = rospy.get_param('~icp_max_iterations', 15) 
        self.min_icp_fitness = rospy.get_param('~min_icp_fitness', 0.15) 
        
        self.processing_rate = rospy.get_param('~processing_rate', 10.0) 

        # 地面分割参数 (应用于裁剪出的局部地图)
        self.ground_seg_distance_threshold = rospy.get_param('~ground_seg_distance_threshold', 0.20)
        self.ground_seg_ransac_n = rospy.get_param('~ground_seg_ransac_n', 3)
        self.ground_seg_num_iterations = rospy.get_param('~ground_seg_num_iterations', 100)

        # 非地面点云聚类参数 (应用于裁剪出的局部地图的非地面部分)
        self.nonground_cluster_eps = rospy.get_param('~nonground_cluster_eps', 0.8)
        self.nonground_cluster_min_points_dbscan = rospy.get_param('~nonground_cluster_min_points_dbscan', 20) # DBSCAN参数
        self.nonground_cluster_min_points_for_vis = rospy.get_param('~nonground_cluster_min_points_for_vis', 25) # 可视化时簇的最小点数

        T_imu_lidar_list = rospy.get_param('~T_imu_lidar', [
            [-0.01325259, 0.99988831, -0.0069081, -0.19803817],
            [-0.99962256, -0.01308217, 0.024158, 0.02344096],
            [0.02406492, 0.00722565, 0.99968428, -0.07190712],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.T_imu_lidar = np.array(T_imu_lidar_list)
        self.T_lidar_imu = np.linalg.inv(self.T_imu_lidar)
        
        # ——————————————————————————————新增目标识别功能 ——————————————————
        rospy.loginfo("LiDAR-IMU Odometry Node (with Local Map Segmentation/Clustering & Basic Object Recognition) - Initializing...")

        # 新增参数：目标识别相关 (非常基础)
        self.enable_basic_object_recognition = rospy.get_param('~enable_basic_object_recognition', True)
        self.vehicle_like_min_dim = rospy.get_param('~vehicle_like_min_dim', [1.5, 0.8, 0.8]) # [min_len, min_wid, min_hgt]
        self.vehicle_like_max_dim = rospy.get_param('~vehicle_like_max_dim', [60.0, 60.0, 60.0]) # [max_len, max_wid, max_hgt]
        self.vehicle_like_min_points = rospy.get_param('~vehicle_like_min_points', 50)
        
        self.small_object_max_dim = rospy.get_param('~small_object_max_dim', [1.0, 1.0, 1.5]) # [max_len, max_wid, max_hgt]
        self.small_object_min_points = rospy.get_param('~small_object_min_points', 10)

        # 新增参数：地面校正相关
        self.enable_local_map_alignment = rospy.get_param('~enable_local_map_alignment', True)
        self.align_ransac_dist_thresh = rospy.get_param('~align_ransac_dist_thresh', 0.15) # RANSAC地面估计的距离阈值
        self.align_ransac_iters = rospy.get_param('~align_ransac_iters', 50) # RANSAC迭代次数

        # 新增参数：2.5D栅格地图相关
        self.enable_25d_grid_map = rospy.get_param('~enable_25d_grid_map', True)
        self.grid_map_resolution = rospy.get_param('~grid_map_resolution', 2) # 米/单元
        self.grid_map_size_meters = rospy.get_param('~grid_map_size_meters', 400.0) # 地图边长 (米)
        self.grid_map_min_z_ref = rospy.get_param('~grid_map_min_z_ref', -100.0) # 用于高度归一化的Z值下限
        self.grid_map_max_z_ref = rospy.get_param('~grid_map_max_z_ref', -70.0)  # 用于高度归一化的Z值上限

        # ----------- 文件路径设置 -----------
        self.base_data_path = pathlib.Path(self.base_data_path_str)
        self.pointcloud_dir = self.base_data_path / self.pointcloud_dir_name
        self.sensors_dir = self.base_data_path / self.sensors_dir_name

        # ----------- ROS 发布器 -----------
        # 发布处理后的局部地图 (分割+聚类)
        self.processed_local_map_pub = rospy.Publisher('~processed_local_map', PointCloud2, queue_size=2) 
        # 可选：发布完整的、按Z轴着色的历史地图
        self.global_map_colored_pub = rospy.Publisher('~global_map_colored', PointCloud2, queue_size=2)

        self.odom_pub = rospy.Publisher('~odometry', Odometry, queue_size=10)
        self.path_pub = rospy.Publisher('~path', RosPath, queue_size=2) 
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() 

        # 新增：用于发布识别出的目标边界框
        self.object_markers_pub = rospy.Publisher('~detected_object_markers', MarkerArray, queue_size=2)
        # 新增：用于发布2.5D栅格地图
        self.grid_map_pub = rospy.Publisher('~height_grid_map', OccupancyGrid, queue_size=2)
        
        # ----------- 状态变量 -----------
        self.current_lidar_pose_W = np.eye(4) 
        self.prev_imu_rotation_G_I = None    
        self.map_scans_W_deque = deque(maxlen=self.n_map_frames if self.n_map_frames > 0 else None) 
        
        self.path_msg = RosPath()
        self.path_msg.header.frame_id = self.odom_frame_id

        self.first_frame = True 
        self.frame_count = 0 

        self.ply_files = get_sorted_files(str(self.pointcloud_dir), "ply")
        if not self.ply_files:
            rospy.logfatal(f"错误：在 '{self.pointcloud_dir}' 中未找到PLY文件。节点将关闭。")
            rospy.signal_shutdown("No PLY files found, shutting down.")
            return # 确保在此处返回，避免后续代码执行
        
        rospy.loginfo(f"找到 {len(self.ply_files)} 个PLY文件待处理。")


    def add_scan_to_history(self, new_scan_W: o3d.geometry.PointCloud):
        if new_scan_W is not None and not new_scan_W.is_empty():
            self.map_scans_W_deque.append(new_scan_W)

    def build_map_from_history(self, voxel_size: float) -> o3d.geometry.PointCloud:
        if not self.map_scans_W_deque:
            return o3d.geometry.PointCloud()

        # 优化：如果只有一个扫描，且不需要下采样，直接返回其副本
        if len(self.map_scans_W_deque) == 1 and voxel_size <= 0:
            return o3d.geometry.PointCloud(self.map_scans_W_deque[0])
            
        # 高效合并，避免多次复制大量点
        all_points_list = [np.asarray(pcd_W.points) for pcd_W in self.map_scans_W_deque if not pcd_W.is_empty()]
        if not all_points_list:
            return o3d.geometry.PointCloud()
        
        combined_points_np = np.vstack(all_points_list)
        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(combined_points_np)

        if voxel_size > 0 and not combined_pcd.is_empty() and len(combined_pcd.points) > 0:
            # rospy.logdebug(f"Build map: Before voxel downsample: {len(combined_pcd.points)} points.")
            combined_pcd_down = combined_pcd.voxel_down_sample(voxel_size)
            # rospy.logdebug(f"Build map: After voxel downsample ({voxel_size}m): {len(combined_pcd_down.points)} points.")
            return combined_pcd_down
        return combined_pcd

    def crop_map_around_pose(self, 
                             source_map_pcd: o3d.geometry.PointCloud, 
                             center_pose_W_translation: np.ndarray, # 更改为仅接收平移向量
                             radius: float, 
                             target_voxel_size: float = 0.0) -> o3d.geometry.PointCloud:
        if source_map_pcd is None or source_map_pcd.is_empty() or not source_map_pcd.has_points():
            # rospy.logdebug_throttle(5.0, "crop_map_around_pose: 源地图为空或无点，返回空点云。")
            return o3d.geometry.PointCloud()

        center_translation_W = center_pose_W_translation[:3] # 确保是3维向量
        
        try:
            kdtree = o3d.geometry.KDTreeFlann(source_map_pcd)
            k, idx, _ = kdtree.search_radius_vector_3d(center_translation_W, radius)
        except RuntimeError as e:
            rospy.logwarn_throttle(5.0, f"crop_map_around_pose: KDTree搜索错误: {e}. 可能源点云点数过少。返回空点云。")
            return o3d.geometry.PointCloud()

        if not idx: 
            # rospy.logdebug_throttle(5.0, f"crop_map_around_pose: 在半径 {radius} 内未找到点。中心: {center_translation_W.round(2)}")
            return o3d.geometry.PointCloud()
            
        cropped_pcd = source_map_pcd.select_by_index(idx)
        # rospy.logdebug(f"crop_map_around_pose: 裁剪得到 {len(cropped_pcd.points)} 点。")

        if target_voxel_size > 0 and not cropped_pcd.is_empty() and len(cropped_pcd.points) > 0:
            cropped_pcd = cropped_pcd.voxel_down_sample(target_voxel_size)
            # rospy.logdebug(f"crop_map_around_pose: 裁剪后下采样得到 {len(cropped_pcd.points)} 点。")
        return cropped_pcd
    
    # 基础的目标识别：遍历点云簇，根据几何规则分类并创建Marker
    def recognize_objects_from_clusters_basic(self, 
                                              nonground_clustered_pcd: o3d.geometry.PointCloud, 
                                              header_for_markers: Header) -> MarkerArray:
        """
        常基础的目标识别：遍历点云簇，根据几何规则分类并创建Marker。
        Args:
            nonground_clustered_pcd (o3d.geometry.PointCloud): 包含多个已着色簇的非地面点云。
                                                               假设每个簇已经有了不同的颜色。
            header_for_markers (Header): 用于创建Marker的ROS消息头。
        Returns:
            MarkerArray: 包含所有识别出的目标边界框的MarkerArray。
        """
        marker_array = MarkerArray()
        if nonground_clustered_pcd is None or nonground_clustered_pcd.is_empty() or not nonground_clustered_pcd.has_colors():
            return marker_array # 没有簇或颜色信息无法区分簇

        points = np.asarray(nonground_clustered_pcd.points)
        colors = np.asarray(nonground_clustered_pcd.colors)
        
        # 通过颜色来区分不同的簇 (这是一个简化的假设，cluster_nonground_points函数保证了这点)
        unique_colors = np.unique(colors, axis=0)
        obj_id_counter = 0

        for color_val in unique_colors:
            # 找到属于当前颜色的所有点 (即一个簇)
            # np.all(colors == color_val, axis=1) 比较浮点数颜色可能不精确
            # 更鲁棒的方法是在 cluster_nonground_points 中直接返回簇的索引列表或点列表
            # 这里我们使用颜色，需要注意浮点比较的精度问题
            # 使用一个小的容差来比较颜色
            color_tolerance = 1e-5
            cluster_indices = np.where(np.linalg.norm(colors - color_val, axis=1) < color_tolerance)[0]

            if len(cluster_indices) == 0:
                continue
                
            current_cluster_points = points[cluster_indices]

            # 为该簇计算AABB
            aabb = get_axis_aligned_bounding_box_for_cluster(current_cluster_points)
            if aabb is None:
                continue

            extent = aabb.get_extent() # x_len, y_len, z_len
            num_points_in_cluster = len(current_cluster_points)
            label = "unknown"
            marker_color = (0.7, 0.7, 0.7, 0.5) # 默认为灰色

            # 规则1: "Vehicle-like"
            if (self.vehicle_like_min_dim[0] < extent[0] < self.vehicle_like_max_dim[0] and
                self.vehicle_like_min_dim[1] < extent[1] < self.vehicle_like_max_dim[1] and
                self.vehicle_like_min_dim[2] < extent[2] < self.vehicle_like_max_dim[2] and
                num_points_in_cluster > self.vehicle_like_min_points):
                label = "vehicle_like"
                marker_color = (0.0, 1.0, 0.0, 0.5) # 绿色
            # 规则2: "Small_object"
            elif (extent[0] < self.small_object_max_dim[0] and
                  extent[1] < self.small_object_max_dim[1] and
                  extent[2] < self.small_object_max_dim[2] and
                  num_points_in_cluster > self.small_object_min_points):
                label = "small_object"
                marker_color = (0.0, 0.0, 1.0, 0.5) # 蓝色
            
            if label != "unknown":
                bbox_marker = create_bounding_box_marker(aabb, obj_id_counter, label, header_for_markers, marker_color)
                marker_array.markers.append(bbox_marker)
                obj_id_counter += 1
                rospy.logdebug(f"  Recognized: ID {obj_id_counter-1}, Label: {label}, Points: {num_points_in_cluster}, Extent: {extent.round(2)}")

        # 清除旧的markers (如果上一帧的物体这一帧没有了)
        # 这是通过 Marker.ADD 配合 lifetime 实现的，或者可以手动发送 DELETEALL
        # 如果需要更精确的跟踪，需要实现物体ID的跟踪和关联

        return marker_array
    def run(self):
        rate = rospy.Rate(self.processing_rate)
        file_iterator = iter(self.ply_files)
                
        while not rospy.is_shutdown():
            try:
                ply_file = next(file_iterator)
            except StopIteration:
                rospy.loginfo("所有PLY文件处理完毕。节点将保持运行。")
                break 

            current_time_ros = rospy.Time.now()
            timestamp_str = os.path.splitext(os.path.basename(ply_file))[0]
            sensor_file = self.sensors_dir / f"{timestamp_str}_sensors.csv"

            if not sensor_file.exists():
                rospy.logwarn(f"警告：找不到传感器文件 {sensor_file}，跳过帧 {timestamp_str}")
                continue

            rospy.loginfo(f"处理帧: {timestamp_str} ({self.frame_count + 1}/{len(self.ply_files)})")

            # 1. 加载当前帧LiDAR点云 (Lk)
            current_scan_L_raw = o3d.io.read_point_cloud(str(ply_file))
            if current_scan_L_raw.is_empty() or len(current_scan_L_raw.points) < 10:
                rospy.logwarn(f"警告: PLY文件 {ply_file} 为空或点数过少，跳过。")
                continue
            
            current_scan_L = o3d.geometry.PointCloud(current_scan_L_raw) # 创建副本
            if self.voxel_size_scan > 0:
                current_scan_L = current_scan_L.voxel_down_sample(self.voxel_size_scan)
                if current_scan_L.is_empty() or len(current_scan_L.points) < 10:
                    rospy.logwarn(f"警告: PLY文件 {ply_file} 下采样后为空或点数过少，跳过。")
                    continue
            
            current_imu_rotation_G_I = load_imu_data_from_csv(str(sensor_file))
            if current_imu_rotation_G_I is None: # 如果IMU加载失败
                rospy.logwarn(f"IMU数据加载失败 {sensor_file}，跳过帧 {timestamp_str}")
                continue


            # 2. 里程计核心逻辑 (获取 refined_T_W_Lk)
            refined_T_W_Lk = np.eye(4) 

            if self.first_frame:
                self.current_lidar_pose_W = np.eye(4)
                self.prev_imu_rotation_G_I = current_imu_rotation_G_I
                refined_T_W_Lk = self.current_lidar_pose_W
                self.first_frame = False
            else:
                delta_rot_imu_body = np.linalg.inv(self.prev_imu_rotation_G_I) @ current_imu_rotation_G_I
                delta_T_Lk_1_Lk_imu = self.T_lidar_imu @ create_transform_matrix(delta_rot_imu_body, np.zeros(3)) @ self.T_imu_lidar
                initial_T_W_Lk_guess = self.current_lidar_pose_W @ delta_T_Lk_1_Lk_imu

                map_for_icp_target_W = self.build_map_from_history(self.voxel_size_map_history)
                # rospy.logdebug(f"  ICP目标地图 (W): {len(map_for_icp_target_W.points)} 点")
                
                refined_T_W_Lk = initial_T_W_Lk_guess 
                if not map_for_icp_target_W.is_empty() and len(map_for_icp_target_W.points) >= 20 and \
                   not current_scan_L.is_empty() and len(current_scan_L.points) >=10 :
                    
                    source_icp_L = o3d.geometry.PointCloud(current_scan_L) # ICP的source
                    if not source_icp_L.has_normals():
                        source_icp_L.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_scan * 2.5, max_nn=30))
                    if not map_for_icp_target_W.has_normals():
                        map_for_icp_target_W.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_map_history * 1.5, max_nn=30))
                    
                    # rospy.logdebug(f"  ICP: source(Lk) {len(source_icp_L.points)}, target(W) {len(map_for_icp_target_W.points)}")
                    try:
                        reg_result = o3d.pipelines.registration.registration_generalized_icp(
                            source_icp_L, map_for_icp_target_W, 
                            self.icp_max_correspondence_distance, initial_T_W_Lk_guess,
                            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=self.icp_max_iterations)
                        )
                        if reg_result.fitness > self.min_icp_fitness and reg_result.inlier_rmse < self.icp_max_correspondence_distance * 1.2 :
                            refined_T_W_Lk = reg_result.transformation
                            # rospy.logdebug(f"  ICP成功: fitness={reg_result.fitness:.4f}, rmse={reg_result.inlier_rmse:.4f}")
                        else:
                            # rospy.logwarn(f"警告: ICP fitness ({reg_result.fitness:.4f}) 或 RMSE ({reg_result.inlier_rmse:.4f}) 不佳. 使用IMU预测.")
                            pass 
                    except RuntimeError as e:
                        rospy.logerr(f"ICP 运行时发生错误: {e}. 使用IMU预测结果。")
                # else:
                    # rospy.logwarn("ICP源或目标点数不足，仅使用IMU预测。")
                
                self.current_lidar_pose_W = refined_T_W_Lk 
                self.prev_imu_rotation_G_I = current_imu_rotation_G_I

            # 3. 更新历史地图数据 (将当前帧配准后的点云加入deque)
            scan_to_history_W = o3d.geometry.PointCloud(current_scan_L).transform(refined_T_W_Lk)
            self.add_scan_to_history(scan_to_history_W)

            # 4. 构建完整的历史地图 (用于裁剪和可选的全局地图发布)
            history_map_W_complete = self.build_map_from_history(self.voxel_size_map_history)
            rospy.logdebug(f"  完整历史地图 (history_map_W_complete): {len(history_map_W_complete.points)} 点")

            # 5. 可选：发布完整的、按Z轴着色的历史地图
            # if history_map_W_complete.has_points():
            #     display_global_map_W = o3d.geometry.PointCloud(history_map_W_complete) # 创建副本进行着色
            #     if self.colormap_name_global_map:
            #         color_pointcloud_by_z_inplace(display_global_map_W, self.colormap_name_global_map)
            #     global_map_msg = o3d_pcd_to_ros_pcd2(display_global_map_W, self.odom_frame_id, current_time_ros)
            #     self.global_map_colored_pub.publish(global_map_msg)

            # 6. 裁剪局部地图并进行处理 (分割、聚类) 用于发布
            rospy.logdebug(f"  处理局部地图 (中心: {refined_T_W_Lk[:3,3].round(2)}, 半径: {self.local_map_crop_radius_for_processing}m)")
            current_pose_translation_W = refined_T_W_Lk[:3, 3]
            
            # 从完整的、未着色的 history_map_W_complete 中裁剪
            local_map_for_processing_W_original = self.crop_map_around_pose(
                history_map_W_complete, 
                current_pose_translation_W, 
                self.local_map_crop_radius_for_processing,
                target_voxel_size=0.0 # 不在这里下采样，保持原始密度进行校正和分割
            )
            rospy.logdebug(f"  裁剪得到原始局部地图 (W): {len(local_map_for_processing_W_original.points)} 点")

            # === 新增：局部地图地面校正 ===
            local_map_for_processing_W_aligned = o3d.geometry.PointCloud(local_map_for_processing_W_original) # 默认等于原始
            alignment_transform_applied = np.eye(4) # 记录实际应用的变换
            # ground_plane_model_in_W = None # 记录在原始W系下的地面模型

            if self.enable_local_map_alignment and local_map_for_processing_W_original.has_points():
                rospy.logdebug("  尝试对局部地图进行地面校正...")
                # aligned_map_temp, T_align_actual, plane_model_temp = align_local_map_to_xy_plane(
                #     local_map_for_processing_W_original,
                #     ransac_dist_thresh=self.align_ransac_dist_thresh,
                #     ransac_iters=self.align_ransac_iters
                # )
                aligned_map_temp = align_local_map_to_xy_plane_prior(local_map_for_processing_W_original)
                if aligned_map_temp.has_points(): # 确保校正后仍有点
                    local_map_for_processing_W_aligned = aligned_map_temp
                    # alignment_transform_applied = T_align_actual
                    # ground_plane_model_in_W = plane_model_temp # 这是在原始W系下估计的地面
                    # rospy.logdebug(f"    局部地图已校正。应用变换:\n{alignment_transform_applied.round(3)}")
                else:
                    rospy.logwarn("    地面校正后点云为空，将使用原始未校正局部地图。")
            # ==============================

            # 后续的分割、聚类、目标识别都在 local_map_for_processing_W_aligned 上进行
            # 注意：这个 aligned 地图的点坐标是相对于原始世界坐标系W的，但姿态被校正了

            # === 新增：构建和发布2.5D栅格地图 ===
            if self.enable_25d_grid_map and local_map_for_processing_W_aligned .has_points():
                rospy.logdebug("  正在构建2.5D栅格地图...")
                points_for_grid_map_np = np.asarray(local_map_for_processing_W_aligned.points)
                current_lidar_xy_for_grid_center = refined_T_W_Lk[:2, 3] # 使用当前LiDAR的XY作为栅格地图中心

                grid_map_msg, _, _ = create_25d_grid_map(
                    points_for_grid_map_np,
                    current_lidar_xy_for_grid_center,
                    self.grid_map_resolution,
                    self.grid_map_size_meters,
                    self.grid_map_min_z_ref,
                    self.grid_map_max_z_ref
                )
                if grid_map_msg:
                    self.grid_map_pub.publish(grid_map_msg)
                    rospy.logdebug(f"    2.5D栅格地图已发布 (大小: {grid_map_msg.info.width}x{grid_map_msg.info.height} @ {grid_map_msg.info.resolution:.2f}m res).")
            # ===================================

            processed_local_map_to_display_W = o3d.geometry.PointCloud() # 最终用于显示的组合点云
            if local_map_for_processing_W_aligned.has_points():
                ground_local_map_W, nonground_local_map_W = segment_ground_ransac(
                    local_map_for_processing_W_aligned, 
                    self.ground_seg_distance_threshold, 
                    self.ground_seg_ransac_n, 
                    self.ground_seg_num_iterations
                )
                rospy.logdebug(f"  局部地图分割 (W): 地面 {len(ground_local_map_W.points)}, 非地面 {len(nonground_local_map_W.points)}")
                
                nonground_clustered_local_map_W = cluster_nonground_points(
                    nonground_local_map_W, 
                    self.nonground_cluster_eps, 
                    self.nonground_cluster_min_points_dbscan,
                    self.nonground_cluster_min_points_for_vis
                )
                rospy.logdebug(f"  局部地图非地面聚类 (W): {len(nonground_clustered_local_map_W.points)} 点 (有效簇)")

                temp_ground_W_vis = o3d.geometry.PointCloud(ground_local_map_W)
                temp_ground_W_vis.paint_uniform_color([0.5, 0.5, 0.5]) # 灰色地面
                
                if temp_ground_W_vis.has_points(): processed_local_map_to_display_W += temp_ground_W_vis
                if nonground_clustered_local_map_W.has_points(): processed_local_map_to_display_W += nonground_clustered_local_map_W

            rospy.logdebug(f"  合并处理后局部地图 (W, for vis): {len(processed_local_map_to_display_W.points)} 点")
            # 发布处理后的局部地图
            processed_local_map_msg = o3d_pcd_to_ros_pcd2(processed_local_map_to_display_W, self.odom_frame_id, current_time_ros)
            self.processed_local_map_pub.publish(processed_local_map_msg)
        
            # 7. 对聚类后的非地面点进行目标识别 (如果启用)
            if self.enable_basic_object_recognition and nonground_clustered_local_map_W.has_points():
                marker_header = Header(stamp=current_time_ros, frame_id=self.odom_frame_id)
                detected_objects_markers = self.recognize_objects_from_clusters_basic(
                    nonground_clustered_local_map_W,
                    marker_header
                )
                if detected_objects_markers.markers: # 只有当识别到物体时才发布
                    self.object_markers_pub.publish(detected_objects_markers)
                    rospy.logdebug(f"  发布了 {len(detected_objects_markers.markers)} 个检测到的物体标记。")
            elif self.enable_basic_object_recognition: # 如果启用了但没有点云，可以发一个空的marker array来清除旧的
                empty_marker_array = MarkerArray()
                # 可以添加一个特殊的marker来指示清除 (或者依赖lifetime)
                # clear_marker = Marker()
                # clear_marker.header.stamp = current_time_ros
                # clear_marker.header.frame_id = self.odom_frame_id
                # clear_marker.ns = "detected_objects"
                # clear_marker.id = 0 # Or some other convention
                # clear_marker.action = Marker.DELETEALL
                # empty_marker_array.markers.append(clear_marker)
                self.object_markers_pub.publish(empty_marker_array)

            # ----------- 发布里程计、TF和路径 (使用 refined_T_W_Lk) -----------
            # 注意：这里发布的里程计和TF仍然是LiDAR在原始世界坐标系W中的位姿，
            # 而不是相对于校正后局部地图的位姿。局部地图的校正主要是为了处理和感知。
            trans_W_Lk = refined_T_W_Lk[:3, 3]
            quat_W_Lk_xyzw = tft.quaternion_from_matrix(refined_T_W_Lk) 

            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time_ros
            tf_msg.header.frame_id = self.odom_frame_id      
            tf_msg.child_frame_id = self.base_link_frame_id 
            tf_msg.transform.translation.x = trans_W_Lk[0]
            tf_msg.transform.translation.y = trans_W_Lk[1]
            tf_msg.transform.translation.z = trans_W_Lk[2]
            tf_msg.transform.rotation.x = quat_W_Lk_xyzw[0]
            tf_msg.transform.rotation.y = quat_W_Lk_xyzw[1]
            tf_msg.transform.rotation.z = quat_W_Lk_xyzw[2]
            tf_msg.transform.rotation.w = quat_W_Lk_xyzw[3]
            self.tf_broadcaster.sendTransform(tf_msg)

            odom_msg = Odometry()
            odom_msg.header.stamp = current_time_ros
            odom_msg.header.frame_id = self.odom_frame_id      
            odom_msg.child_frame_id = self.base_link_frame_id 
            odom_msg.pose.pose.position = Point(*trans_W_Lk)
            odom_msg.pose.pose.orientation = Quaternion(*quat_W_Lk_xyzw)
            self.odom_pub.publish(odom_msg)

            current_pose_stamped = PoseStamped()
            current_pose_stamped.header.stamp = current_time_ros
            current_pose_stamped.header.frame_id = self.odom_frame_id 
            current_pose_stamped.pose.position = Point(*trans_W_Lk)
            current_pose_stamped.pose.orientation = Quaternion(*quat_W_Lk_xyzw)
            self.path_msg.poses.append(current_pose_stamped)
            self.path_msg.header.stamp = current_time_ros 
            self.path_pub.publish(self.path_msg)
            self.frame_count += 1
            rate.sleep()


if __name__ == '__main__':
    # 移除性能分析部分，使其作为标准的ROS节点运行
    try:
        node = LidarImuOdometryNode()
        if hasattr(node, 'ply_files') and node.ply_files: # 确保节点初始化成功且有文件可处理
             node.run()
        else:
             rospy.logwarn("节点初始化可能未完全成功或无文件处理，请检查日志。节点将等待关闭。")
             rospy.spin() # 如果run没有执行，则spin以保持节点存活
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点因 Ctrl+C 或类似中断而关闭。")
    except Exception as e:
        rospy.logfatal(f"LidarImuOdometryNode 发生未捕获的严重异常: {e}", exc_info=True)
    finally:
        rospy.loginfo("LidarImuOdometryNode 正在关闭。")