#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensors_collect_v3.py — 多传感器可视化与采集系统（进程3）

╔══════════════════════════════════════════════════════════════════════╗
║  本文件在三进程架构中的位置:                                         ║
║  进程1: lslidar_ls_driver_node (C++)  ← UDP收包、点云解析            ║
║      ↓ /lslidar_point_cloud                                         ║
║  进程2: lslidar_ls_pipeline_node (C++) ← 算法处理管线                ║
║      ↓ /processed_point_cloud                                       ║
║  【进程3: 本文件】← 可视化GUI + 传感器采集 + 数据保存               ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import datetime
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String, Float32
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from PyQt5 import QtCore
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from MainWindow import Ui_MainWindow
import pyqtgraph.opengl as gl
import cv2
import csv
import queue
import sys
import os
import threading
import time
import logging
from IMUReader import IMUReader
from GPSReader3 import GPSReader

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

# ============================================================================
# PyQt信号定义 — 用于跨线程安全传递数据
#
# Qt规则: 不同线程不能直接操作GUI控件，必须通过信号(signal)传递
# 后台线程收到ROS消息 → emit信号 → 主线程槽函数更新界面
# ============================================================================
class GuiSignals(QtCore.QObject):
    sig_imu_updated          = QtCore.pyqtSignal(dict)      # IMU数据
    sig_gps_updated          = QtCore.pyqtSignal(dict)      # GPS数据
    sig_point_cloud_updated  = QtCore.pyqtSignal(object)    # 点云 Nx3
    sig_point_cloud_colors   = QtCore.pyqtSignal(object)    # 颜色 Nx3
    sig_obstacle_markers     = QtCore.pyqtSignal(object)    # 障碍物Marker
    sig_landing_zone         = QtCore.pyqtSignal(object)    # 着陆区Marker
    sig_camera_updated       = QtCore.pyqtSignal(object)    # 相机帧
    sig_pipeline_diagnostics = QtCore.pyqtSignal(str)       # 诊断信息
    sig_fps_updated          = QtCore.pyqtSignal(float)     # 帧率

# ============================================================================
# 相机控制器 — 独立线程采集图像
#
# 线程安全: 用 Lock 保护 self.frame
# ============================================================================
class CameraController:
    def __init__(self, index=0, width=1920, height=1080, fps=20):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise IOError(f"无法打开相机 {index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.fps = fps
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()

    def _loop(self):
        while self.running and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame
            time.sleep(max(0, 1.0 / self.fps))

    def get_latest_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        if self.cap.isOpened():
            self.cap.release()

# ============================================================================
# 数据保存器 — 独立线程异步写盘
#
# 数据流:
#   调用方: add(type, timestamp, data) → 队列
#   worker线程: 队列 → _save_ply / _save_csv / _save_image
#
# 为什么用队列?
#   磁盘I/O是阻塞操作，直接在ROS回调线程写盘会阻塞数据到来
# ============================================================================
class DataSaver:
    _PLY_HEADER = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex {count}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    )

    def __init__(self, base_path, queue_size=500):
        self.base_path = base_path
        self.session_path = None
        self.queue = queue.Queue(maxsize=queue_size)
        self.running = False
        self.save_count = {'pointcloud': 0, 'sensors': 0, 'image': 0}
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

    def _ensure_session(self):
        if self.session_path is None:
            folder = os.path.join(self.base_path, time.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(folder, exist_ok=True)
            os.makedirs(os.path.join(folder, "pointcloud"), exist_ok=True)
            os.makedirs(os.path.join(folder, "sensors"), exist_ok=True)
            os.makedirs(os.path.join(folder, "image"), exist_ok=True)
            self.session_path = folder

    def add(self, data_type, timestamp, data):
        if not self.running:
            return
        try:
            self.queue.put_nowait((data_type, timestamp, data))
        except queue.Full:
            pass

    def _worker(self):
        while True:
            try:
                data_type, ts, data = self.queue.get(timeout=1)
            except queue.Empty:
                continue
            try:
                if data_type == 'pointcloud' and isinstance(data, np.ndarray):
                    self._save_ply(ts, data)
                elif data_type == 'sensors':
                    self._save_csv(ts, data)
                elif data_type == 'image':
                    self._save_image(ts, data)
            except Exception as e:
                logging.error(f"保存失败 [{data_type}]: {e}")

    def _save_ply(self, ts, points):
        self._ensure_session()
        path = os.path.join(self.session_path, "pointcloud", f"{ts}.ply")
        header = self._PLY_HEADER.format(count=len(points))
        points_flat = np.asarray(points[:, :3], dtype=np.float32).tobytes()
        with open(path, 'wb') as f:
            f.write(header.encode())
            f.write(points_flat)
        self.save_count['pointcloud'] += 1

    def _save_csv(self, ts, data):
        self._ensure_session()
        path = os.path.join(self.session_path, "sensors", f"{ts}.csv")
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            if isinstance(data, dict):
                w.writerow(data.keys())
                w.writerow(data.values())
        self.save_count['sensors'] += 1

    def _save_image(self, ts, img):
        self._ensure_session()
        path = os.path.join(self.session_path, "image", f"{ts}.jpg")
        cv2.imwrite(path, img)
        self.save_count['image'] += 1

    def start(self):
        self.running = True
        self.session_path = None

    def stop(self):
        self.running = False

# ============================================================================
# ROS传感器节点 — 订阅话题 + 轮询传感器
#
# 职责: 作为ROS和PyQt5之间的桥梁
#   - 订阅C++管线发布的ROS话题
#   - 轮询IMU/GPS/相机硬件
#   - 通过PyQt信号将数据传递给GUI主线程
#
# 数据流:
#   ROS话题回调 → 信号emit → GUI槽函数更新界面
#   传感器轮询  → 信号emit → GUI槽函数更新界面
# ============================================================================
class SensorRosNode:
    def __init__(self, signals, data_saver, camera_ctl, imu_reader, gps_reader):
        self.signals = signals
        self.data_saver = data_saver
        self.camera_ctl = camera_ctl        # 可能为None（相机初始化失败时）
        self.imu_reader = imu_reader
        self.gps_reader = gps_reader
        self.current_points = None
        self.running = False
        self.thread = None
        self._subs = []                     # 所有订阅者列表（统一管理生命周期）

    # ──────────────────────────────────────────────────────────────────
    # ROS话题回调
    # ──────────────────────────────────────────────────────────────────

    def _pointcloud_callback(self, msg):
        """
        订阅 /processed_point_cloud 回调
        
        C++管线发布着色点云后，此函数被ROS自动调用。
        intensity字段编码了颜色:
          intensity=100   → 地面点（灰色）
          intensity=200   → 未知障碍物（黄色）
          intensity=230   → 车辆（红色）
          intensity=260   → 行人（绿色）
          intensity=290   → 建筑（蓝色）
          intensity=320   → 电线杆（橙色）
        """
        try:
            # 提取xyz坐标
            points = np.array(
                list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=np.float32)
            if len(points) < 10:
                return

            # 提取intensity（着色依据）
            intensity = None
            if 'intensity' in [f.name for f in msg.fields]:
                intensity = np.array(
                    [p[0] for p in pc2.read_points(msg, field_names=("intensity",), skip_nans=True)],
                    dtype=np.float32)

            # 下采样（GUI不需要超过2万个点）
            step = max(1, len(points) // 20000)
            display_pts = points[::step]
            self.current_points = display_pts

            # 发送点云到GUI
            self.signals.sig_point_cloud_updated.emit(display_pts)

            # 根据intensity生成RGB颜色
            if intensity is not None and len(intensity) == len(points):
                color_vals = intensity[::step]
                colors = np.zeros((len(display_pts), 3), dtype=np.float32)
                for i, val in enumerate(color_vals):
                    if val < 150:       colors[i] = [0.5, 0.5, 0.5]   # 地面=灰
                    elif val < 215:     colors[i] = [1.0, 1.0, 0.0]   # 未知=黄
                    elif val < 245:     colors[i] = [1.0, 0.2, 0.2]   # 车辆=红
                    elif val < 275:     colors[i] = [0.2, 1.0, 0.2]   # 行人=绿
                    elif val < 305:     colors[i] = [0.5, 0.5, 1.0]   # 建筑=蓝
                    else:               colors[i] = [1.0, 0.6, 0.0]   # 电线杆=橙
                self.signals.sig_point_cloud_colors.emit(colors)

            # 记录模式：保存点云到文件
            if self.data_saver.running:
                ts = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + \
                     f"{datetime.datetime.now().microsecond // 1000:03d}"
                self.data_saver.add('pointcloud', ts, points)

        except Exception as e:
            rospy.logerr_throttle(5, f"点云回调错误: {e}")

    def _obstacle_callback(self, msg):
        """订阅 /obstacle_markers 回调 — 转发障碍物边界盒到GUI"""
        self.signals.sig_obstacle_markers.emit(msg)

    def _landing_zone_callback(self, msg):
        """订阅 /landing_zone_marker 回调 — 转发着陆区到GUI"""
        self.signals.sig_landing_zone.emit(msg)

    def _diagnostics_callback(self, msg):
        """订阅 /pipeline_diagnostics 回调 — 转发诊断信息到GUI"""
        self.signals.sig_pipeline_diagnostics.emit(msg.data)

    def _fps_callback(self, msg):
        """订阅 /pipeline_fps 回调 — 转发帧率到GUI"""
        self.signals.sig_fps_updated.emit(msg.data)

    # ──────────────────────────────────────────────────────────────────
    # 传感器轮询线程
    # ──────────────────────────────────────────────────────────────────

    def _poll_sensors(self):
        """在独立线程中以30Hz轮询IMU/GPS/相机，通过信号发送到GUI"""
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            # IMU
            try:
                imu_data = self.imu_reader.get_latest_data()
                if imu_data:
                    self.signals.sig_imu_updated.emit(imu_data)
                    if self.data_saver.running:
                        self.data_saver.add('sensors', time.strftime("%H%M%S"),
                                            {'type': 'imu', **imu_data})
            except Exception:
                pass

            # GPS
            try:
                gps_data = self.gps_reader.get_latest_data()
                if gps_data:
                    self.signals.sig_gps_updated.emit(gps_data)
                    if self.data_saver.running:
                        self.data_saver.add('sensors', time.strftime("%H%M%S"),
                                            {'type': 'gps', **gps_data})
            except Exception:
                pass

            # 相机（camera_ctl可能为None）
            if self.camera_ctl is not None:
                try:
                    frame = self.camera_ctl.get_latest_frame()
                    if frame is not None:
                        self.signals.sig_camera_updated.emit(frame)
                        if self.data_saver.running:
                            self.data_saver.add('image', time.strftime("%H%M%S"), frame)
                except Exception:
                    pass

            rate.sleep()

    # ──────────────────────────────────────────────────────────────────
    # 生命周期管理
    # ──────────────────────────────────────────────────────────────────

    def start(self):
        """启动所有订阅和轮询线程"""
        self.running = True

        # 订阅C++管线发布的话题
        self._subs = [
            rospy.Subscriber("/processed_point_cloud", PointCloud2,
                             self._pointcloud_callback, queue_size=5),
            rospy.Subscriber("/obstacle_markers", MarkerArray,
                             self._obstacle_callback, queue_size=5),
            rospy.Subscriber("/landing_zone_marker", Marker,
                             self._landing_zone_callback, queue_size=5),
            rospy.Subscriber("/pipeline_diagnostics", String,
                             self._diagnostics_callback, queue_size=10),
            rospy.Subscriber("/pipeline_fps", Float32,
                             self._fps_callback, queue_size=10),
        ]

        # 启动传感器轮询线程
        self.thread = threading.Thread(target=self._poll_sensors, daemon=True)
        self.thread.start()

        rospy.loginfo("[GUI] 已订阅: /processed_point_cloud /obstacle_markers "
                      "/landing_zone_marker /pipeline_diagnostics /pipeline_fps")

    def stop(self):
        """停止轮询并取消所有订阅"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        for sub in self._subs:
            sub.unregister()
        self._subs.clear()

# ============================================================================
# 主窗口 — PyQt5 GUI
#
# 功能:
#   1. 3D点云可视化 (pyqtgraph OpenGL)
#   2. 障碍物边界盒 + 着陆区叠加显示
#   3. 相机画面显示
#   4. IMU/GPS状态显示
#   5. 管线诊断信息显示
#   6. 数据记录控制
#
# 刷新机制:
#   定时器(50ms=20fps) 从缓存取最新数据 → 更新3D视图
#   数据接收(后台线程) 和 GUI刷新(主线程) 解耦
# ============================================================================
class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, app, cfg, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.app = app
        self.cfg = cfg

        # ── 1. 创建信号对象 ──
        self.signals = GuiSignals()

        # ── 2. 初始化相机（失败不影响其他功能） ──
        self.camera_ctl = None
        try:
            self.camera_ctl = CameraController(
                index=cfg['camera_index'], width=cfg['camera_width'],
                height=cfg['camera_height'], fps=cfg['camera_fps'])
            self.camera_ctl.start()
            logging.info(f"相机已启动: index={cfg['camera_index']}")
        except Exception as e:
            logging.warning(f"相机初始化失败: {e}")

        # ── 3. 初始化IMU/GPS串口 ──
        self.imu_reader = IMUReader(port=cfg['imu_port'], baud_rate=cfg['imu_baud_rate'])
        self.gps_reader = GPSReader(port=cfg['gps_port'], baud_rate=cfg['gps_baud_rate'])
        self.imu_reader.start()
        self.gps_reader.start()

        # ── 4. 初始化数据保存器 ──
        self.data_saver = DataSaver(cfg['save_path'])

        # ── 5. 初始化ROS节点（订阅话题 + 轮询传感器） ──
        self.ros_node = SensorRosNode(
            self.signals, self.data_saver,
            self.camera_ctl, self.imu_reader, self.gps_reader)

        # ── 6. 初始化3D视图和UI ──
        self._init_3d_viewer()
        self._init_ui()

        # ── 7. 连接信号到槽函数 ──
        self._connect_signals()

        # ── 8. 启动定时器刷新3D视图 ──
        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh_3d)
        self.timer.start(50)

        # 3D视图动态元素
        self.obstacle_items = []
        self.landing_zone_item = None
        self._display_points = None
        self._display_colors = None

        self.append_log("系统已启动 - C++管线处理中，GUI专注可视化")

    # ──────────────────────────────────────────────────────────────────
    # 初始化
    # ──────────────────────────────────────────────────────────────────

    def _init_3d_viewer(self):
        """初始化pyqtgraph 3D点云查看器"""
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setBackgroundColor('k')
        self.gl_widget.setCameraPosition(distance=80, elevation=30, azimuth=45)
        self.gl_widget.show()

        axis = gl.GLAxisItem()
        axis.setSize(10, 10, 10)
        self.gl_widget.addItem(axis)

        grid = gl.GLGridItem()
        grid.scale(5, 5, 1)
        self.gl_widget.addItem(grid)

        self.scatter = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3)), size=2, color=(1, 1, 1, 0.5))
        self.gl_widget.addItem(self.scatter)

        layout = QVBoxLayout()
        layout.addWidget(self.gl_widget)
        self.pc_group.setLayout(layout)

    def _init_ui(self):
        """初始化UI控件"""
        self.setWindowTitle("LS3 LiDAR 多传感器采集系统 v3.0 (C++管线版)")
        self.btn_record.clicked.connect(self._toggle_recording)
        self.checkBox_obstacle_recog.setChecked(True)
        self.checkBox_obstacle_recog.setText("障碍物显示")
        self.checkBox_landing.setChecked(True)
        self.checkBox_landing.setText("着陆区显示")
        self.checkBox_process_cloud.hide()

    def _connect_signals(self):
        """连接所有PyQt信号到GUI槽函数"""
        self.signals.sig_point_cloud_updated.connect(self._on_pointcloud)
        self.signals.sig_point_cloud_colors.connect(self._on_colors)
        self.signals.sig_obstacle_markers.connect(self._on_obstacles)
        self.signals.sig_landing_zone.connect(self._on_landing_zone)
        self.signals.sig_camera_updated.connect(self._on_camera)
        self.signals.sig_imu_updated.connect(self._on_imu)
        self.signals.sig_gps_updated.connect(self._on_gps)
        self.signals.sig_pipeline_diagnostics.connect(self._on_diagnostics)
        self.signals.sig_fps_updated.connect(self._on_fps)

    # ──────────────────────────────────────────────────────────────────
    # 槽函数 — 后台线程通过信号emit后，这些函数在主线程执行
    # ──────────────────────────────────────────────────────────────────

    def _on_pointcloud(self, points):
        """缓存点云数据，等定时器刷新"""
        self._display_points = points

    def _on_colors(self, colors):
        """缓存颜色数据"""
        self._display_colors = colors

    def _refresh_3d(self):
        """定时器触发：刷新3D点云显示"""
        if self._display_points is not None:
            colors = self._display_colors
            if colors is not None and len(colors) != len(self._display_points):
                colors = None
            self.scatter.setData(pos=self._display_points, color=colors, size=2)

    def _on_obstacles(self, markers):
        """接收障碍物Marker，绘制3D边界盒"""
        # 清除旧的边界盒
        for item in self.obstacle_items:
            self.gl_widget.removeItem(item)
        self.obstacle_items.clear()

        if not self.checkBox_obstacle_recog.isChecked():
            return

        for marker in markers.markers:
            if marker.type == Marker.CUBE:
                p = marker.pose.position
                s = marker.scale
                c = (marker.color.r, marker.color.g, marker.color.b, marker.color.a)
                self._add_box(p.x, p.y, p.z, s.x, s.y, s.z, c)

    def _add_box(self, cx, cy, cz, sx, sy, sz, color):
        """用GLMeshItem绘制半透明立方体"""
        v = np.array([
            [cx-sx/2,cy-sy/2,cz-sz/2], [cx+sx/2,cy-sy/2,cz-sz/2],
            [cx+sx/2,cy+sy/2,cz-sz/2], [cx-sx/2,cy+sy/2,cz-sz/2],
            [cx-sx/2,cy-sy/2,cz+sz/2], [cx+sx/2,cy-sy/2,cz+sz/2],
            [cx+sx/2,cy+sy/2,cz+sz/2], [cx-sx/2,cy+sy/2,cz+sz/2]])
        f = np.array([[0,1,2],[0,2,3],[4,5,6],[4,6,7],
                      [0,1,5],[0,5,4],[2,3,7],[2,7,6],
                      [0,3,7],[0,7,4],[1,2,6],[1,6,5]])
        mesh = gl.GLMeshItem(vertexes=v, faces=f,
                             color=color, shader='shaded', glOptions='translucent')
        self.gl_widget.addItem(mesh)
        self.obstacle_items.append(mesh)

    def _on_landing_zone(self, marker):
        """接收着陆区Marker，绘制绿色圆盘"""
        if self.landing_zone_item:
            self.gl_widget.removeItem(self.landing_zone_item)
            self.landing_zone_item = None
        if not self.checkBox_landing.isChecked():
            return
        cx, cy, cz = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
        r = marker.scale.x / 2
        pts = np.zeros((200, 3))
        for i in range(200):
            a = 2 * np.pi * i / 200
            rr = r * np.sqrt(np.random.random())
            pts[i] = [cx + rr * np.cos(a), cy + rr * np.sin(a), cz]
        lz = gl.GLScatterPlotItem(pos=pts, color=(0, 1, 0, 0.3), size=8)
        self.gl_widget.addItem(lz)
        self.landing_zone_item = lz

    def _on_camera(self, frame):
        """显示相机画面"""
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        self.camera_label.setPixmap(
            QPixmap.fromImage(qimg).scaled(320, 240, Qt.KeepAspectRatio))

    def _on_imu(self, d):
        self.imu_labels_pose.setText(
            f"R:{d.get('roll',0):.1f} P:{d.get('pitch',0):.1f} Y:{d.get('yaw',0):.1f}")
        self.imu_labels_accel.setText(
            f"X:{d.get('ax',0):.2f} Y:{d.get('ay',0):.2f} Z:{d.get('az',0):.2f}")
        self.imu_labels_gyro.setText(
            f"X:{d.get('gx',0):.2f} Y:{d.get('gy',0):.2f} Z:{d.get('gz',0):.2f}")

    def _on_gps(self, d):
        self.gps_labels_sat.setText(str(d.get('satellites', '-')))
        self.gps_labels_lat.setText(f"{d.get('latitude', '-'):.6f}")
        self.gps_labels_lon.setText(f"{d.get('longitude', '-'):.6f}")
        self.gps_labels_alt.setText(f"{d.get('altitude', '-'):.1f}")

    def _on_diagnostics(self, text):
        self.result_text.appendPlainText(text)

    def _on_fps(self, fps):
        self.fps_label.setText(f"管线 FPS: {fps:.1f}")

    def append_log(self, msg):
        self.log_text.appendPlainText(str(msg))

    def _toggle_recording(self, checked):
        if checked:
            self.data_saver.start()
            self.btn_record.setText("■ 停止记录")
            self.append_log("开始记录数据...")
        else:
            self.data_saver.stop()
            self.btn_record.setText("● 开始记录")
            self.append_log(f"记录停止。本次保存: {self.data_saver.save_count}")

    def closeEvent(self, event):
        self.append_log("正在关闭系统...")
        self.timer.stop()
        self.ros_node.stop()
        if self.camera_ctl:
            self.camera_ctl.stop()
        self.imu_reader.stop()
        self.gps_reader.stop()
        event.accept()

# ============================================================================
# 主入口
#
# 启动顺序:
#   1. rospy.init_node（必须最先调用，之后才能 get_param）
#   2. 加载配置参数
#   3. 创建主窗口（内部初始化相机/IMU/GPS/ROS节点）
#   4. 启动ROS传感器节点
#   5. 进入PyQt5事件循环
# ============================================================================
def main():
    app = QApplication(sys.argv)

    # 必须先init_node，才能调用get_param
    rospy.init_node('sensors_collect_v3', anonymous=True)

    # 加载配置参数
    cfg = {
        'save_path':    rospy.get_param('~save_path',    '/home/nuaa/Ayra/data_lidargpsimu/'),
        'camera_index': rospy.get_param('~camera_index', 0),
        'camera_width': rospy.get_param('~camera_width', 1920),
        'camera_height':rospy.get_param('~camera_height',1080),
        'camera_fps':   rospy.get_param('~camera_fps',   20),
        'gps_port':     rospy.get_param('~gps_port',     '/dev/wheeltec_gps'),
        'gps_baud_rate':rospy.get_param('~gps_baud_rate',9600),
        'imu_port':     rospy.get_param('~imu_port',     '/dev/wheeltec_IMU'),
        'imu_baud_rate':rospy.get_param('~imu_baud_rate',460800),
    }

    window = MainWindow(app, cfg)
    window.show()
    window.ros_node.start()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
