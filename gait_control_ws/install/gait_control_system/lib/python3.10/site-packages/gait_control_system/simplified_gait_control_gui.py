#!/usr/bin/env python3
"""
简化步态控制系统图形化界面
基于PyQt5的ROS2步态控制测试界面
支持运动模式参数 + 五次多项式助力曲线预览
"""
import sys
import os
import time
import numpy as np

# PyQt5组件
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QTextEdit,
    QGroupBox, QDoubleSpinBox, QMessageBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot, QObject, QTimer

# matplotlib组件用于曲线预览
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
# 设置中文字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'SimHei', 'WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

from .motion_mode_defaults import get_default_motion_modes

# ROS2组件
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32
    ROS2_AVAILABLE = True
except ImportError:
    print("警告: ROS2未安装，运行在模拟模式")
    ROS2_AVAILABLE = False


class MockROS2:
    """ROS2模拟类，用于无ROS2环境的测试"""
    def __init__(self):
        self.current_mode = "walking"
        self.parameters = {}

    def publish_mode(self, mode):
        print(f"[模拟] 发布运动模式: {mode}")
        self.current_mode = mode

    def set_parameter(self, param_name, value):
        print(f"[模拟] 设置参数: {param_name} = {value}")
        self.parameters[param_name] = value
        return True

    def enable_motors(self):
        print("[模拟] 发送电机使能命令")
        return True


class ROS2Interface(QObject):
    """ROS2通信接口 - 简化版本，仅用于模式切换和参数设置"""
    mode_changed = pyqtSignal(str)
    parameter_updated = pyqtSignal(str, float)
    system_status_changed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        if ROS2_AVAILABLE:
            self.node = None
            self.mode_pub = None
            self.mechanical_zero_pub = None  # 机械标零发布器
            self.motor_enable_pub = None
            self.phase_bias_sub = None
            self.connected = False
            self.running = False
            self.ros_thread = None
        else:
            self.mock_ros = MockROS2()
            self.connected = True

    def start(self):
        if ROS2_AVAILABLE:
            self.running = True
            self.ros_thread = QThread()
            self.ros_thread.run = self._ros_thread
            self.ros_thread.start()
            self.connected = True
            print("ROS2接口启动")
        else:
            print("模拟模式启动")

    def stop(self):
        if ROS2_AVAILABLE:
            self.running = False
            if self.ros_thread:
                self.ros_thread.wait()
        self.connected = False
        print("ROS2接口已停止")

    def _ros_thread(self):
        if not ROS2_AVAILABLE:
            return
        try:
            rclpy.init()
            self.node = Node('simplified_gait_gui_interface')
            self.mode_pub = self.node.create_publisher(String, '/motion_mode/set', 10)
            self.mechanical_zero_pub = self.node.create_publisher(String, '/mechanical_zero/command', 10)
            self.motor_enable_pub = self.node.create_publisher(String, '/motor_enable/command', 10)
            self.phase_bias_sub = self.node.create_subscription(
                Float32, '/phase_bias/adjusted', self._on_phase_bias_msg, 10
            )
            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"ROS2线程错误: {e}")
        finally:
            if self.node:
                self.node.destroy_node()
            try:
                rclpy.shutdown()
            except:
                pass  # 忽略重复shutdown错误

    def publish_mode(self, mode):
        if ROS2_AVAILABLE and self.mode_pub:
            msg = String()
            msg.data = mode
            self.mode_pub.publish(msg)
            print(f"发布运动模式: {mode}")
        else:
            self.mock_ros.publish_mode(mode)

    def publish_mechanical_zero(self):
        """发布机械标零命令"""
        if ROS2_AVAILABLE and self.mechanical_zero_pub:
            msg = String()
            msg.data = "mechanical_zero"
            self.mechanical_zero_pub.publish(msg)
            print("发布机械标零命令")
            return True
        elif ROS2_AVAILABLE:
            print("机械标零发布器未就绪")
            return False
        else:
            print("[模拟] 发布机械标零命令")
            return True

    def publish_motor_enable(self):
        """发布电机使能命令"""
        if ROS2_AVAILABLE and self.motor_enable_pub:
            msg = String()
            msg.data = "enable"
            self.motor_enable_pub.publish(msg)
            print("发布电机使能命令")
            return True
        elif ROS2_AVAILABLE:
            print("电机使能发布器未就绪")
            return False
        else:
            return self.mock_ros.enable_motors()

    def _on_phase_bias_msg(self, msg):
        """接收来自控制节点的 phase_bias 自动调整值并转发到GUI。"""
        try:
            value = float(msg.data)
        except Exception:
            return
        self.parameter_updated.emit("walking.phase_bias", value)

    def set_parameter(self, param_name, value):
        if ROS2_AVAILABLE and self.node:
            try:
                # 使用 ROS2 参数客户端替代 subprocess，提高效率
                if not hasattr(self, '_param_client'):
                    from rclpy.parameter import Parameter
                    from rcl_interfaces.srv import SetParameters
                    self._param_client = self.node.create_client(SetParameters, '/gait_analysis_node/set_parameters')
                
                if self._param_client.service_is_ready():
                    from rclpy.parameter import Parameter
                    from rcl_interfaces.srv import SetParameters
                    request = SetParameters.Request()
                    param = Parameter(name=param_name, value=value)
                    request.parameters = [param.to_parameter_msg()]
                    
                    future = self._param_client.call_async(request)
                    # 非阻塞调用，避免GUI卡顿
                    print(f"参数设置请求已发送: {param_name} = {value}")
                    return True
                else:
                    # 如果服务不可用，回退到 subprocess 方式
                    import subprocess
                    result = subprocess.run([
                        'ros2', 'param', 'set', '/gait_analysis_node', param_name, str(value)
                    ], capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        print(f"参数设置成功: {param_name} = {value}")
                        return True
                    else:
                        print(f"参数设置失败: {result.stderr}")
                        return False
            except Exception as e:
                print(f"参数设置异常: {e}")
                return False
        else:
            return self.mock_ros.set_parameter(param_name, value)

    def is_connected(self):
        return self.connected


class ParameterControlWidget(QWidget):
    """参数控制组件"""
    parameter_changed = pyqtSignal(str, float)
    mode_changed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        # 运动模式参数定义（与节点共享默认配置）
        self.motion_modes = get_default_motion_modes()
        self.current_mode = "walking"
        self.init_ui()

    def init_ui(self):
        # 创建主水平布局（两列）
        main_layout = QHBoxLayout()
        
        # 第一列：运动模式和正弦函数参数
        left_column = QVBoxLayout()

        # 运动模式选择
        mode_group = QGroupBox("运动模式选择")
        mode_layout = QVBoxLayout()
        self.mode_combo = QComboBox()
        for mode_key, mode_info in self.motion_modes.items():
            self.mode_combo.addItem(mode_info["name"], mode_key)
        self.mode_combo.currentTextChanged.connect(self.on_mode_changed)
        self.mode_description = QLabel(self.motion_modes["walking"]["description"])
        self.mode_description.setWordWrap(True)
        mode_layout.addWidget(QLabel("选择运动模式:"))
        mode_layout.addWidget(self.mode_combo)
        mode_layout.addWidget(QLabel("模式描述:"))
        mode_layout.addWidget(self.mode_description)
        mode_group.setLayout(mode_layout)

        # 伸展阶段五次多项式助力参数
        extension_group = QGroupBox("伸展阶段五次多项式助力参数")
        extension_layout = QGridLayout()
        
        # 伸展起始时刻 ext_t0
        extension_layout.addWidget(QLabel("伸展起始时刻 ext_t0 (0-1):"), 0, 0)
        ext_t0_layout = QHBoxLayout()
        ext_t0_dec_btn = QPushButton("-")
        ext_t0_dec_btn.setFixedSize(25, 25)
        ext_t0_dec_btn.clicked.connect(lambda: self.ext_t0_spin.setValue(max(self.ext_t0_spin.minimum(), self.ext_t0_spin.value() - self.ext_t0_spin.singleStep())))
        self.ext_t0_spin = QDoubleSpinBox()
        self.ext_t0_spin.setRange(0.0, 1.0)
        self.ext_t0_spin.setSingleStep(0.01)
        self.ext_t0_spin.setDecimals(3)
        self.ext_t0_spin.setValue(0.0)  # 默认步行模式值
        self.ext_t0_spin.setToolTip("伸展助力起始时刻，归一化相位0-1")
        self.ext_t0_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.ext_t0", v))
        ext_t0_inc_btn = QPushButton("+")
        ext_t0_inc_btn.setFixedSize(25, 25)
        ext_t0_inc_btn.clicked.connect(lambda: self.ext_t0_spin.setValue(min(self.ext_t0_spin.maximum(), self.ext_t0_spin.value() + self.ext_t0_spin.singleStep())))
        ext_t0_layout.addWidget(ext_t0_dec_btn)
        ext_t0_layout.addWidget(self.ext_t0_spin)
        ext_t0_layout.addWidget(ext_t0_inc_btn)
        ext_t0_widget = QWidget()
        ext_t0_widget.setLayout(ext_t0_layout)
        extension_layout.addWidget(ext_t0_widget, 0, 1)
        
        # 伸展结束时刻 ext_tf
        extension_layout.addWidget(QLabel("伸展结束时刻 ext_tf (0-1):"), 1, 0)
        ext_tf_layout = QHBoxLayout()
        ext_tf_dec_btn = QPushButton("-")
        ext_tf_dec_btn.setFixedSize(25, 25)
        ext_tf_dec_btn.clicked.connect(lambda: self.ext_tf_spin.setValue(max(self.ext_tf_spin.minimum(), self.ext_tf_spin.value() - self.ext_tf_spin.singleStep())))
        self.ext_tf_spin = QDoubleSpinBox()
        self.ext_tf_spin.setRange(0.0, 1.0)
        self.ext_tf_spin.setSingleStep(0.01)
        self.ext_tf_spin.setDecimals(3)
        self.ext_tf_spin.setValue(0.55)  # 默认步行模式值
        self.ext_tf_spin.setToolTip("伸展助力结束时刻，归一化相位0-1")
        self.ext_tf_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.ext_tf", v))
        ext_tf_inc_btn = QPushButton("+")
        ext_tf_inc_btn.setFixedSize(25, 25)
        ext_tf_inc_btn.clicked.connect(lambda: self.ext_tf_spin.setValue(min(self.ext_tf_spin.maximum(), self.ext_tf_spin.value() + self.ext_tf_spin.singleStep())))
        ext_tf_layout.addWidget(ext_tf_dec_btn)
        ext_tf_layout.addWidget(self.ext_tf_spin)
        ext_tf_layout.addWidget(ext_tf_inc_btn)
        ext_tf_widget = QWidget()
        ext_tf_widget.setLayout(ext_tf_layout)
        extension_layout.addWidget(ext_tf_widget, 1, 1)
        
        # 伸展峰值位置 ext_p
        extension_layout.addWidget(QLabel("伸展峰值位置 ext_p (0-1):"), 2, 0)
        ext_p_layout = QHBoxLayout()
        ext_p_dec_btn = QPushButton("-")
        ext_p_dec_btn.setFixedSize(25, 25)
        ext_p_dec_btn.clicked.connect(lambda: self.ext_p_spin.setValue(max(self.ext_p_spin.minimum(), self.ext_p_spin.value() - self.ext_p_spin.singleStep())))
        self.ext_p_spin = QDoubleSpinBox()
        self.ext_p_spin.setRange(0.0, 1.0)
        self.ext_p_spin.setSingleStep(0.01)
        self.ext_p_spin.setDecimals(3)
        self.ext_p_spin.setValue(0.40)  # 默认步行模式值
        self.ext_p_spin.setToolTip("伸展助力峰值位置，相对于t0-tf区间的位置0-1")
        self.ext_p_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.ext_p", v))
        ext_p_inc_btn = QPushButton("+")
        ext_p_inc_btn.setFixedSize(25, 25)
        ext_p_inc_btn.clicked.connect(lambda: self.ext_p_spin.setValue(min(self.ext_p_spin.maximum(), self.ext_p_spin.value() + self.ext_p_spin.singleStep())))
        ext_p_layout.addWidget(ext_p_dec_btn)
        ext_p_layout.addWidget(self.ext_p_spin)
        ext_p_layout.addWidget(ext_p_inc_btn)
        ext_p_widget = QWidget()
        ext_p_widget.setLayout(ext_p_layout)
        extension_layout.addWidget(ext_p_widget, 2, 1)
        
        # 伸展最大力矩 ext_Tmax
        extension_layout.addWidget(QLabel("伸展最大力矩 ext_Tmax (Nm):"), 3, 0)
        ext_Tmax_layout = QHBoxLayout()
        ext_Tmax_dec_btn = QPushButton("-")
        ext_Tmax_dec_btn.setFixedSize(25, 25)
        ext_Tmax_dec_btn.clicked.connect(lambda: self.ext_Tmax_spin.setValue(max(self.ext_Tmax_spin.minimum(), self.ext_Tmax_spin.value() - self.ext_Tmax_spin.singleStep())))
        self.ext_Tmax_spin = QDoubleSpinBox()
        self.ext_Tmax_spin.setRange(0.0, 18.0)
        self.ext_Tmax_spin.setSingleStep(0.1)
        self.ext_Tmax_spin.setValue(3.0)  # 默认步行模式值
        self.ext_Tmax_spin.setToolTip("伸展助力最大力矩，单位Nm")
        self.ext_Tmax_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.ext_Tmax", v))
        ext_Tmax_inc_btn = QPushButton("+")
        ext_Tmax_inc_btn.setFixedSize(25, 25)
        ext_Tmax_inc_btn.clicked.connect(lambda: self.ext_Tmax_spin.setValue(min(self.ext_Tmax_spin.maximum(), self.ext_Tmax_spin.value() + self.ext_Tmax_spin.singleStep())))
        ext_Tmax_layout.addWidget(ext_Tmax_dec_btn)
        ext_Tmax_layout.addWidget(self.ext_Tmax_spin)
        ext_Tmax_layout.addWidget(ext_Tmax_inc_btn)
        ext_Tmax_widget = QWidget()
        ext_Tmax_widget.setLayout(ext_Tmax_layout)
        extension_layout.addWidget(ext_Tmax_widget, 3, 1)
        
        extension_group.setLayout(extension_layout)
        
        # 屈曲阶段五次多项式助力参数
        flexion_group = QGroupBox("屈曲阶段五次多项式助力参数")
        flexion_layout = QGridLayout()
        
        # 屈曲起始时刻 flex_t0
        flexion_layout.addWidget(QLabel("屈曲起始时刻 flex_t0 (0-1):"), 0, 0)
        flex_t0_layout = QHBoxLayout()
        flex_t0_dec_btn = QPushButton("-")
        flex_t0_dec_btn.setFixedSize(25, 25)
        flex_t0_dec_btn.clicked.connect(lambda: self.flex_t0_spin.setValue(max(self.flex_t0_spin.minimum(), self.flex_t0_spin.value() - self.flex_t0_spin.singleStep())))
        self.flex_t0_spin = QDoubleSpinBox()
        self.flex_t0_spin.setRange(0.0, 1.0)
        self.flex_t0_spin.setSingleStep(0.01)
        self.flex_t0_spin.setDecimals(3)
        self.flex_t0_spin.setValue(0.60)  # 默认步行模式值
        self.flex_t0_spin.setToolTip("屈曲助力起始时刻，归一化相位0-1")
        self.flex_t0_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.flex_t0", v))
        flex_t0_inc_btn = QPushButton("+")
        flex_t0_inc_btn.setFixedSize(25, 25)
        flex_t0_inc_btn.clicked.connect(lambda: self.flex_t0_spin.setValue(min(self.flex_t0_spin.maximum(), self.flex_t0_spin.value() + self.flex_t0_spin.singleStep())))
        flex_t0_layout.addWidget(flex_t0_dec_btn)
        flex_t0_layout.addWidget(self.flex_t0_spin)
        flex_t0_layout.addWidget(flex_t0_inc_btn)
        flex_t0_widget = QWidget()
        flex_t0_widget.setLayout(flex_t0_layout)
        flexion_layout.addWidget(flex_t0_widget, 0, 1)
        
        # 屈曲结束时刻 flex_tf
        flexion_layout.addWidget(QLabel("屈曲结束时刻 flex_tf (0-1):"), 1, 0)
        flex_tf_layout = QHBoxLayout()
        flex_tf_dec_btn = QPushButton("-")
        flex_tf_dec_btn.setFixedSize(25, 25)
        flex_tf_dec_btn.clicked.connect(lambda: self.flex_tf_spin.setValue(max(self.flex_tf_spin.minimum(), self.flex_tf_spin.value() - self.flex_tf_spin.singleStep())))
        self.flex_tf_spin = QDoubleSpinBox()
        self.flex_tf_spin.setRange(0.0, 1.0)
        self.flex_tf_spin.setSingleStep(0.01)
        self.flex_tf_spin.setDecimals(3)
        self.flex_tf_spin.setValue(0.90)  # 默认步行模式值
        self.flex_tf_spin.setToolTip("屈曲助力结束时刻，归一化相位0-1")
        self.flex_tf_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.flex_tf", v))
        flex_tf_inc_btn = QPushButton("+")
        flex_tf_inc_btn.setFixedSize(25, 25)
        flex_tf_inc_btn.clicked.connect(lambda: self.flex_tf_spin.setValue(min(self.flex_tf_spin.maximum(), self.flex_tf_spin.value() + self.flex_tf_spin.singleStep())))
        flex_tf_layout.addWidget(flex_tf_dec_btn)
        flex_tf_layout.addWidget(self.flex_tf_spin)
        flex_tf_layout.addWidget(flex_tf_inc_btn)
        flex_tf_widget = QWidget()
        flex_tf_widget.setLayout(flex_tf_layout)
        flexion_layout.addWidget(flex_tf_widget, 1, 1)
        
        # 屈曲峰值位置 flex_p
        flexion_layout.addWidget(QLabel("屈曲峰值位置 flex_p (0-1):"), 2, 0)
        flex_p_layout = QHBoxLayout()
        flex_p_dec_btn = QPushButton("-")
        flex_p_dec_btn.setFixedSize(25, 25)
        flex_p_dec_btn.clicked.connect(lambda: self.flex_p_spin.setValue(max(self.flex_p_spin.minimum(), self.flex_p_spin.value() - self.flex_p_spin.singleStep())))
        self.flex_p_spin = QDoubleSpinBox()
        self.flex_p_spin.setRange(0.0, 1.0)
        self.flex_p_spin.setSingleStep(0.01)
        self.flex_p_spin.setDecimals(3)
        self.flex_p_spin.setValue(0.35)  # 默认步行模式值
        self.flex_p_spin.setToolTip("屈曲助力峰值位置，相对于t0-tf区间的位置0-1")
        self.flex_p_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.flex_p", v))
        flex_p_inc_btn = QPushButton("+")
        flex_p_inc_btn.setFixedSize(25, 25)
        flex_p_inc_btn.clicked.connect(lambda: self.flex_p_spin.setValue(min(self.flex_p_spin.maximum(), self.flex_p_spin.value() + self.flex_p_spin.singleStep())))
        flex_p_layout.addWidget(flex_p_dec_btn)
        flex_p_layout.addWidget(self.flex_p_spin)
        flex_p_layout.addWidget(flex_p_inc_btn)
        flex_p_widget = QWidget()
        flex_p_widget.setLayout(flex_p_layout)
        flexion_layout.addWidget(flex_p_widget, 2, 1)
        
        # 屈曲最大力矩 flex_Tmax
        flexion_layout.addWidget(QLabel("屈曲最大力矩 flex_Tmax (Nm):"), 3, 0)
        flex_Tmax_layout = QHBoxLayout()
        flex_Tmax_dec_btn = QPushButton("-")
        flex_Tmax_dec_btn.setFixedSize(25, 25)
        flex_Tmax_dec_btn.clicked.connect(lambda: self.flex_Tmax_spin.setValue(max(self.flex_Tmax_spin.minimum(), self.flex_Tmax_spin.value() - self.flex_Tmax_spin.singleStep())))
        self.flex_Tmax_spin = QDoubleSpinBox()
        self.flex_Tmax_spin.setRange(0.0, 18.0)
        self.flex_Tmax_spin.setSingleStep(0.1)
        self.flex_Tmax_spin.setValue(2.5)  # 默认步行模式值
        self.flex_Tmax_spin.setToolTip("屈曲助力最大力矩，单位Nm")
        self.flex_Tmax_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.flex_Tmax", v))
        flex_Tmax_inc_btn = QPushButton("+")
        flex_Tmax_inc_btn.setFixedSize(25, 25)
        flex_Tmax_inc_btn.clicked.connect(lambda: self.flex_Tmax_spin.setValue(min(self.flex_Tmax_spin.maximum(), self.flex_Tmax_spin.value() + self.flex_Tmax_spin.singleStep())))
        flex_Tmax_layout.addWidget(flex_Tmax_dec_btn)
        flex_Tmax_layout.addWidget(self.flex_Tmax_spin)
        flex_Tmax_layout.addWidget(flex_Tmax_inc_btn)
        flex_Tmax_widget = QWidget()
        flex_Tmax_widget.setLayout(flex_Tmax_layout)
        flexion_layout.addWidget(flex_Tmax_widget, 3, 1)
        
        flexion_group.setLayout(flexion_layout)
        
        # 偏置相位参数
        phase_bias_group = QGroupBox("偏置相位参数")
        phase_bias_layout = QGridLayout()
        
        # 偏置相位 phase_bias
        phase_bias_layout.addWidget(QLabel("偏置相位 phase_bias (-1到1):"), 0, 0)
        phase_bias_layout_h = QHBoxLayout()
        phase_bias_dec_btn = QPushButton("-")
        phase_bias_dec_btn.setFixedSize(25, 25)
        phase_bias_dec_btn.clicked.connect(lambda: self.phase_bias_spin.setValue(max(self.phase_bias_spin.minimum(), self.phase_bias_spin.value() - self.phase_bias_spin.singleStep())))
        self.phase_bias_spin = QDoubleSpinBox()
        self.phase_bias_spin.setRange(-1.0, 1.0)
        self.phase_bias_spin.setSingleStep(0.01)
        self.phase_bias_spin.setDecimals(3)
        self.phase_bias_spin.setValue(0.15)  # 默认步行模式值
        self.phase_bias_spin.setToolTip("偏置相位，用于整体调整助力曲线的相位位置，范围-1到1；正值助力提前，负值助力迟后")
        self.phase_bias_spin.valueChanged.connect(
            lambda v: self.on_param_changed_and_update_plot(f"{self.current_mode}.phase_bias", v))
        phase_bias_inc_btn = QPushButton("+")
        phase_bias_inc_btn.setFixedSize(25, 25)
        phase_bias_inc_btn.clicked.connect(lambda: self.phase_bias_spin.setValue(min(self.phase_bias_spin.maximum(), self.phase_bias_spin.value() + self.phase_bias_spin.singleStep())))
        phase_bias_layout_h.addWidget(phase_bias_dec_btn)
        phase_bias_layout_h.addWidget(self.phase_bias_spin)
        phase_bias_layout_h.addWidget(phase_bias_inc_btn)
        phase_bias_widget = QWidget()
        phase_bias_widget.setLayout(phase_bias_layout_h)
        phase_bias_layout.addWidget(phase_bias_widget, 0, 1)

        # 0.6Hz 对应相位偏置
        phase_bias_layout.addWidget(QLabel("0.6Hz 相位偏置:"), 1, 0)
        phase_bias_0p6_layout = QHBoxLayout()
        phase_bias_0p6_dec_btn = QPushButton("-")
        phase_bias_0p6_dec_btn.setFixedSize(25, 25)
        phase_bias_0p6_dec_btn.clicked.connect(
            lambda: self.phase_bias_0p6_spin.setValue(
                max(self.phase_bias_0p6_spin.minimum(), self.phase_bias_0p6_spin.value() - self.phase_bias_0p6_spin.singleStep())
            )
        )
        self.phase_bias_0p6_spin = QDoubleSpinBox()
        self.phase_bias_0p6_spin.setRange(-1.0, 1.0)
        self.phase_bias_0p6_spin.setSingleStep(0.01)
        self.phase_bias_0p6_spin.setDecimals(3)
        self.phase_bias_0p6_spin.setToolTip("步频为0.6Hz时的相位偏置，用于walking/cycling插值")
        self.phase_bias_0p6_spin.valueChanged.connect(
            lambda v: self.parameter_changed.emit(f"{self.current_mode}.phase_bias_at_0p6", v)
        )
        phase_bias_0p6_inc_btn = QPushButton("+")
        phase_bias_0p6_inc_btn.setFixedSize(25, 25)
        phase_bias_0p6_inc_btn.clicked.connect(
            lambda: self.phase_bias_0p6_spin.setValue(
                min(self.phase_bias_0p6_spin.maximum(), self.phase_bias_0p6_spin.value() + self.phase_bias_0p6_spin.singleStep())
            )
        )
        phase_bias_0p6_layout.addWidget(phase_bias_0p6_dec_btn)
        phase_bias_0p6_layout.addWidget(self.phase_bias_0p6_spin)
        phase_bias_0p6_layout.addWidget(phase_bias_0p6_inc_btn)
        self.phase_bias_0p6_widget = QWidget()
        self.phase_bias_0p6_widget.setLayout(phase_bias_0p6_layout)
        phase_bias_layout.addWidget(self.phase_bias_0p6_widget, 1, 1)

        # 线性插值斜率
        phase_bias_layout.addWidget(QLabel("线性插值斜率:"), 2, 0)
        phase_bias_slope_layout = QHBoxLayout()
        phase_bias_slope_dec_btn = QPushButton("-")
        phase_bias_slope_dec_btn.setFixedSize(25, 25)
        phase_bias_slope_dec_btn.clicked.connect(
            lambda: self.phase_bias_slope_spin.setValue(
                max(self.phase_bias_slope_spin.minimum(), self.phase_bias_slope_spin.value() - self.phase_bias_slope_spin.singleStep())
            )
        )
        self.phase_bias_slope_spin = QDoubleSpinBox()
        self.phase_bias_slope_spin.setRange(-10.0, 10.0)
        self.phase_bias_slope_spin.setSingleStep(0.05)
        self.phase_bias_slope_spin.setDecimals(3)
        self.phase_bias_slope_spin.setToolTip("频率-相位偏置线性插值斜率 (偏置/Hz)")
        self.phase_bias_slope_spin.valueChanged.connect(
            lambda v: self.parameter_changed.emit(f"{self.current_mode}.phase_bias_slope", v)
        )
        phase_bias_slope_inc_btn = QPushButton("+")
        phase_bias_slope_inc_btn.setFixedSize(25, 25)
        phase_bias_slope_inc_btn.clicked.connect(
            lambda: self.phase_bias_slope_spin.setValue(
                min(self.phase_bias_slope_spin.maximum(), self.phase_bias_slope_spin.value() + self.phase_bias_slope_spin.singleStep())
            )
        )
        phase_bias_slope_layout.addWidget(phase_bias_slope_dec_btn)
        phase_bias_slope_layout.addWidget(self.phase_bias_slope_spin)
        phase_bias_slope_layout.addWidget(phase_bias_slope_inc_btn)
        self.phase_bias_slope_widget = QWidget()
        self.phase_bias_slope_widget.setLayout(phase_bias_slope_layout)
        phase_bias_layout.addWidget(self.phase_bias_slope_widget, 2, 1)
        
        phase_bias_group.setLayout(phase_bias_layout)

        # 步态检测参数
        gait_detection_group = QGroupBox("步态检测参数")
        gait_detection_layout = QGridLayout()
        # rT 阈值
        gait_detection_layout.addWidget(QLabel("步态检测阈值 rT:"), 0, 0)
        rT_layout = QHBoxLayout()
        rT_dec_btn = QPushButton("-")
        rT_dec_btn.setFixedSize(25, 25)
        rT_dec_btn.clicked.connect(lambda: self.rT_spin.setValue(max(self.rT_spin.minimum(), self.rT_spin.value() - self.rT_spin.singleStep())))
        self.rT_spin = QDoubleSpinBox()
        self.rT_spin.setRange(0.1, 2.0)
        self.rT_spin.setValue(0.3)
        self.rT_spin.setSingleStep(0.05)
        self.rT_spin.setDecimals(2)
        self.rT_spin.valueChanged.connect(
            lambda v: self.parameter_changed.emit("rT", v))
        rT_inc_btn = QPushButton("+")
        rT_inc_btn.setFixedSize(25, 25)
        rT_inc_btn.clicked.connect(lambda: self.rT_spin.setValue(min(self.rT_spin.maximum(), self.rT_spin.value() + self.rT_spin.singleStep())))
        rT_layout.addWidget(rT_dec_btn)
        rT_layout.addWidget(self.rT_spin)
        rT_layout.addWidget(rT_inc_btn)
        rT_widget = QWidget()
        rT_widget.setLayout(rT_layout)
        gait_detection_layout.addWidget(rT_widget, 0, 1)
        gait_detection_group.setLayout(gait_detection_layout)
        
        # 第一列：运动模式和伸展参数
        left_column.addWidget(mode_group)
        left_column.addWidget(extension_group)
        
        # 第二列：步态检测、屈曲参数和偏置相位
        middle_column = QVBoxLayout()
        middle_column.addWidget(gait_detection_group)
        middle_column.addWidget(flexion_group)
        middle_column.addWidget(phase_bias_group)
        
        # 第三列：助力曲线预览（单独一列）
        right_column = QVBoxLayout()
        self.create_assist_curve_preview()
        right_column.addWidget(self.curve_preview_group)
        
        # 将三列添加到主布局
        main_layout.addLayout(left_column)
        main_layout.addLayout(middle_column)
        main_layout.addLayout(right_column)
        
        self.setLayout(main_layout)
        
        # 显式调用一次模式切换，确保界面参数与默认模式一致
        self.on_mode_changed(self.mode_combo.currentText())

    def on_param_changed_and_update_plot(self, param_name, value):
        """参数变化时更新曲线并发送参数更新信号"""
        # 发送参数更新信号
        self.parameter_changed.emit(param_name, value)
        # 更新助力曲线预览
        self.update_assist_curve_preview()

    def on_mode_changed(self, text):
        mode_key = self.mode_combo.currentData()
        if mode_key and mode_key in self.motion_modes:
            self.current_mode = mode_key
            mode_info = self.motion_modes[mode_key]
            
            # 更新模式描述
            self.mode_description.setText(mode_info["description"])
            
            # 更新五次多项式助力曲线参数
            # 伸展阶段
            self.ext_t0_spin.setValue(mode_info["ext_t0"])
            self.ext_tf_spin.setValue(mode_info["ext_tf"])
            self.ext_p_spin.setValue(mode_info["ext_p"])
            self.ext_Tmax_spin.setValue(mode_info["ext_Tmax"])
            
            # 屈曲阶段
            self.flex_t0_spin.setValue(mode_info["flex_t0"])
            self.flex_tf_spin.setValue(mode_info["flex_tf"])
            self.flex_p_spin.setValue(mode_info["flex_p"])
            self.flex_Tmax_spin.setValue(mode_info["flex_Tmax"])
            
            # 偏置相位
            self.phase_bias_spin.setValue(mode_info.get("phase_bias", 0.0))
            self.phase_bias_0p6_spin.setValue(mode_info.get("phase_bias_at_0p6", 0.0))
            self.phase_bias_slope_spin.setValue(mode_info.get("phase_bias_slope", 0.0))
            interp_mode = mode_key in ("walking", "cycling")
            self.phase_bias_0p6_widget.setEnabled(interp_mode)
            self.phase_bias_slope_widget.setEnabled(interp_mode)
            
            # 发送模式切换信号
            self.mode_changed.emit(mode_key)
            
            # 更新助力曲线预览
            self.update_assist_curve_preview()

    def update_phase_bias_display(self, value: float):
        """外部更新时同步偏置相位显示，不触发信号循环。"""
        self.phase_bias_spin.blockSignals(True)
        self.phase_bias_spin.setValue(value)
        self.phase_bias_spin.blockSignals(False)
        if self.current_mode in self.motion_modes:
            self.motion_modes[self.current_mode]["phase_bias"] = value
        self.update_assist_curve_preview()
    
    def create_assist_curve_preview(self):
        """创建助力曲线预览widget"""
        self.curve_preview_group = QGroupBox("助力曲线预览（一个步态周期）")
        curve_layout = QVBoxLayout()
        
        # 创建matplotlib图形，增大尺寸以适应单独一列
        self.figure = Figure(figsize=(6, 8), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        
        curve_layout.addWidget(self.canvas)
        self.curve_preview_group.setLayout(curve_layout)
        
        # 初始化曲线
        self.update_assist_curve_preview()
    
    def quintic_minjerk(self, s):
        """五次多项式（最小加加速度）"""
        return 10 * s**3 - 15 * s**4 + 6 * s**5
    
    def quintic_window(self, phase, t0, tf, p):
        """计算五次多项式窗口函数值"""
        if t0 >= tf:
            return 0.0
        
        # 处理相位循环边界
        if t0 < tf:
            # 正常情况：窗口不跨越边界
            if phase < t0 or phase > tf:
                return 0.0
        else:
            # 窗口跨越0-1边界
            if phase > tf and phase < t0:
                return 0.0
            # 归一化相位到窗口内
            if phase < tf:
                phase = phase + 1.0
        
        # 计算窗口内的归一化位置 s ∈ [0, 1]
        s = (phase - t0) / (tf - t0)
        
        # 分段五次多项式
        if s < p:
            # 上升段：从0到峰值
            s_rise = s / p if p > 0 else 0
            return self.quintic_minjerk(s_rise)
        else:
            # 下降段：从峰值到0
            s_fall = (s - p) / (1 - p) if (1 - p) > 0 else 1
            return self.quintic_minjerk(1 - s_fall)
    
    def calculate_assist_torque(self, phase, params):
        """计算给定相位的助力力矩"""
        # 应用偏置相位
        phase_shifted = (phase + params.get('phase_bias', 0.0)) % 1.0
        
        # 伸展助力
        T_ext = params['ext_Tmax'] * self.quintic_window(
            phase_shifted, params['ext_t0'], params['ext_tf'], params['ext_p']
        )
        
        # 屈曲助力（取负值）
        T_flex = -params['flex_Tmax'] * self.quintic_window(
            phase_shifted, params['flex_t0'], params['flex_tf'], params['flex_p']
        )
        
        # 总力矩 = 伸展 + 屈曲
        return T_ext + T_flex
    
    def update_assist_curve_preview(self):
        """更新助力曲线预览"""
        try:
            # 清除旧图形
            self.ax.clear()
            
            # 获取当前参数（包含phase_bias）
            params = {
                'ext_t0': self.ext_t0_spin.value(),
                'ext_tf': self.ext_tf_spin.value(),
                'ext_p': self.ext_p_spin.value(),
                'ext_Tmax': self.ext_Tmax_spin.value(),
                'flex_t0': self.flex_t0_spin.value(),
                'flex_tf': self.flex_tf_spin.value(),
                'flex_p': self.flex_p_spin.value(),
                'flex_Tmax': self.flex_Tmax_spin.value(),
                'phase_bias': self.phase_bias_spin.value()
            }
            
            # 生成一个步态周期的相位点
            phase_points = np.linspace(0, 1, 200)
            torque_points = [self.calculate_assist_torque(p, params) for p in phase_points]
            
            # 绘制曲线
            self.ax.plot(phase_points * 100, torque_points, 'b-', linewidth=2, label='总助力')
            
            # 标记关键点
            # 伸展区域
            if params['ext_t0'] < params['ext_tf']:
                self.ax.axvspan(params['ext_t0']*100, params['ext_tf']*100, 
                               alpha=0.2, color='green', label='伸展区')
            # 屈曲区域
            if params['flex_t0'] < params['flex_tf']:
                self.ax.axvspan(params['flex_t0']*100, params['flex_tf']*100, 
                               alpha=0.2, color='red', label='屈曲区')
            
            # 添加零线
            self.ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            
            # 设置标签和标题（使用英文避免字体问题）
            self.ax.set_xlabel('Gait Phase (%)', fontsize=11)
            self.ax.set_ylabel('Assist Torque (Nm)', fontsize=11)
            # 使用英文名称避免乱码
            mode_name_en = self.motion_modes[self.current_mode].get("name_en", "Unknown")
            self.ax.set_title(f'Assist Curve - {mode_name_en}', 
                            fontsize=12, fontweight='bold')
            self.ax.grid(True, alpha=0.3)
            # 修改图例标签为英文
            handles, labels = self.ax.get_legend_handles_labels()
            new_labels = []
            for label in labels:
                if '总助力' in label or 'total' in label.lower():
                    new_labels.append('Total Assist')
                elif '伸展' in label or 'extension' in label.lower():
                    new_labels.append('Extension Zone')
                elif '屈曲' in label or 'flexion' in label.lower():
                    new_labels.append('Flexion Zone')
                else:
                    new_labels.append(label)
            self.ax.legend(handles, new_labels, loc='upper right', fontsize=9)
            
            # 设置坐标轴范围
            self.ax.set_xlim(0, 100)
            y_max = max(abs(min(torque_points)), abs(max(torque_points))) * 1.2
            self.ax.set_ylim(-y_max, y_max)
            
            # 刷新画布
            self.figure.tight_layout()
            self.canvas.draw()
            
        except Exception as e:
            print(f"更新曲线预览失败: {e}")


class SimplifiedGaitControlMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ros_interface = ROS2Interface()
        self.init_ui()
        self.setup_connections()
        self.start_system()

    def init_ui(self):
        self.setWindowTitle("步态控制系统 - 简化控制界面")
        self.setGeometry(100, 100, 600, 600)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        # 系统控制
        control_group = QGroupBox("系统控制")
        control_layout = QVBoxLayout()
        
        # 连接按钮
        self.connect_btn = QPushButton("电机使能")
        self.connect_btn.clicked.connect(self.enable_motors)
        
        # 机械标零按钮
        self.mechanical_zero_btn = QPushButton("机械标零")
        self.mechanical_zero_btn.clicked.connect(self.mechanical_zero)
        self.mechanical_zero_btn.setStyleSheet("""
            QPushButton {
                background-color: #FFA500;
                color: white;
                font-weight: bold;
                padding: 8px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #FF8C00;
            }
            QPushButton:pressed {
                background-color: #FF7F00;
            }
            QPushButton:disabled {
                background-color: #CCCCCC;
                color: #666666;
            }
        """)
        self.mechanical_zero_btn.setEnabled(False)  # 初始状态禁用
        
        # 紧急停止按钮
        self.emergency_btn = QPushButton("紧急停止")
        self.emergency_btn.clicked.connect(self.emergency_stop)
        self.emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #DC143C;
                color: white;
                font-weight: bold;
                padding: 8px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #B22222;
            }
            QPushButton:pressed {
                background-color: #8B0000;
            }
        """)
        
        # 状态标签
        self.status_label = QLabel("系统状态: 未连接")
        self.zero_status_label = QLabel("标零状态: 未标零")
        self.zero_status_label.setStyleSheet("color: #DC143C; font-weight: bold;")
        
        control_layout.addWidget(self.connect_btn)
        control_layout.addWidget(self.mechanical_zero_btn)
        control_layout.addWidget(self.emergency_btn)
        control_layout.addWidget(self.status_label)
        control_layout.addWidget(self.zero_status_label)
        control_group.setLayout(control_layout)
        # 参数组件
        self.parameter_control = ParameterControlWidget()
        # 日志
        log_group = QGroupBox("操作日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        # 装配
        main_layout.addWidget(control_group)
        main_layout.addWidget(self.parameter_control)
        main_layout.addWidget(log_group)
        central_widget.setLayout(main_layout)

    def setup_connections(self):
        self.parameter_control.parameter_changed.connect(self.on_parameter_changed)
        self.parameter_control.mode_changed.connect(self.on_mode_changed)
        self.ros_interface.parameter_updated.connect(self.on_parameter_updated_from_ros)

    def start_system(self):
        self.ros_interface.start()
        self.status_label.setText("系统状态: 已连接")
        self.connect_btn.setText("电机使能")
        self.mechanical_zero_btn.setEnabled(True)  # 连接后启用机械标零按钮
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] 系统已连接，请先执行机械标零")

    def enable_motors(self):
        if not self.ros_interface.is_connected():
            QMessageBox.warning(self, "使能失败", "系统未连接，无法使能电机！")
            return
        success = self.ros_interface.publish_motor_enable()
        if success:
            self.log_text.append(f"[{time.strftime('%H:%M:%S')}] 电机使能命令已发送")
        else:
            QMessageBox.warning(self, "使能失败", "电机使能命令发送失败！")

    def mechanical_zero(self):
        """执行机械标零"""
        reply = QMessageBox.question(
            self, 
            "机械标零确认", 
            "确定要执行机械标零吗？\n\n注意：\n1. 确保电机处于安全位置\n2. 标零过程中请勿移动设备\n3. 标零完成后才能开始助力操作",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            success = self.ros_interface.publish_mechanical_zero()
            if success:
                self.log_text.append(f"[{time.strftime('%H:%M:%S')}] 机械标零命令已发送")
                self.zero_status_label.setText("标零状态: 标零中...")
                self.zero_status_label.setStyleSheet("color: #FFA500; font-weight: bold;")
                self.mechanical_zero_btn.setEnabled(False)  # 标零过程中禁用按钮
                
                # 模拟标零完成（实际应该通过ROS话题监听标零完成状态）
                QTimer.singleShot(3000, self.on_mechanical_zero_complete)
            else:
                QMessageBox.warning(self, "标零失败", "机械标零命令发送失败！")

    def on_mechanical_zero_complete(self):
        """机械标零完成回调"""
        self.zero_status_label.setText("标零状态: 已完成")
        self.zero_status_label.setStyleSheet("color: #32CD32; font-weight: bold;")
        self.mechanical_zero_btn.setEnabled(True)  # 重新启用按钮，允许再次标零
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] 机械标零完成，系统准备就绪，可以开始助力操作")
        QMessageBox.information(self, "标零完成", "机械标零已完成！\n系统准备就绪，可以开始助力操作。")

    def emergency_stop(self):
        self.ros_interface.publish_mode("emergency_stop")
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] 紧急停止指令已发送")
        QMessageBox.warning(self, "紧急停止", "紧急停止指令已发送！")

    @pyqtSlot(str)
    def on_mode_changed(self, mode):
        self.ros_interface.publish_mode(mode)
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] 运动模式切换: {mode}")

    @pyqtSlot(str, float)
    def on_parameter_changed(self, param_name, value):
        success = self.ros_interface.set_parameter(param_name, value)
        msg = "成功" if success else "失败"
        self.log_text.append(f"[{time.strftime('%H:%M:%S')}] 参数设置{msg}: {param_name} = {value}")
    
    @pyqtSlot(str, float)
    def on_parameter_updated_from_ros(self, param_name, value):
        """收到控制节点的参数更新时，同步GUI显示（主要用于自动相位偏置）。"""
        if param_name.endswith("phase_bias"):
            self.parameter_control.update_phase_bias_display(value)
            self.log_text.append(f"[{time.strftime('%H:%M:%S')}] 自动更新: {param_name} = {value:.3f}")

    def closeEvent(self, event):
        self.ros_interface.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = SimplifiedGaitControlMainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
