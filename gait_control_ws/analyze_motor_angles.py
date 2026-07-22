#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步态电机角度分析与相位估计
提取CSV文件中的左右电机角度，估计相位并绘图
包含步态开始/结束检测逻辑
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from scipy.interpolate import interp1d
import os
import glob
from collections import deque

# 全局配置参数
CONFIG = {
    'FILTER_CUTOFF': 0.3,      # 高通滤波器截止频率 (Hz)
    'OSC_OMEGA_INIT': 2 * np.pi * 0.4, # 初始频率 (rad/s)
    'OSC_OMEGA_MIN': 0.5,      # 最小频率 (rad/s)
    'OSC_OMEGA_MAX': 10.0,     # 最大频率 (rad/s)
}

# 设置中文字体支持
# 优先尝试系统已安装的中文字体
plt.rcParams['font.sans-serif'] = [
    'Noto Sans CJK SC',     # Linux (Google Noto Fonts)
    'AR PL UMing CN',       # Linux (Arphic Fonts)
    'WenQuanYi Micro Hei',  # Linux
    'SimHei',               # Windows
    'Microsoft YaHei',      # Windows
    'DejaVu Sans'           # Fallback
]
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


def load_gait_data(csv_file):
    """
    加载步态数据
    
    参数:
        csv_file: CSV文件路径
    
    返回:
        DataFrame: 步态数据
    """
    df = pd.read_csv(csv_file)
    print(f"数据加载成功，共{len(df)}条记录")
    print(f"数据列: {df.columns.tolist()}")
    return df


def estimate_phase_hilbert(angle_data, fs=100):
    """
    使用Hilbert变换估计相位
    
    参数:
        angle_data: 角度数据数组
        fs: 采样频率 (Hz)
    
    返回:
        phase: 估计的相位 (0到2π)
    """
    # 去除均值
    angle_centered = angle_data - np.mean(angle_data)
    
    # Hilbert变换
    analytic_signal = signal.hilbert(angle_centered)
    
    # 提取瞬时相位
    instantaneous_phase = np.angle(analytic_signal)
    
    # 将相位从[-π, π]转换到[0, 2π]
    phase = np.mod(instantaneous_phase, 2 * np.pi)
    
    return phase


def estimate_phase_derivative(angle_data, time_data):
    """
    基于角度导数（角速度）估计相位
    
    参数:
        angle_data: 角度数据数组
        time_data: 时间数据数组
    
    返回:
        phase: 估计的相位 (0到2π)
    """
    # 计算角速度
    velocity = np.gradient(angle_data, time_data)
    
    # 寻找峰值点（步态周期的起始点）
    peaks, _ = signal.find_peaks(angle_data, distance=50, prominence=0.1)
    
    if len(peaks) < 2:
        print("警告: 检测到的峰值过少，使用Hilbert方法")
        return estimate_phase_hilbert(angle_data)
    
    # 初始化相位数组
    phase = np.zeros_like(angle_data)
    
    # 对每个步态周期进行相位估计
    for i in range(len(peaks) - 1):
        start_idx = peaks[i]
        end_idx = peaks[i + 1]
        
        # 在一个周期内线性插值相位从0到2π
        cycle_length = end_idx - start_idx
        phase[start_idx:end_idx] = np.linspace(0, 2 * np.pi, cycle_length, endpoint=False)
    
    # 处理第一个周期之前的数据
    if peaks[0] > 0:
        phase[0:peaks[0]] = np.linspace(0, 2 * np.pi, peaks[0], endpoint=False)
    
    # 处理最后一个周期之后的数据
    if peaks[-1] < len(angle_data):
        remaining = len(angle_data) - peaks[-1]
        phase[peaks[-1]:] = np.linspace(0, 2 * np.pi, remaining, endpoint=False)
    
    return phase


def estimate_phase_adaptive_oscillator(signal_data, dt):
    """
    基于MATLAB代码移植的自适应振荡器
    
    参数:
        signal_data: 输入信号数组
        dt: 时间步长
    
    返回:
        phase: 估计的相位 (0到2π)
    """
    # 预处理：使用高通滤波器去除直流分量和低频漂移
    try:
        cutoff = CONFIG['FILTER_CUTOFF']
        sos = signal.butter(2, cutoff, 'hp', fs=1/dt, output='sos')
        u = signal.sosfiltfilt(sos, signal_data)
    except Exception as e:
        print(f"滤波失败，回退到去均值: {e}")
        u = signal_data - np.mean(signal_data)
        
    # 计算速度 (dq) 用于初始化
    dq = np.gradient(u, dt)
    
    n = len(u)
    phase = np.zeros(n)
    
    # 振荡器阶数 (MATLAB code: order=2)
    # Index 0: 0阶 (DC)
    # Index 1: 1阶 (基频)
    order = 2
    para = np.zeros((order, 3)) # [amp, freq, phase]
    
    # 初始频率 (使用配置值)
    init_freq = CONFIG['OSC_OMEGA_INIT']
    
    # 初始化参数 (对应 MATLAB 代码中的 init 部分)
    # 0阶振荡器
    para[0, 0] = 0.0      # amp
    para[0, 2] = np.pi/2  # phase
    
    # 1阶振荡器 (基频)
    para[1, 0] = 0.4      # amp
    para[1, 1] = init_freq # freq
    
    # 初始相位: atan2(omega*q, dq)
    if abs(dq[0]) > 1e-6:
        para[1, 2] = np.arctan2(para[1, 1] * u[0], dq[0])
    else:
        para[1, 2] = 0.0
        
    # 修正幅值: max(q/sin(fai), 0.3)
    if abs(np.sin(para[1, 2])) > 1e-6:
        para[1, 0] = max(u[0] / np.sin(para[1, 2]), 0.3)
    else:
        para[1, 0] = 0.3
        
    # 循环处理
    for i in range(n):
        q = u[i]
        
        # 动态计算收敛参数 (基于当前频率)
        current_freq = para[1, 1]
        if current_freq < 0.1: current_freq = 0.1 # 防止除零
        
        T = 2 * np.pi / current_freq
        T_aefa = 2 * T
        T_amig = 2 * T
        
        yita = 2 / T_aefa
        vaomig = 20 / (T_amig**2)
        vfai = np.sqrt(24.2 * vaomig)
        
        # 1. 估计输出 (Tracking)
        q_esti = np.zeros(order)
        for j in range(order):
            q_esti[j] = para[j, 0] * np.sin(para[j, 2])
            
        y_out = np.sum(q_esti)
        F = q - y_out # 估计误差
        
        # 2. 计算参数变化率 (Derivatives)
        para_dot = np.zeros((order, 3))
        
        # 0阶幅值变化率
        para_dot[0, 0] = yita * F
        
        # 1阶及以上
        for j in range(1, order):
            # 幅值变化率
            para_dot[j, 0] = yita * F * np.sin(para[j, 2])
            
            # 频率变化率 (仅基频)
            if j == 1:
                para_dot[j, 1] = vaomig * F * np.cos(para[j, 2])
            
            # 相位变化率
            # MATLAB: para_dot(j,3)=para(j,2)+vfai*F*cos(para(j,3));
            para_dot[j, 2] = para[j, 1] + vfai * F * np.cos(para[j, 2])
            
            # 相位变化率非负限制
            if para_dot[j, 2] < 0:
                para_dot[j, 2] = 0
                
        # 3. 更新参数 (Euler Integration)
        
        # 更新幅值
        para[0, 0] += para_dot[0, 0] * dt
        para[0, 0] = 0 # MATLAB代码强制置零
        
        for j in range(1, order):
            para[j, 0] += para_dot[j, 0] * dt
            
        # 更新频率 (基频)
        para[1, 1] += para_dot[1, 1] * dt
        if para[1, 1] < 0: para[1, 1] = 0
        
        # 频率限制 (可选，防止发散)
        para[1, 1] = np.clip(para[1, 1], CONFIG['OSC_OMEGA_MIN'], CONFIG['OSC_OMEGA_MAX'])
        
        # 更新相位
        for j in range(1, order):
            para[j, 2] += para_dot[j, 2] * dt
            
        # 存储结果 (取基频相位)
        phase[i] = np.mod(para[1, 2], 2 * np.pi)
    
    return phase


class GaitDetector:
    """步态开始/结束检测器"""
    
    def __init__(self, buffer_size=100, dt=0.05, rT=0.4):
        self.buffer_size = buffer_size
        self.dt = dt
        self.rT = rT  # 步态检测阈值
        
        # 缓冲区
        self.lhip_buffer = deque(maxlen=buffer_size)
        self.rhip_buffer = deque(maxlen=buffer_size)
        
        # 步态状态
        self.gait_state = 0  # 0=停止, 1=步行
        self.timer_r = 0.0   # r值连续小于阈值的计时器
        
    def add_data(self, left_angle, right_angle):
        """添加角度数据到缓冲区"""
        self.lhip_buffer.append(left_angle)
        self.rhip_buffer.append(right_angle)
    
    def detect(self, left_angle, right_angle):
        """
        检测步态开始和结束
        
        Returns:
            gait_state: 0=停止, 1=步行
            state_changed: 状态是否发生变化
            r_value: 当前r值
        """
        # 添加数据
        self.add_data(left_angle, right_angle)
        
        # 需要足够的数据才能计算
        if len(self.lhip_buffer) < 20 or len(self.rhip_buffer) < 20:
            return self.gait_state, False, 0.0
        
        # 计算 r(i) = sqrt((θ_L(i) - θ_L_mean)² + (θ_R(i) - θ_R_mean)²)
        left_mean = np.mean(list(self.lhip_buffer)[-20:])
        right_mean = np.mean(list(self.rhip_buffer)[-20:])
        
        r_value = np.sqrt((left_angle - left_mean)**2 + (right_angle - right_mean)**2)
        
        state_changed = False
        
        if self.gait_state == 0:  # 当前为停止状态
            # 步态开始条件：r > rT
            if r_value > self.rT:
                self.gait_state = 1
                self.timer_r = 0.0
                state_changed = True
        else:  # 当前为步行状态
            # 检查是否需要停止步态
            if r_value > self.rT:
                self.timer_r = 0.0
            else:
                self.timer_r += self.dt
            
            # 步态停止条件：timer_r > 2.5秒
            if self.timer_r > 2.5:
                self.gait_state = 0
                self.timer_r = 0.0
                state_changed = True
        
        return self.gait_state, state_changed, r_value
    
    def reset(self):
        """重置检测器"""
        self.gait_state = 0
        self.timer_r = 0.0
        self.lhip_buffer.clear()
        self.rhip_buffer.clear()


def plot_file_analysis(df, left_phase_est, right_phase_est, detected_state, r_values, output_path):
    """
    绘制单个文件的分析结果
    包含：左右腿角度、角度差、步态检测、相位对比
    """
    time = np.arange(len(df)) * 0.05
    angle_diff = df['left_angle'] - df['right_angle']
    
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    filename = os.path.basename(output_path)
    fig.suptitle(f'步态分析: {filename}', fontsize=16)
    
    # 1. 左右腿角度
    axes[0, 0].plot(time, df['left_angle'], 'b', label='左腿角度')
    axes[0, 0].plot(time, df['right_angle'], 'r', label='右腿角度')
    axes[0, 0].set_title('左右腿角度')
    axes[0, 0].set_xlabel('时间 (s)')
    axes[0, 0].set_ylabel('角度 (rad)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # 2. 角度差
    axes[0, 1].plot(time, angle_diff, 'g', label='角度差 (左-右)')
    axes[0, 1].fill_between(time, min(angle_diff), max(angle_diff), 
                            where=detected_state>0, alpha=0.2, color='green', label='检测到步行')
    axes[0, 1].set_title('左右腿角度差')
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('角度 (rad)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # 3. r值和检测阈值
    axes[1, 0].plot(time, r_values, 'b-', linewidth=1.5, label='r值')
    axes[1, 0].axhline(y=0.4, color='r', linestyle='--', linewidth=2, label='阈值 (rT=0.4)')
    axes[1, 0].fill_between(time, 0, max(r_values) if len(r_values) > 0 else 1, 
                            where=detected_state>0, alpha=0.2, color='green', label='检测到步行')
    axes[1, 0].set_title('步态检测指标')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('r值')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # 4. 检测到的步态状态
    axes[1, 1].plot(time, detected_state, 'b-', linewidth=2, label='检测状态')
    axes[1, 1].set_title('步态状态检测')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('步态状态')
    axes[1, 1].set_yticks([0, 1])
    axes[1, 1].set_yticklabels(['停止', '步行'])
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    # 5. 左腿相位 (文件 vs 估计)
    axes[2, 0].plot(time, df['phase_left'], 'b--', alpha=0.6, label='文件相位')
    axes[2, 0].plot(time, left_phase_est, 'b', label='估计相位')
    axes[2, 0].fill_between(time, 0, 2*np.pi, 
                            where=detected_state>0, alpha=0.1, color='green')
    axes[2, 0].set_title('左腿相位对比')
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].set_ylabel('相位 (rad)')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)
    
    # 6. 右腿相位 (文件 vs 估计)
    axes[2, 1].plot(time, df['phase_right'], 'r--', alpha=0.6, label='文件相位')
    axes[2, 1].plot(time, right_phase_est, 'r', label='估计相位')
    axes[2, 1].fill_between(time, 0, 2*np.pi, 
                            where=detected_state>0, alpha=0.1, color='green')
    axes[2, 1].set_title('右腿相位对比')
    axes[2, 1].set_xlabel('时间 (s)')
    axes[2, 1].set_ylabel('相位 (rad)')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def process_file(csv_file, output_dir):
    """
    处理单个CSV文件，包含步态检测
    """
    try:
        df = pd.read_csv(csv_file)
        
        # 检查必要的数据列
        required_cols = ['left_angle', 'right_angle', 'phase_left', 'phase_right']
        if not all(col in df.columns for col in required_cols):
            print(f"跳过 {os.path.basename(csv_file)}: 缺少必要数据列")
            return

        # 提取数据
        left_angle = df['left_angle'].values
        right_angle = df['right_angle'].values
        n_samples = len(left_angle)
        dt = 0.05  # 假设50ms采样间隔
        
        # 初始化步态检测器
        detector = GaitDetector(buffer_size=100, dt=dt, rT=0.4)
        
        # 存储检测结果
        detected_state = np.zeros(n_samples)
        r_values = np.zeros(n_samples)
        left_phase_est = np.zeros(n_samples)
        right_phase_est = np.zeros(n_samples)
        
        # 逐点处理，进行步态检测
        angle_diff = left_angle - right_angle
        
        for i in range(n_samples):
            # 步态检测
            state, state_changed, r_val = detector.detect(left_angle[i], right_angle[i])
            detected_state[i] = state
            r_values[i] = r_val
        
        # 使用自适应振荡器估计相位
        base_phase = estimate_phase_adaptive_oscillator(angle_diff, dt)
        
        # 根据检测到的步态状态修正相位
        # 在停止状态下，相位应该重置为0
        for i in range(n_samples):
            if detected_state[i] == 0:  # 停止状态
                left_phase_est[i] = 0.0
                right_phase_est[i] = np.pi
            else:  # 步行状态
                left_phase_est[i] = base_phase[i]
                right_phase_est[i] = np.mod(base_phase[i] + np.pi, 2 * np.pi)
        
        # 生成输出文件名
        basename = os.path.basename(csv_file).replace('.csv', '.png')
        output_path = os.path.join(output_dir, basename)
        
        # 绘图
        plot_file_analysis(df, left_phase_est, right_phase_est, detected_state, r_values, output_path)
        
        # 统计信息
        walking_time = np.sum(detected_state) * dt
        total_time = n_samples * dt
        walking_ratio = walking_time / total_time * 100 if total_time > 0 else 0
        
        print(f"已处理: {os.path.basename(csv_file)} -> {basename}")
        print(f"  总时长: {total_time:.1f}s, 步行时长: {walking_time:.1f}s ({walking_ratio:.1f}%)")
        
    except Exception as e:
        print(f"处理 {os.path.basename(csv_file)} 时出错: {e}")


def main():
    """
    主函数：批量处理所有CSV文件
    """
    input_dir = 'gait_logs'
    output_dir = 'analysis_results'
    
    # 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"创建输出目录: {output_dir}")
    
    # 获取所有CSV文件
    csv_files = sorted(glob.glob(os.path.join(input_dir, '*.csv')))
    
    if not csv_files:
        print(f"在 {input_dir} 中未找到CSV文件")
        return
        
    print(f"找到 {len(csv_files)} 个CSV文件，开始处理...")
    print("="*60)
    
    for csv_file in csv_files:
        process_file(csv_file, output_dir)
    
    print("="*60)
    print(f"所有文件处理完成！结果保存在 {output_dir} 目录中。")


if __name__ == '__main__':
    main()
