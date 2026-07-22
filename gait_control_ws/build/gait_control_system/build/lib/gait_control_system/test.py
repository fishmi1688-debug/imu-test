import numpy as np
import matplotlib.pyplot as plt
import math

def calculate_phase_based_torque(phi, A_ext, phi_ext_start, phi_ext_end, 
                                A_fle, phi_fle_start, phi_fle_end, zero_gap):
    """
    根据步态相位计算助力力矩（从代码中提取的函数）
    参数：
        phi: 当前步态相位 (0-2π)
        A_ext: 伸展助力峰值
        phi_ext_start: 伸展助力起始相位
        phi_ext_end: 伸展助力结束相位
        A_fle: 屈曲助力峰值
        phi_fle_start: 屈曲助力起始相位
        phi_fle_end: 屈曲助力结束相位
        zero_gap: 零助力过渡区宽度
    返回：
        力矩值 (Nm)
    """
    # 将相位归一化到 [0, 2π] 范围
    phi = phi % (2 * np.pi)
    
    # 检查是否在伸展助力区间
    if phi_ext_start <= phi <= phi_ext_end:
        # 伸展助力：正向半正弦波
        if phi_ext_end > phi_ext_start:
            phase_progress = (phi - phi_ext_start) / (phi_ext_end - phi_ext_start)
            torque = A_ext * math.sin(np.pi * phase_progress)
            return torque
        else:
            return 0.0
    
    # 检查是否在屈曲助力区间
    elif phi_fle_start <= phi <= phi_fle_end:
        # 屈曲助力：反向半正弦波
        if phi_fle_end > phi_fle_start:
            phase_progress = (phi - phi_fle_start) / (phi_fle_end - phi_fle_start)
            torque = -A_fle * math.sin(np.pi * phase_progress)
            return torque
        else:
            return 0.0
    
    # 处理跨越2π边界的情况（屈曲助力区间可能跨越边界）
    elif phi_fle_start > phi_fle_end:  # 跨越2π边界
        if phi >= phi_fle_start or phi <= phi_fle_end:
            # 计算在跨边界区间内的相位进度
            if phi >= phi_fle_start:
                # 从起始点到2π
                total_range = (2*np.pi - phi_fle_start) + phi_fle_end
                phase_progress = (phi - phi_fle_start) / total_range
            else:
                # 从0到结束点
                total_range = (2*np.pi - phi_fle_start) + phi_fle_end
                phase_progress = ((2*np.pi - phi_fle_start) + phi) / total_range
            
            torque = -A_fle * math.sin(np.pi * phase_progress)
            return torque
    
    # 处理跨越2π边界的情况（伸展助力区间可能跨越边界）
    elif phi_ext_start > phi_ext_end:  # 跨越2π边界
        if phi >= phi_ext_start or phi <= phi_ext_end:
            # 计算在跨边界区间内的相位进度
            if phi >= phi_ext_start:
                # 从起始点到2π
                total_range = (2*np.pi - phi_ext_start) + phi_ext_end
                phase_progress = (phi - phi_ext_start) / total_range
            else:
                # 从0到结束点
                total_range = (2*np.pi - phi_ext_start) + phi_ext_end
                phase_progress = ((2*np.pi - phi_ext_start) + phi) / total_range
            
            torque = A_ext * math.sin(np.pi * phase_progress)
            return torque
    
    # 其他情况：零助力过渡区或未定义区间
    return 0.0

def plot_torque_curves_from_code():
    """绘制代码中定义的各种运动模式的力矩曲线"""
    
    # 从代码中提取的运动模式参数
    motion_modes = {
        "walking": {
            "name": "平地行走",
            "A_ext": 3.0,           # 伸展助力峰值 (Nm)
            "phi_ext_start": 0.0,   # 伸展助力起始相位 (rad)
            "phi_ext_end": np.pi,   # 伸展助力结束相位 (rad)
            "A_fle": 2.5,           # 屈曲助力峰值 (Nm)
            "phi_fle_start": np.pi + 0.5,  # 屈曲助力起始相位 (rad)
            "phi_fle_end": 2*np.pi - 0.5,  # 屈曲助力结束相位 (rad)
            "zero_gap": 0.5,        # 零助力过渡区宽度 (rad)
            "color": "blue"
        },
        "stairs_up": {
            "name": "上楼梯",
            "A_ext": 5.0,           # 上楼需要更大的伸展助力
            "phi_ext_start": 0.0,
            "phi_ext_end": np.pi,
            "A_fle": 4.0,           # 上楼需要更大的屈曲助力
            "phi_fle_start": np.pi + 0.3,
            "phi_fle_end": 2*np.pi - 0.3,
            "zero_gap": 0.3,
            "color": "red"
        },
        "stairs_down": {
            "name": "下楼梯",
            "A_ext": 2.0,           # 下楼伸展助力较小
            "phi_ext_start": 0.2,
            "phi_ext_end": np.pi - 0.2,
            "A_fle": 3.0,           # 下楼屈曲助力适中
            "phi_fle_start": np.pi + 0.7,
            "phi_fle_end": 2*np.pi - 0.7,
            "zero_gap": 0.7,
            "color": "green"
        },
        "cycling": {
            "name": "骑自行车",
            "A_ext": 4.0,           # 踩踏伸展助力
            "phi_ext_start": 0.3,
            "phi_ext_end": np.pi + 0.3,
            "A_fle": 3.5,           # 踩踏屈曲助力
            "phi_fle_start": np.pi + 0.8,
            "phi_fle_end": 2*np.pi,
            "zero_gap": 0.4,
            "color": "orange"
        }
    }
    
    # 相位数组 (0 到 2π)
    phase_array = np.linspace(0, 2*np.pi, 1000)
    
    # 创建子图
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('步态相位助力力矩曲线 (基于代码实现)', fontsize=16, fontweight='bold')
    
    # 扁平化axes便于遍历
    axes_flat = axes.flatten()
    
    # 为每种运动模式绘制力矩曲线
    for idx, (mode_key, params) in enumerate(motion_modes.items()):
        ax = axes_flat[idx]
        
        # 计算该模式下的力矩曲线
        torque_curve = []
        for phi in phase_array:
            torque = calculate_phase_based_torque(
                phi, 
                params["A_ext"], params["phi_ext_start"], params["phi_ext_end"],
                params["A_fle"], params["phi_fle_start"], params["phi_fle_end"],
                params["zero_gap"]
            )
            torque_curve.append(torque)
        
        # 绘制力矩曲线
        ax.plot(phase_array, torque_curve, color=params["color"], linewidth=3, 
                label=f'{params["name"]} 助力曲线')
        
        # 标记关键相位点
        # 伸展助力区间
        ax.axvspan(params["phi_ext_start"], params["phi_ext_end"], 
                   alpha=0.2, color='blue', label='伸展助力区')
        
        # 屈曲助力区间
        if params["phi_fle_start"] > params["phi_fle_end"]:  # 跨越边界
            ax.axvspan(params["phi_fle_start"], 2*np.pi, 
                       alpha=0.2, color='red', label='屈曲助力区')
            ax.axvspan(0, params["phi_fle_end"], 
                       alpha=0.2, color='red')
        else:
            ax.axvspan(params["phi_fle_start"], params["phi_fle_end"], 
                       alpha=0.2, color='red', label='屈曲助力区')
        
        # 添加零线
        ax.axhline(y=0, color='black', linestyle='--', alpha=0.5)
        
        # 设置图形属性
        ax.set_xlabel('步态相位 (rad)', fontsize=12)
        ax.set_ylabel('助力力矩 (Nm)', fontsize=12)
        ax.set_title(f'{params["name"]} - 助力力矩曲线', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=10)
        
        # 设置坐标轴
        ax.set_xlim(0, 2*np.pi)
        y_max = max(params["A_ext"], params["A_fle"]) * 1.2
        ax.set_ylim(-y_max, y_max)
        
        # 设置x轴刻度标签
        ax.set_xticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
        ax.set_xticklabels(['0', 'π/2', 'π', '3π/2', '2π'])
        
        # 添加参数信息
        info_text = f'伸展峰值: {params["A_ext"]} Nm\n屈曲峰值: {params["A_fle"]} Nm'
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    return motion_modes

def plot_comparison_curve():
    """绘制所有运动模式的对比图"""
    
    # 运动模式参数（简化版，用于对比）
    modes = {
        "walking": {"A_ext": 3.0, "A_fle": 2.5, "color": "blue"},
        "stairs_up": {"A_ext": 5.0, "A_fle": 4.0, "color": "red"},
        "stairs_down": {"A_ext": 2.0, "A_fle": 3.0, "color": "green"},
        "cycling": {"A_ext": 4.0, "A_fle": 3.5, "color": "orange"}
    }
    
    phase_array = np.linspace(0, 2*np.pi, 1000)
    
    plt.figure(figsize=(14, 8))
    
    for mode_name, params in modes.items():
        # 简化的助力曲线（基于原始设计思路）
        torque_curve = []
        for phi in phase_array:
            # 伸展阶段 (0 到 π)
            if 0 <= phi <= np.pi:
                torque = params["A_ext"] * np.sin(phi)
            # 屈曲阶段 (π 到 2π)
            else:
                torque = -params["A_fle"] * np.sin(phi - np.pi)
            torque_curve.append(torque)
        
        plt.plot(phase_array, torque_curve, color=params["color"], linewidth=3, 
                label=f'{mode_name} (伸展:{params["A_ext"]}Nm, 屈曲:{params["A_fle"]}Nm)')
    
    # 添加相位区间标识
    plt.axvspan(0, np.pi, alpha=0.1, color='blue', label='伸展相位区间')
    plt.axvspan(np.pi, 2*np.pi, alpha=0.1, color='red', label='屈曲相位区间')
    
    plt.axhline(y=0, color='black', linestyle='--', alpha=0.7)
    plt.xlabel('步态相位 (rad)', fontsize=14)
    plt.ylabel('助力力矩 (Nm)', fontsize=14)
    plt.title('不同运动模式的助力力矩对比', fontsize=16, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=12, loc='upper right')
    
    # 设置x轴刻度
    plt.xticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
               ['0', 'π/2', 'π', '3π/2', '2π'])
    
    plt.tight_layout()
    plt.show()

def analyze_torque_characteristics():
    """分析力矩特性并输出关键参数"""
    
    motion_modes = {
        "walking": {"A_ext": 3.0, "A_fle": 2.5, "name": "平地行走"},
        "stairs_up": {"A_ext": 5.0, "A_fle": 4.0, "name": "上楼梯"},
        "stairs_down": {"A_ext": 2.0, "A_fle": 3.0, "name": "下楼梯"},
        "cycling": {"A_ext": 4.0, "A_fle": 3.5, "name": "骑自行车"}
    }
    
    print("="*60)
    print("步态助力系统 - 力矩特性分析")
    print("="*60)
    
    for mode_key, params in motion_modes.items():
        print(f"\n🚶 {params['name']} ({mode_key}):")
        print(f"   伸展助力峰值: {params['A_ext']:.1f} Nm")
        print(f"   屈曲助力峰值: {params['A_fle']:.1f} Nm")
        print(f"   伸展/屈曲比率: {params['A_ext']/params['A_fle']:.2f}")
        
        # 计算等效功率（假设角速度为1 rad/s）
        avg_power = (params['A_ext'] + params['A_fle']) / 2
        print(f"   平均助力强度: {avg_power:.1f} Nm")
        
        # 根据力矩特性给出适用建议
        if params['A_ext'] > params['A_fle']:
            print(f"   特点: 偏重伸展助力，适合需要推进力的场景")
        elif params['A_fle'] > params['A_ext']:
            print(f"   特点: 偏重屈曲助力，适合需要抬腿力的场景")
        else:
            print(f"   特点: 伸展屈曲平衡，适合平衡性运动")
    
    print("\n="*60)
    print("💡 使用建议:")
    print("   1. 平地行走: 平衡的助力模式，适合日常使用")
    print("   2. 上楼梯: 高强度双向助力，应对爬升阻力")
    print("   3. 下楼梯: 适度助力，保持控制性")
    print("   4. 骑自行车: 高伸展助力，配合踩踏动作")
    print("="*60)

# 执行绘图和分析
if __name__ == "__main__":
    print("🦾 髋关节助力系统 - 力矩曲线可视化")
    print("基于提供代码的实际实现\n")
    
    # 分析力矩特性
    analyze_torque_characteristics()
    
    # 绘制详细的力矩曲线
    print("\n📊 正在绘制详细力矩曲线...")
    motion_modes = plot_torque_curves_from_code()
    
    # 绘制对比曲线
    print("📈 正在绘制对比曲线...")
    plot_comparison_curve()
    
    print("\n✅ 力矩曲线绘制完成！")
    print("💡 力矩曲线特点:")
    print("   - 伸展相位 (0-π): 正向半正弦波，提供向前推进力")
    print("   - 屈曲相位 (π-2π): 负向半正弦波，辅助抬腿动作")
    print("   - 不同运动模式通过调整峰值和相位范围适应各种场景")
    print("   - 零助力过渡区确保助力方向切换的平滑性")