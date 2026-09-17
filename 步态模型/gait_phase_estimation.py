import numpy as np
import pandas as pd
import glob
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from sklearn.preprocessing import StandardScaler
import joblib
import tensorflow as tf
from tensorflow import keras

# 待确定的超参数（来自 optimization_output/best_config.csv）
SAMPLE_RATE   = 30      # Hz
WINDOW_N      = 27      # 最优窗口（900 ms @ 30 Hz）

HIDDEN_CONFIG = [10, 20]       # 优化器搜索得到的最优隐藏层结构
ACTIVATION    = 'relu'         # 优化器搜索得到的最优激活函数
LEARNING_RATE = 0.0005          # 优化器搜索得到的最优学习率
BATCH_SIZE    = 256            # 优化器搜索得到的最优 Batch 大小
MAX_EPOCHS    = 500            # 最大训练轮数

# IMU 通道（10 通道，来自优化器 Step 3 通道筛选结果）
# 包含双侧 Euler + Gyr + 部分 Acc，确保角度信息和角速度信息并存
# 论文原文: "3-axis acceleration and 3-axis angular velocity from the sensors
#   to be the optimal input vector" — 角速度是区分摆动/支撑期、适应变速的关键
IMU_CHANNELS = [
    'left_imu_Euler_Y', 'left_imu_Euler_X', 'right_imu_Euler_Y',
    'right_imu_Euler_X',
    'left_imu_Gyr_X',    'left_imu_Gyr_Y',    'left_imu_Gyr_Z',
    'left_imu_Acc_X',    'left_imu_Acc_Y',     'left_imu_Acc_Z',
]
N_CHANNELS        = len(IMU_CHANNELS)            # 当前启用的 IMU 通道数
N_FEATS_PER_CH    = 7                            # 每通道 7 个统计特征
INPUT_DIM         = N_CHANNELS * N_FEATS_PER_CH  # 输入维度 = 通道数 × 特征数

DATA_DIR          = r'data'
TRAIN_DIR         = os.path.join(DATA_DIR, 'train')
VAL_DIR           = os.path.join(DATA_DIR, 'val')
TEST_DIR          = os.path.join(DATA_DIR, 'test')
SAVE_DIR          = r'model_output'

if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR, exist_ok=True)


PLOT_THEME = {
    'figure_bg': '#ffffff',
    'panel_bg': '#ffffff',
    'grid': '#d8cec2',
    'spine': '#cbbfae',
    'text': '#2f2a24',
    'muted': '#6e6458',
    'truth': '#2f7ebc',
    'pred': '#f28e2b',
    'error_pos': '#e76f51',
    'error_neg': '#4d7ea8',
    'accent': '#d9c3a3',
    'success': '#3b8b6d',
}


# ════════════════════════════════════════════════
# 1. 特征提取工具函数
# ════════════════════════════════════════════════

def extract_window_features(window: np.ndarray) -> np.ndarray:
    """
    从单个通道的滑动窗口片段中提取7个统计特征。
    对应论文 Section III - "seven features (maximum, minimum, mean,
    standard deviation, first, middle, and last)"

    参数:
        window: 形状 (W,) 的一维数组
    返回:
        7维特征向量
    """
    mid = len(window) // 2
    return np.array([
        window.max(),     # 1. 最大值
        window.min(),     # 2. 最小值
        window.mean(),    # 3. 均值
        window.std(),     # 4. 标准差
        window[0],        # 5. 首值
        window[mid],      # 6. 中值
        window[-1],       # 7. 末值
    ], dtype=np.float32)


def compute_stride_rate(phase: np.ndarray, time_sec: np.ndarray) -> np.ndarray:
    """
    计算每个时间步的步频 r = 1/ΔT_stride（步态周期持续时间的倒数）。
    对应论文公式 (3): r = 1/ΔT_current

    方法：通过检测相位从 ~1 跳变到 ~0 的时刻（足跟着地事件）来分割步态周期。

    参数:
        phase:    归一化步态相位数组 (0~1)
        time_sec: 对应的时间戳（秒）
    返回:
        与 phase 等长的步频数组（Hz）
    """
    n = len(phase)
    r = np.zeros(n, dtype=np.float32)

    # 检测步态周期边界：相位从 >0.9 跳到 <0.1（足跟着地）
    heel_strikes = [0]
    for i in range(1, n):
        if phase[i] < 0.1 and phase[i - 1] > 0.9:
            heel_strikes.append(i)
    heel_strikes.append(n)

    # 对每个步态周期，计算周期时长并赋予 r 值
    for k in range(len(heel_strikes) - 1):
        s, e = heel_strikes[k], heel_strikes[k + 1]
        T = time_sec[min(e, n - 1)] - time_sec[s]
        if T > 0.3:           # 过滤掉过短的假周期（<0.3s）
            r[s:e] = 1.0 / T
        else:
            # 用相邻周期的均值作为后备
            if k > 0:
                r[s:e] = r[heel_strikes[k - 1]]

    # 边界填充：用第一个有效值向前填充
    first_valid = next((v for v in r if v > 0), 1.0)
    r[r == 0] = first_valid
    return r

def load_file(csv_path: str):
    """
    加载单个 CSV 文件，提取特征矩阵 X、标签矩阵 Y 和原始相位。

    返回:
        X:     (N_valid, 84)  - 滑动窗口特征
        Y:     (N_valid, 3)   - [cos(2πφ), sin(2πφ), r]
        # Y_pred200: (N_valid, 2) - 200ms 后的 [cos, sin]（已注释，不再使用）
        phase: (N_valid,)     - 真实相位 (0~1)
    """
    df = pd.read_csv(csv_path)
    # 兼容多种时间格式：标准 datetime（含时区）、MM:SS.s、纯数值（秒）
    raw_time = df['time']

    # 1) 优先尝试全列 datetime 解析（最稳健，能处理带时区时间戳）
    parsed_dt = pd.to_datetime(raw_time, utc=True, errors='coerce')
    if parsed_dt.notna().all():
        time_sec = (parsed_dt - parsed_dt.iloc[0]).dt.total_seconds().values.astype(np.float32)
    else:
        # 2) 其次尝试纯数值秒
        numeric_sec = pd.to_numeric(raw_time, errors='coerce')
        if numeric_sec.notna().all():
            time_sec = numeric_sec.values.astype(np.float32)
            time_sec = time_sec - time_sec[0]
        else:
            # 3) 逐元素兜底：数值 -> datetime -> 时分秒格式
            def _to_seconds(s):
                s = str(s).strip()

                # 3.1 数值秒
                try:
                    return float(s)
                except ValueError:
                    pass

                # 3.2 单个时间戳（如 2026-03-06 22:25:01.833333333+08:00）
                dt = pd.to_datetime(s, utc=True, errors='coerce')
                if pd.notna(dt):
                    return float(dt.timestamp())

                # 3.3 时分秒格式（如 MM:SS.s 或 HH:MM:SS.s）
                try:
                    parts = s.replace(',', '.').split(':')
                    val = 0.0
                    for p in parts:
                        val = val * 60 + float(p)
                    return val
                except ValueError as exc:
                    raise ValueError(f"无法解析 time 字段值: {s!r}") from exc

            time_sec = raw_time.apply(_to_seconds).values.astype(np.float32)
            time_sec = time_sec - time_sec[0]

    imu_data  = df[IMU_CHANNELS].values.astype(np.float32)   # (N, N_CHANNELS)
    phase_raw = df['gait_phase_left'].values.astype(np.float32)  # 0~1

    # 计算步频 r
    r_vals = compute_stride_rate(phase_raw, time_sec)

    # 标签编码（论文公式 2、3）：连续化相位表示
    x_lbl = np.cos(2 * np.pi * phase_raw)   # cos 分量
    y_lbl = np.sin(2 * np.pi * phase_raw)   # sin 分量

    W = WINDOW_N
    n_valid = len(df) - W + 1

    X = np.zeros((n_valid, INPUT_DIM), dtype=np.float32)
    for i in range(n_valid):
        win = imu_data[i:i + W]              # (W, N_CHANNELS)
        feats = []
        for c in range(N_CHANNELS):
            feats.append(extract_window_features(win[:, c]))
        X[i] = np.concatenate(feats)

    # 使用窗口末端时刻的标签（因果性）
    idx = np.arange(W - 1, len(df))
    Y = np.stack([x_lbl[idx], y_lbl[idx], r_vals[idx]], axis=1)

    # 200ms 超前标签（用于预测评估）已注释
    # future = np.clip(idx + PREDICT_STEPS, 0, len(df) - 1)
    # Y_pred200 = np.stack([x_lbl[future], y_lbl[future]], axis=1)

    phase = phase_raw[idx]
    return X, Y, phase


# ════════════════════════════════════════════════
# 2. 构建 Encoder-Decoder ANN
# ════════════════════════════════════════════════

def build_network(input_dim=INPUT_DIM,
                  hidden_config=HIDDEN_CONFIG,
                  activation=ACTIVATION,
                  output_dim=3):
    """
        构建与论文公式 (10)-(12) 对应的全连接回归网络。

        论文对应结构：
            Input(X) -> Dense(20, tanh) -> Dense(40, tanh) -> Dense(3, linear)

        其中：
            第一层 Dense(20, tanh) 对应公式 (10): E_a = tanh(W_e,a X + b_e,a)
            第二层 Dense(40, tanh) 对应公式 (11): D_a = tanh(W_d,a E_a + b_d,a)
            输出层 Dense(3, linear) 对应公式 (12): [x, y, r]^T = W_o,a D_a + b_o,a
    """
    layers = [keras.layers.Input(shape=(input_dim,))]
    for i, units in enumerate(hidden_config):
        layers.append(keras.layers.Dense(units, activation=activation,
                                         name=f'hidden_{i+1}'))
    layers.append(keras.layers.Dense(output_dim, activation='linear', name='output'))
    return keras.Sequential(layers, name='GaitPhaseNet')


# ════════════════════════════════════════════════
# 3. 评估指标
# ════════════════════════════════════════════════

def recover_phase(pred_xy: np.ndarray) -> np.ndarray:
    """
    从网络输出的 [cos(2πφ), sin(2πφ)] 恢复步态相位 φ ∈ [0, 1)。
    """
    angle = np.arctan2(pred_xy[:, 1], pred_xy[:, 0])  # -π ~ π
    return (angle / (2 * np.pi)) % 1.0                 # 映射到 0~1


def circular_error(pred: np.ndarray, true: np.ndarray) -> np.ndarray:
    """
    计算循环相位误差（处理 0%/100% 跳变边界）。
    误差映射到 [-0.5, 0.5] 范围（即最多差半个步态周期）。
    """
    err = pred - true
    return (err + 0.5) % 1.0 - 0.5


def rRMSE(pred_phase: np.ndarray, true_phase: np.ndarray) -> float:
    """
    相对均方根误差（论文评估指标），以百分比表示。
    rRMSE = sqrt(mean(err²)) × 100%
    """
    err = circular_error(pred_phase, true_phase)
    return float(np.sqrt(np.mean(err ** 2)) * 100)


def apply_phase_rate_limit(phase_seq: np.ndarray,
                           r_seq: np.ndarray,
                           sample_rate: float = SAMPLE_RATE,
                           max_rate_factor: float = 1.5,
                           min_rate_factor: float = 0.2) -> np.ndarray:
    """
    对恢复出的相位序列施加方向和速率约束：
      1. 禁止相位回退（delta < 0 → 使用期望步进值代替）
      2. 限制最大步进速率（delta > max → 截断）
    步频 r 由网络第3个输出预测，单位 Hz。
    """
    out = phase_seq.copy()
    r_clamped = np.clip(r_seq, 0.3, 2.5)  # 合理步频范围: 0.3~2.5 Hz
    for i in range(1, len(out)):
        expected_delta = r_clamped[i] / sample_rate     # 期望每帧相位增量
        delta = (out[i] - out[i - 1] + 0.5) % 1.0 - 0.5  # 循环差值 ∈ [-0.5, 0.5]

        if delta < expected_delta * min_rate_factor:
            # 相位回退或增长过慢 → 用期望步进替代
            delta = expected_delta
        elif delta > expected_delta * max_rate_factor:
            # 增长过快 → 截断到最大允许值
            delta = expected_delta * max_rate_factor

        out[i] = (out[i - 1] + delta) % 1.0
    return out


def configure_plot_style():
    plt.rcParams.update({
        'figure.facecolor': PLOT_THEME['figure_bg'],
        'axes.facecolor': PLOT_THEME['panel_bg'],
        'axes.edgecolor': PLOT_THEME['spine'],
        'axes.labelcolor': PLOT_THEME['text'],
        'axes.titlecolor': PLOT_THEME['text'],
        'axes.labelsize': 10,
        'axes.titlesize': 12,
        'axes.titleweight': 'semibold',
        'xtick.color': PLOT_THEME['muted'],
        'ytick.color': PLOT_THEME['muted'],
        'grid.color': PLOT_THEME['grid'],
        'grid.alpha': 0.35,
        'grid.linewidth': 0.8,
        'legend.frameon': True,
        'legend.facecolor': '#fffaf3',
        'legend.edgecolor': PLOT_THEME['spine'],
        'legend.framealpha': 0.95,
        'font.size': 10,
    })


def style_axis(ax, *, xlabel: str = None, ylabel: str = None):
    ax.set_facecolor(PLOT_THEME['panel_bg'])
    ax.grid(True, axis='y', alpha=0.38)
    ax.grid(True, axis='x', alpha=0.18, linewidth=0.7)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color(PLOT_THEME['spine'])
    ax.spines['bottom'].set_color(PLOT_THEME['spine'])
    ax.tick_params(length=0)
    ax.margins(x=0)
    if xlabel is not None:
        ax.set_xlabel(xlabel)
    if ylabel is not None:
        ax.set_ylabel(ylabel)


def smooth_curve(values: np.ndarray, window: int = 5) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.size < 3 or window <= 1:
        return values.copy()

    window = min(window, values.size)
    kernel = np.ones(window, dtype=float) / window
    pad_left = window // 2
    pad_right = window - 1 - pad_left
    padded = np.pad(values, (pad_left, pad_right), mode='edge')
    return np.convolve(padded, kernel, mode='valid')


def add_badge(ax, text: str, *, accent: str = None):
    ax.text(
        0.99, 0.96, text,
        transform=ax.transAxes,
        ha='right', va='top',
        fontsize=9,
        color=PLOT_THEME['text'],
        bbox=dict(
            boxstyle='round,pad=0.35',
            facecolor='#fff8ef',
            edgecolor=accent or PLOT_THEME['spine'],
            linewidth=0.9,
        ),
    )


def add_figure_header(fig, title: str, subtitle: str):
    fig.suptitle(
        title,
        x=0.055, y=0.985,
        ha='left',
        fontsize=18,
        fontweight='bold',
        color=PLOT_THEME['text'],
    )
    fig.text(
        0.055, 0.955, subtitle,
        ha='left', va='top',
        fontsize=10,
        color=PLOT_THEME['muted'],
    )


def add_header_metric(fig, label: str, value: str):
    fig.text(
        0.33, 0.957,
        f'{label} {value}',
        ha='left', va='top',
        fontsize=11,
        fontweight='bold',
        color=PLOT_THEME['text'],
        bbox=dict(
            boxstyle='round,pad=0.32',
            facecolor='#fff4e8',
            edgecolor=PLOT_THEME['pred'],
            linewidth=1.1,
        ),
    )


def plot_estimation_results(te_names, te_sizes, split_idx,
                            phase_test: np.ndarray,
                            phase_pred: np.ndarray,
                            history,
                            overall_err: float):
    configure_plot_style()

    n_test_files = len(te_names)
    height_ratios = []
    for _ in range(n_test_files):
        height_ratios.extend([2.2, 1.35])
    height_ratios.append(1.55)

    fig = plt.figure(
        figsize=(26, 3.5 * n_test_files + 3.2),
        facecolor=PLOT_THEME['figure_bg'],
    )
    gs = fig.add_gridspec(len(height_ratios), 1, height_ratios=height_ratios, hspace=0.34)
    axes = [fig.add_subplot(gs[idx, 0]) for idx in range(len(height_ratios))]

    add_figure_header(
        fig,
        'Gait Phase Estimation Overview',
        f'Test files: {n_test_files} | Sample rate: {SAMPLE_RATE} Hz',
    )
    add_header_metric(fig, 'Overall rRMSE =', f'{overall_err:.2f}%')

    for i, (name, size) in enumerate(zip(te_names, te_sizes)):
        s, e = split_idx[i], split_idx[i + 1]
        ph_true = phase_test[s:e]
        ph_pred = phase_pred[s:e]
        err_seq = circular_error(ph_pred, ph_true)
        err_i = rRMSE(ph_pred, ph_true)

        t_i = np.arange(size) / SAMPLE_RATE
        x_max = t_i[-1] if len(t_i) > 1 else max(1.0 / SAMPLE_RATE, 1.0)
        phase_true_pct = ph_true * 100
        phase_pred_pct = ph_pred * 100
        err_pct = err_seq * 100

        ax_phase = axes[i * 2]
        ax_err = axes[i * 2 + 1]

        style_axis(ax_phase, ylabel='Phase (%)')
        ax_phase.plot(
            t_i, phase_true_pct,
            label='Ground truth',
            color=PLOT_THEME['truth'],
            linewidth=1.8,
        )
        ax_phase.plot(
            t_i, phase_pred_pct,
            label='Prediction',
            color=PLOT_THEME['pred'],
            linewidth=1.6,
            linestyle=(0, (5, 2)),
        )
        ax_phase.fill_between(
            t_i, phase_true_pct, phase_pred_pct,
            color=PLOT_THEME['pred'],
            alpha=0.08,
        )
        ax_phase.set_xlim(0, x_max)
        ax_phase.set_ylim(-3, 103)
        ax_phase.set_title(
            f'{name} | Current Phase Estimation | rRMSE {err_i:.2f}%',
            loc='left',
            pad=10,
        )
        ax_phase.tick_params(labelbottom=False)
        ax_phase.xaxis.set_major_locator(MaxNLocator(nbins=7))
        if i == 0 or n_test_files == 1:
            ax_phase.legend(loc='upper left', fontsize=9, ncol=2)
        add_badge(
            ax_phase,
            f'rRMSE {err_i:.2f}%\nMAE {np.mean(np.abs(err_pct)):.1f}%',
            accent=PLOT_THEME['pred'],
        )

        style_axis(ax_err, xlabel='Time (s)', ylabel='Error (%)')
        ax_err.axhspan(-5, 5, color=PLOT_THEME['accent'], alpha=0.14, zorder=0)
        ax_err.fill_between(
            t_i, 0, err_pct,
            where=err_pct >= 0,
            color=PLOT_THEME['error_pos'],
            alpha=0.22,
            linewidth=0,
        )
        ax_err.fill_between(
            t_i, 0, err_pct,
            where=err_pct < 0,
            color=PLOT_THEME['error_neg'],
            alpha=0.18,
            linewidth=0,
        )
        ax_err.plot(t_i, err_pct, color=PLOT_THEME['error_pos'], linewidth=1.2)
        ax_err.axhline(0, color=PLOT_THEME['muted'], linestyle='--', linewidth=1.0)
        err_limit = max(8.0, float(np.percentile(np.abs(err_pct), 99) * 1.15))
        err_limit = np.ceil(err_limit / 5.0) * 5.0
        ax_err.set_xlim(0, x_max)
        ax_err.set_ylim(-err_limit, err_limit)
        ax_err.set_title(
            f'{name} | Circular Phase Error | rRMSE {err_i:.2f}%',
            loc='left',
            pad=10,
        )
        ax_err.xaxis.set_major_locator(MaxNLocator(nbins=7))
        add_badge(
            ax_err,
            f'rRMSE {err_i:.2f}%\nBias {np.mean(err_pct):+.1f}%\nStd {np.std(err_pct):.1f}%',
            accent=PLOT_THEME['error_pos'],
        )

    ax_train = axes[-1]
    train_loss = np.asarray(history.history['loss'], dtype=float)
    val_loss = np.asarray(history.history['val_loss'], dtype=float)
    epochs = np.arange(1, len(train_loss) + 1)
    train_smooth = smooth_curve(train_loss, window=5)
    val_smooth = smooth_curve(val_loss, window=5)
    best_idx = int(np.argmin(val_loss))

    style_axis(ax_train, xlabel='Epoch', ylabel='MSE Loss')
    ax_train.plot(epochs, train_loss, color=PLOT_THEME['truth'], linewidth=1.0, alpha=0.18)
    ax_train.plot(epochs, val_loss, color=PLOT_THEME['pred'], linewidth=1.0, alpha=0.18)
    ax_train.plot(epochs, train_smooth, color=PLOT_THEME['truth'], linewidth=2.2, label='Training loss')
    ax_train.plot(epochs, val_smooth, color=PLOT_THEME['pred'], linewidth=2.2, label='Validation loss')
    ax_train.scatter(
        epochs[best_idx], val_loss[best_idx],
        s=40,
        color=PLOT_THEME['success'],
        zorder=5,
        label='Best validation',
    )
    ax_train.set_xlim(1, max(2, len(epochs)))
    ax_train.set_title('Training Curve', loc='left', pad=10)
    ax_train.xaxis.set_major_locator(MaxNLocator(integer=True, nbins=8))
    ax_train.legend(loc='upper right', fontsize=9)
    add_badge(
        ax_train,
        f'Best val {val_loss[best_idx]:.4f}\nEpoch {epochs[best_idx]}',
        accent=PLOT_THEME['success'],
    )

    fig.subplots_adjust(top=0.90, left=0.06, right=0.99, bottom=0.06)
    fig_path = os.path.join(SAVE_DIR, 'gait_phase_result.png')
    fig.savefig(fig_path, dpi=220, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"\n结果图已保存至: {fig_path}")


def plot_cosine_results(te_names, te_sizes, split_idx,
                        phase_test: np.ndarray,
                        phase_pred: np.ndarray,
                        overall_err: float):
    configure_plot_style()

    n_test_files = len(te_names)
    fig, axes = plt.subplots(
        n_test_files, 1,
        figsize=(26, 2.8 * n_test_files + 1.6),
        facecolor=PLOT_THEME['figure_bg'],
    )
    axes = np.atleast_1d(axes)

    add_figure_header(
        fig,
        'Cosine View of Phase Prediction',
        f'Test files: {n_test_files} | Sample rate: {SAMPLE_RATE} Hz',
    )
    add_header_metric(fig, 'Overall rRMSE =', f'{overall_err:.2f}%')

    for i, (name, size) in enumerate(zip(te_names, te_sizes)):
        s, e = split_idx[i], split_idx[i + 1]
        ph_true = phase_test[s:e]
        ph_pred = phase_pred[s:e]
        err_i = rRMSE(ph_pred, ph_true)

        t_i = np.arange(size) / SAMPLE_RATE
        x_max = t_i[-1] if len(t_i) > 1 else max(1.0 / SAMPLE_RATE, 1.0)
        cos_true = np.cos(2 * np.pi * ph_true)
        cos_pred = np.cos(2 * np.pi * ph_pred)

        ax = axes[i]
        style_axis(ax, xlabel='Time (s)', ylabel='cos(2πφ)')
        ax.plot(t_i, cos_true, color=PLOT_THEME['truth'], linewidth=1.7, label='Ground truth')
        ax.plot(
            t_i, cos_pred,
            color=PLOT_THEME['pred'],
            linewidth=1.5,
            linestyle=(0, (5, 2)),
            label='Prediction',
        )
        ax.fill_between(t_i, cos_true, cos_pred, color=PLOT_THEME['pred'], alpha=0.08)
        ax.axhline(0, color=PLOT_THEME['muted'], linestyle='--', linewidth=0.9)
        ax.set_xlim(0, x_max)
        ax.set_ylim(-1.1, 1.1)
        ax.set_title(f'{name} | Cosine Projection', loc='left', pad=10)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=7))
        if i == 0 or n_test_files == 1:
            ax.legend(loc='upper right', fontsize=9, ncol=2)
        add_badge(ax, f'rRMSE {err_i:.2f}%', accent=PLOT_THEME['pred'])

    fig.subplots_adjust(top=0.84, left=0.06, right=0.99, bottom=0.08, hspace=0.36)
    fig2_path = os.path.join(SAVE_DIR, 'gait_phase_result_cos.png')
    fig.savefig(fig2_path, dpi=220, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"cos 对比图已保存至: {fig2_path}")


# ════════════════════════════════════════════════
# 4. 主流程
# ════════════════════════════════════════════════

def main():
    tf.random.set_seed(42)
    np.random.seed(42)

    # ── 4.1 加载已划分好的数据集 ─────────────────────────
    def load_split(directory):
        csv_files = sorted(glob.glob(os.path.join(directory, '*.csv')))
        Xs, Ys, phases, names = [], [], [], []
        for fpath in csv_files:
            X, Y, phase = load_file(fpath)
            Xs.append(X)
            Ys.append(Y)
            phases.append(phase)
            names.append(os.path.basename(fpath))
            print(f"  {os.path.basename(fpath):20s} → {X.shape[0]} 样本")
        return Xs, Ys, phases, names

    print(f"\n加载训练集（{TRAIN_DIR}）...")
    tr_X, tr_Y, tr_phase, _ = load_split(TRAIN_DIR)
    print(f"加载验证集（{VAL_DIR}）...")
    va_X, va_Y, va_phase, _ = load_split(VAL_DIR)
    print(f"加载测试集（{TEST_DIR}）...")
    te_X, te_Y, te_phase, te_names = load_split(TEST_DIR)

    X_train = np.vstack(tr_X)
    Y_train = np.vstack(tr_Y)
    X_val   = np.vstack(va_X)
    Y_val   = np.vstack(va_Y)
    X_test  = np.vstack(te_X)
    Y_test  = np.vstack(te_Y)
    phase_test = np.concatenate(te_phase)
    print(f"\n训练集: {X_train.shape}，验证集: {X_val.shape}，测试集: {X_test.shape}")

    # ── 4.3 特征标准化 ────────────────────────────────
    # 论文未明确提及，但标准化有助于收敛
    scaler = StandardScaler()
    X_train_sc = scaler.fit_transform(X_train)
    X_test_sc  = scaler.transform(X_test)
    joblib.dump(scaler, os.path.join(SAVE_DIR, 'feature_scaler.pkl'))
    print("特征标准化器已保存")

    # ── 4.4 训练网络 ──────────────────────────────────
    print("\n构建并训练 Encoder-Decoder ANN...")
    model = build_network()
    model.summary()

    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=LEARNING_RATE),
        loss='mse',
    )  # LEARNING_RATE=1e-3, BATCH_SIZE=64（由优化搜索确定）

    callbacks = [
        keras.callbacks.EarlyStopping(
            monitor='val_loss', patience=20,
            restore_best_weights=True, verbose=1
        ),
        keras.callbacks.ModelCheckpoint(
            os.path.join(SAVE_DIR, 'gait_phase_model.keras'),
            save_best_only=True, verbose=0
        ),
    ]

    X_val_sc = scaler.transform(X_val)

    history = model.fit(
        X_train_sc, Y_train,
        epochs=MAX_EPOCHS,
        batch_size=BATCH_SIZE,
        validation_data=(X_val_sc, Y_val),
        callbacks=callbacks,
        verbose=1,
    )

    # ── 4.5 评估 ──────────────────────────────────────
    print("\n===== 测试集评估 =====")
    Y_pred = model.predict(X_test_sc, batch_size=512)

    # 逐段施加相位约束（禁止回退 + 限制最大变化速率）
    te_sizes  = [len(p) for p in te_phase]
    split_idx = np.cumsum([0] + te_sizes)
    phase_pred_raw = recover_phase(Y_pred[:, :2])
    r_pred = Y_pred[:, 2]  # 网络预测的步频 (Hz)
    phase_pred = phase_pred_raw.copy()
    for _i in range(len(te_names)):
        _s, _e = split_idx[_i], split_idx[_i + 1]
        phase_pred[_s:_e] = apply_phase_rate_limit(
            phase_pred_raw[_s:_e], r_pred[_s:_e]
        )

    err0 = rRMSE(phase_pred, phase_test)
    plot_estimation_results(
        te_names=te_names,
        te_sizes=te_sizes,
        split_idx=split_idx,
        phase_test=phase_test,
        phase_pred=phase_pred,
        history=history,
        overall_err=err0,
    )

    plot_cosine_results(
        te_names=te_names,
        te_sizes=te_sizes,
        split_idx=split_idx,
        phase_test=phase_test,
        phase_pred=phase_pred,
        overall_err=err0,
    )

    result_df = pd.DataFrame({
        'phase_true': phase_test,
        'phase_pred': phase_pred,
        'error_pct': circular_error(phase_pred, phase_test) * 100,
    })
    csv_out = os.path.join(SAVE_DIR, 'prediction_results.csv')
    result_df.to_csv(csv_out, index=False)
    print(f"预测结果已保存至: {csv_out}")

    return model, scaler, err0
    print(f"rRMSE₀  (当前相位估计):    {err0:.2f}%")
    print(f"（论文参考值：rRMSE₀=6.3%）")

    # ── 4.6 可视化：每个测试文件单独画一组子图 ──────────────
    # te_sizes / split_idx 已在上方计算
    n_test_files = len(te_names)

    n_rows = n_test_files * 2 + 1  # 每个文件 2 行（相位 + 误差）+ 1 行训练曲线
    fig, axes = plt.subplots(n_rows, 1, figsize=(40, 4 * n_test_files + 5))
    fig.suptitle('Gait Phase Estimation Results', fontsize=14)

    for i, (name, size) in enumerate(zip(te_names, te_sizes)):
        s, e = split_idx[i], split_idx[i + 1]
        ph_true = phase_test[s:e]
        ph_pred = phase_pred[s:e]

        err_seq_i = circular_error(ph_pred, ph_true)
        err_i     = rRMSE(ph_pred, ph_true)

        N_SHOW = size
        t_i = np.arange(N_SHOW) / SAMPLE_RATE

        ax_phase = axes[i * 2]
        ax_err   = axes[i * 2 + 1]

        ax_phase.plot(t_i, ph_true[:N_SHOW] * 100,
                      label='ground truth', color='steelblue', alpha=0.9, linewidth=1.5)
        ax_phase.plot(t_i, ph_pred[:N_SHOW] * 100, '--',
                      label=f'predicted (rRMSE₀={err_i:.1f}%)',
                      color='orange', alpha=0.85, linewidth=1.5)
        ax_phase.set_ylabel('Phase (%)')
        ax_phase.set_xlabel('Time (s)')
        ax_phase.set_title(f'[{name}] Current Gait Phase Estimation  (rRMSE₀={err_i:.2f}%)')
        ax_phase.legend(loc='upper right', fontsize=8)
        ax_phase.grid(True, alpha=0.4)

        ax_err.plot(t_i, err_seq_i[:N_SHOW] * 100,
                    color='tomato', alpha=0.8, linewidth=1)
        ax_err.axhline(0, color='k', linestyle='--', linewidth=0.8)
        ax_err.fill_between(t_i, err_seq_i[:N_SHOW] * 100, alpha=0.2, color='tomato')
        ax_err.set_ylabel('Error (%)')
        ax_err.set_xlabel('Time (s)')
        ax_err.set_title(f'[{name}] Phase Estimation Error (rRMSE₀={err_i:.2f}%)')
        ax_err.grid(True, alpha=0.4)

    # 训练曲线（最后一行）
    ax_train = axes[-1]
    ax_train.plot(history.history['loss'],     label='Training Loss', linewidth=1.5)
    ax_train.plot(history.history['val_loss'], label='Validation Loss', linewidth=1.5)
    ax_train.set_ylabel('MSE Loss')
    ax_train.set_xlabel('Epoch')
    ax_train.set_title('Training Process')
    ax_train.legend()
    ax_train.grid(True, alpha=0.4)

    plt.tight_layout()
    fig_path = os.path.join(SAVE_DIR, 'gait_phase_result.png')
    plt.savefig(fig_path, dpi=150)
    print(f"\n结果图已保存至: {fig_path}")
    plt.close()

    # ── 4.7 cos(2πφ) 对比图 ──────────────────────────────
    fig2, axes2 = plt.subplots(n_test_files, 1, figsize=(40, 4 * n_test_files))
    if n_test_files == 1:
        axes2 = [axes2]
    fig2.suptitle('Gait Phase Estimation — cos(2πφ) View', fontsize=14)

    for i, (name, size) in enumerate(zip(te_names, te_sizes)):
        s, e = split_idx[i], split_idx[i + 1]
        ph_true = phase_test[s:e]
        ph_pred = phase_pred[s:e]
        err_i   = rRMSE(ph_pred, ph_true)

        N_SHOW = size
        t_i = np.arange(N_SHOW) / SAMPLE_RATE

        cos_true = np.cos(2 * np.pi * ph_true[:N_SHOW])
        cos_pred = np.cos(2 * np.pi * ph_pred[:N_SHOW])

        ax = axes2[i]
        ax.plot(t_i, cos_true, label='ground truth',
                color='steelblue', alpha=0.9, linewidth=1.5)
        ax.plot(t_i, cos_pred, '--',
                label=f'predicted (rRMSE₀={err_i:.1f}%)',
                color='orange', alpha=0.85, linewidth=1.5)
        ax.set_ylabel('cos(2πφ)')
        ax.set_xlabel('Time (s)')
        ax.set_title(f'[{name}]  cos(2πφ)  (rRMSE₀={err_i:.2f}%)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.4)

    plt.tight_layout()
    fig2_path = os.path.join(SAVE_DIR, 'gait_phase_result_cos.png')
    plt.savefig(fig2_path, dpi=150)
    print(f"cos 对比图已保存至: {fig2_path}")
    plt.close()

    # 保存预测结果到 CSV（便于后续分析）
    result_df = pd.DataFrame({
        'phase_true': phase_test,
        'phase_pred': phase_pred,
        'error_pct':  circular_error(phase_pred, phase_test) * 100,
    })
    csv_out = os.path.join(SAVE_DIR, 'prediction_results.csv')
    result_df.to_csv(csv_out, index=False)
    print(f"预测结果已保存至: {csv_out}")

    return model, scaler, err0


# ════════════════════════════════════════════════
# 5. 推理接口（用于新数据/实时调用）
# ════════════════════════════════════════════════

def predict_from_file(csv_path: str,
                      model_path: str = None,
                      scaler_path: str = None):
    """
    对新的数据文件进行步态相位推理，无需重新训练。

    参数:
        csv_path:   新采集的 CSV 文件路径
        model_path: 已训练模型路径（默认使用 SAVE_DIR 下的模型）
        scaler_path: 标准化器路径
    """

    model_path  = model_path  or os.path.join(SAVE_DIR, 'gait_phase_model.keras')
    scaler_path = scaler_path or os.path.join(SAVE_DIR, 'feature_scaler.pkl')

    model  = keras.models.load_model(model_path)
    scaler = joblib.load(scaler_path)

    X, _, phase_true = load_file(csv_path)
    X_sc   = scaler.transform(X)
    Y_pred = model.predict(X_sc, batch_size=512)

    phase_pred = recover_phase(Y_pred[:, :2])
    r_pred     = Y_pred[:, 2]

    err = rRMSE(phase_pred, phase_true)
    print(f"文件: {os.path.basename(csv_path)}")
    print(f"rRMSE₀ = {err:.2f}%")

    return phase_pred, r_pred, phase_true


if __name__ == '__main__':
    model, scaler, err0 = main()
