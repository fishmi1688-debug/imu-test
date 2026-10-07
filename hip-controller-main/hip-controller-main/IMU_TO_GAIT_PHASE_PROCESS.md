# IMU 原始数据到步态相位输出的完整过程

本文档说明本项目中 **IMU 原始髋关节角度/角速度** 如何进入控制器，并经过预处理、步态事件检测、相平面归一化，最终输出 gait phase（步态相位）。文中所说的“原始 IMU 数据”指进入本仓库控制器之前已经由上游 IMU 模块整理好的髋角度与髋角速度，而不是加速度计、陀螺仪、磁力计的最低层融合过程。

## 1. 代码范围说明

本仓库的核心控制器从 `SensorSignal` 开始工作：

```python
SensorSignal(
    timestamp=...,
    angle_rad=...,
    velocity_rad_per_sec=...,
)
```

也就是说，本仓库不直接实现 IMU 姿态融合算法，例如从 accelerometer/gyroscope/magnetometer 融合出 quaternion 的过程。它假设上游已经提供：

- `angle_rad`：髋关节角度，单位 rad。
- `velocity_rad_per_sec`：髋关节角速度，单位 rad/s。
- `timestamp`：当前采样时间，单位 s。

README 中给出的在线使用方式是：

```python
signal_left = SensorSignal(
    timestamp=timestamp_left,
    angle_rad=data_left.quat.to_euler(seq="xyz").z,
    velocity_rad_per_sec=data_left.device_data.gyro.z,
)

signal_right = SensorSignal(
    timestamp=timestamp_right,
    angle_rad=data_right.quat.to_euler(seq="xyz").z,
    velocity_rad_per_sec=data_right.device_data.gyro.z,
)
```

因此在线系统中的数据来源可以理解为：

```text
IMU 低层数据
 -> 上游姿态融合得到 quaternion
 -> quaternion.to_euler(seq="xyz")
 -> 取 z 分量作为 raw hip angle
 -> 取 gyro.z 作为 raw hip angular velocity
 -> 封装成 SensorSignal
 -> 送入 WalkOnController.step()
```

离线 CSV 回放时，`CSVPlayer` 读取 CSV 中的列：

- `angle_left (rad)`
- `angle_right (rad)`
- `vel_left (rad/s)`
- `vel_right (rad/s)`

然后同样封装为 `SensorSignal`，再送入控制器。

相关代码：

- `src/hip_controller/definitions.py`：`SensorSignal`
- `src/hip_controller/plotter/csv_player.py`：`CSVPlayer`
- `src/hip_controller/control/app.py`：`WalkOnController`

## 2. 左右腿是否分别估计相位

是的，左右腿分别估计步态相位。

通常会创建两个独立控制器：

```python
controller_left = WalkOnController(left_limb=True, config=config)
controller_right = WalkOnController(left_limb=False, config=config)
```

每个 `WalkOnController` 内部都会各自创建：

```text
SensorPreprocessor
GaitController
AmplitudeModulation
MotionReferenceController
```

因此左腿和右腿的预处理状态、极值检测状态、stride event 状态、相平面中心值、相位输出都是独立的。

整体结构是：

```text
left IMU signal
 -> left SensorPreprocessor
 -> left GaitController
 -> left gait phase

right IMU signal
 -> right SensorPreprocessor
 -> right GaitController
 -> right gait phase
```

## 3. 总体数据流

从 IMU 输入到 gait phase 输出的主流程如下：

```text
SensorSignal(raw angle, raw velocity, timestamp)
 -> WalkOnController.step()
 -> 静止/暂停检测
 -> SensorPreprocessor.filter()
    -> baseline removal
    -> drift removal
    -> angle filtering
    -> velocity estimation
    -> optional velocity drift removal
 -> GaitController.update_and_compute()
    -> MotionStateMachine.update_motion_state()
    -> SteadyStateTracker.update_extrema()
    -> StrideEventDetector.is_valid_stride_event()
    -> SteadyStateTracker.recenter()
    -> SteadyStateTracker.update_steady_state()
    -> atan2(velocity_steady_state, angle_steady_state)
 -> gait_phase_rad
```

在 `WalkOnController.step()` 中，相位计算对应这段逻辑：

```python
if self.filtered:
    filtered_signal = curr_signal
else:
    filtered_signal = self.pre_processor.filter(raw_signal=curr_signal)

gait_phase = self.gait_controller.update_and_compute(
    curr_signal=filtered_signal
)
```

`filtered=True` 表示输入信号已经在外部处理好，控制器跳过内部预处理；默认 `filtered=False`，即使用本项目的预处理链路。

## 4. 第一步：WalkOnController 接收原始信号

主入口是：

```python
WalkOnController.step(curr_signal: SensorSignal) -> float
```

虽然 `step()` 最终返回的是 motor command，但其中会保存最近一次相位：

```python
self.last_gait_phase_rad = gait_phase
```

所以从“IMU 到相位”的角度看，`step()` 是总调度函数。

输入信号结构：

```python
@dataclass
class SensorSignal:
    timestamp: float | None
    angle_rad: float = 0.0
    velocity_rad_per_sec: float = 0.0
```

这里的 `angle_rad` 是原始髋关节角度，`velocity_rad_per_sec` 是原始髋关节角速度。

## 5. 第二步：静止/暂停检测

在预处理之前，`WalkOnController.step()` 会根据原始角速度判断用户是否仍在走路。

核心思想是对 `abs(raw velocity)` 做指数滑动平均：

```text
envelope[k] =
    (1 - alpha) * envelope[k-1]
    + alpha * abs(raw_velocity[k])
```

默认参数：

```python
PAUSE_DETECT_ENVELOPE_ALPHA = 0.10
PAUSE_ENTER_THRESHOLD = 0.2
PAUSE_EXIT_THRESHOLD = 0.5
```

逻辑：

```text
如果当前处于 walking 状态，且 envelope < 0.2 rad/s
 -> 判定为 paused
 -> pre_processor.set_walking_mode(False)

如果当前处于 paused 状态，且 envelope > 0.5 rad/s
 -> 判定为 walking
 -> pre_processor.set_walking_mode(True)
```

这个状态主要影响 SOGI-FLL：

- 站立时冻结 FLL 频率估计，避免噪声让步频估计漂移。
- 恢复行走时保留之前的步频估计，使 SOGI 更快重新锁定。

## 6. 第三步：SensorPreprocessor 预处理

预处理器入口是：

```python
SensorPreprocessor.filter(raw_signal: SensorSignal) -> SensorSignal
```

默认预处理链路：

```text
raw angle
 -> baseline removal
 -> drift removal
 -> SOGI-FLL angle filtering
 -> SOGI quadrature velocity estimation
 -> velocity drift removal
 -> filtered SensorSignal
```

输出仍然是 `SensorSignal`：

```python
SensorSignal(
    timestamp=raw_signal.timestamp,
    angle_rad=angle_out_rad,
    velocity_rad_per_sec=velocity_out_rad_per_sec,
)
```

这里的 `angle_rad` 已经不是最原始的 IMU 角度，而是控制器后续用于相位估计的预处理角度；`velocity_rad_per_sec` 也是预处理后的速度或速度代理。

### 6.1 Baseline removal：去安装零偏

相关代码：

- `control/signal_processing/baseline_removal.py`
- `BaselineRemoval`

IMU 绑在人体上时会有安装角度偏差，所以 raw angle 往往带有 DC offset。该模块通过主开关触发一个短时间窗口，取站立静止时的平均角度作为 offset：

```text
offset = mean(still_angle_samples)
corrected_angle = raw_angle - offset
```

窗口长度来自配置：

```python
BASELINE_REMOVAL_WINDOW_S = 0.2
BASELINE_REMOVAL_MAX_VELOCITY_RAD_PER_SEC = 0.2
```

注意点：

- offset 不是启动时自动取的，而是由 `set_baseline_removal_trigger(active=True)` 触发。
- 如果窗口内检测到速度过大，说明腿在动，会清空当前窗口重新采样。
- offset 没有采好之前，该模块等价于直接通过原始角度。

### 6.2 时间差 dt 检查

预处理器保存上一次 timestamp，计算：

```text
dt = current_timestamp - previous_timestamp
```

如果 `dt <= 0`，直接报错，因为时间不能倒退或停止。

如果 `dt > 1.0s`，认为发生了数据 dropout，不把这个巨大 dt 当作真实采样间隔，而是：

- 重置 drift removal。
- 重置 velocity drift removal。
- 重置非 SOGI 的 velocity estimator。
- 使用名义时间差 `0.01s` 继续。

这样可以避免长时间断流后下一次积分产生异常尖峰。

### 6.3 Drift removal：去慢漂移

相关代码：

- `control/signal_processing/drift_removal.py`
- `LowPassDriftRemoval`
- `NotchDriftRemoval`

默认方式是 `LowPassDriftRemoval`。

它用一个低截止频率二阶低通估计慢漂移：

```text
drift_estimate = LPF(raw_angle)
angle_no_drift = raw_angle - drift_estimate
```

也可以用 `NotchDriftRemoval`，通过 notch filter 去掉特定频段/0Hz 附近的漂移。

### 6.4 Angle filtering：角度滤波

相关代码：

- `control/signal_processing/filtering.py`
- `SogiFllFiltering`
- `LowPassFiltering`
- `KalmanFiltering`

由 `BasicConfig.filtering_method` 决定使用哪种方法。默认是：

```python
FilteringMethod.SOGI
```

因此默认会调用 SOGI-FLL：

```text
angle_no_drift
 -> SOGI-FLL
 -> angle_surrogate
 -> quadrature
```

其中：

- `angle_surrogate` 是平滑后的角度。
- `quadrature` 是与角度相差 90 度的分量，在本项目中默认作为速度代理使用。

### 6.5 SOGI-FLL 的作用

相关代码：

- `filters/sogi_fll_filter.py`
- `SogiFllFilter`

SOGI-FLL 可以理解为一个会自适应步频的振荡器：

```text
输入：去漂移后的髋角度
输出：
  inphase    -> 平滑角度
  quadrature -> 速度代理/正交分量
  frequency  -> 估计出来的步频
```

它内部有两部分：

1. SOGI core：生成 in-phase 和 quadrature。
2. FLL：根据相位误差和信号能量调整内部频率，使振荡器锁定当前步态频率。

为了让真实使用更稳定，代码中加入了几个状态保护：

- `stop_walking()`：暂停行走时冻结频率估计。
- `start_walking()`：恢复行走时清空 oscillator 状态，但保留之前频率，并短暂禁止 FLL 适应。
- `set_config()`：切换楼梯/平地/坡道模式时换 SOGI 参数，并让 FLL 冷却几十个采样点。
- `clear_state_keep_frequency()`：从下楼模式切到非下楼模式时，清空 SOGI 状态但保留频率。

### 6.6 Velocity estimation：速度估计

相关代码：

- `control/signal_processing/velocity_estimation.py`

默认速度估计方式是：

```python
VelocityEstimationMethod.SOGI
```

也就是说，不再另外做差分，而是直接使用 SOGI-FLL 的 quadrature：

```text
velocity_pre_drift = SOGI quadrature
```

项目也支持其他速度估计方式：

- `DISCRETE_DERIVATIVE`：对角度做离散差分。
- `LOW_PASS`：角度进二阶低通，同时取低通内部导数。
- `GYROSCOPE`：直接使用 raw gyroscope velocity。

默认 SOGI 路径的好处是角度和速度来自同一个自适应振荡器，相位关系更一致。

### 6.7 Velocity drift removal：速度去 DC 漂移

速度估计之后，默认还会经过一个 notch drift removal：

```text
velocity_out = notch(velocity_pre_drift)
```

这是为了消除速度信号中的 DC 偏置，避免后续相平面中心和 gait phase 受到低频偏移影响。

## 7. 第四步：GaitController 估计步态相位

相关代码：

- `control/gait_phase_control/gait_controller.py`
- `GaitController.update_and_compute()`

入口：

```python
gait_phase = self.gait_controller.update_and_compute(
    curr_signal=filtered_signal
)
```

这里输入的是预处理后的 `SensorSignal`：

```text
filtered angle
filtered velocity
timestamp
```

`GaitController` 内部维护：

```text
prev_signal
curr_signal
MotionStateMachine
StrideEventDetector
SteadyStateTracker
controller_initialized
```

如果当前或上一个 timestamp 为空，直接返回 `0.0`。这通常发生在第一帧。

## 8. 第五步：MotionStateMachine 检测极值

相关代码：

- `control/gait_phase_control/motion_state_machine.py`
- `MotionStateMachine`
- `ExtremaTrigger`

步态信号近似周期运动，因此可以通过角度和速度的过零关系检测四类极值：

```text
angle max：velocity 从正到负过零，且当前 angle > 0
angle min：velocity 从负到正过零，且当前 angle < 0
velocity max：angle 从负到正过零，且当前 velocity > 0
velocity min：angle 从正到负过零，且当前 velocity < 0
```

代码中的状态顺序是：

```text
VELOCITY_MAX
 -> ANGLE_MAX
 -> VELOCITY_MIN
 -> ANGLE_MIN
 -> VELOCITY_MAX
 -> ...
```

这样做的目的不是检测所有局部波动，而是要求极值按照人体周期运动的合理顺序出现，从而减少噪声引起的小峰值误检。

状态机还有 `tmin/tmax` 时间约束：

- 在太短时间内不接受新状态，避免抖动。
- 如果太久没有等到下一个合理极值，就回到 `INITIAL`，重新同步。

## 9. 第六步：SteadyStateTracker 记录极值并归一化相平面

相关代码：

- `control/gait_phase_control/steady_state_tracker.py`
- `SteadyStateTracker`

当状态机检测到新的有效极值时：

```python
self.steady_state_tracker.update_extrema(
    state=state,
    curr_signal=self.curr_signal,
)
```

它会更新四个极值：

```text
angle_max
angle_min
velocity_max
velocity_min
```

这些极值用于计算：

```text
center_ang = (angle_max + angle_min) / 2
center_vel = (velocity_max + velocity_min) / 2
scale_factor = abs(velocity_max - velocity_min) / abs(angle_max - angle_min)
```

然后每个采样点都会被转换成相平面上的中心化、缩放值：

```text
vel_steady_state = velocity - center_vel

ang_steady_state =
    - (angle - center_ang) * scale_factor
```

这里角度前面有一个负号，是为了让相平面的旋转方向和后续控制映射一致。

## 10. 第七步：StrideEventDetector 决定何时重新居中

相关代码：

- `control/gait_phase_control/stride_event_detector.py`
- `StrideEventDetector`

只检测极值还不够，因为一开始中心值和缩放因子可能还没稳定。因此代码会等待一个有效 stride event。

Stride event 的核心条件是：

```text
velocity 从上往下穿过 offset = -0.1
```

并结合 `valid_ang_max` 和计时窗口控制。

当检测到新的有效 stride event 上升沿时：

```python
self.steady_state_tracker.recenter()
```

`recenter()` 会用当前保存的四个极值重新计算中心值和 scale factor。

在检测到第一个有效 stride 之前：

```python
if not self.controller_initialized:
    return 0.0
```

因此控制器刚开始时 gait phase 固定为 `0.0`，直到识别出有效步态事件后才开始输出真实相位。

## 11. 第八步：atan2 输出 gait phase

最终相位由 `GaitController.calculate_gait_phase()` 计算：

```python
return atan2(
    self.steady_state_tracker.vel_steady_state,
    self.steady_state_tracker.ang_steady_state,
)
```

数学意义是：把当前步态状态看作相平面上的一个点：

```text
x = ang_steady_state
y = vel_steady_state
```

则：

```text
gait_phase = atan2(y, x)
```

输出范围是：

```text
[-pi, pi]
```

这就是后续电机参考控制使用的步态相位。

## 12. 相位输出后的使用方式

虽然本文重点是“IMU 到相位”，但在实际控制中 `gait_phase` 会继续送入电机参考控制：

```text
gait_phase
 -> transform_to_cyclic(): -sin(gait_phase + LAG_COMPENSATION)
 -> CubicSpline lookup table
 -> 乘 amplitude modulation
 -> motor command low-pass filter
 -> position/velocity limitation
 -> motor_command
```

相关代码：

- `control/motor_reference_control/motor_reference_controller.py`
- `MotionReferenceController.compute_motor_command()`
- `MotionMapping`

## 13. 关键类之间的职责划分

### SensorSignal

文件：`definitions.py`

职责：

- 统一表示一帧传感器数据。
- 包含时间、角度、角速度。

### WalkOnController

文件：`control/app.py`

职责：

- 单腿控制器总调度。
- 负责静止检测、预处理、相位估计、幅值调制、电机命令。
- 每条腿一个实例。

### SensorPreprocessor

文件：`control/signal_processing/sensor_preprocessor.py`

职责：

- 将 raw `SensorSignal` 变成适合相位估计的 filtered `SensorSignal`。
- 内部采用策略模式组合 drift removal、angle filtering、velocity estimation。

### BaselineRemoval

文件：`control/signal_processing/baseline_removal.py`

职责：

- 主开关触发采样静止角度。
- 去除安装零偏。

### DriftRemovalStrategy

文件：`control/signal_processing/drift_removal.py`

职责：

- 去除慢漂移。
- 当前实现包括低通减法和 notch filter。

### SogiFllFilter

文件：`filters/sogi_fll_filter.py`

职责：

- 从髋角度中提取平滑角度和正交速度代理。
- 自适应估计步态频率。
- 在暂停、恢复和模式切换时保持稳定。

### GaitController

文件：`control/gait_phase_control/gait_controller.py`

职责：

- 将预处理后的角度/速度转换为 gait phase。
- 组合状态机、stride event detector 和 steady-state tracker。

### MotionStateMachine

文件：`control/gait_phase_control/motion_state_machine.py`

职责：

- 按合理周期顺序检测角度/速度极值。
- 抑制错误极值和噪声触发。

### SteadyStateTracker

文件：`control/gait_phase_control/steady_state_tracker.py`

职责：

- 保存 angle/velocity 的最大最小值。
- 计算中心值与缩放因子。
- 生成相平面中的 `ang_steady_state` 和 `vel_steady_state`。

### StrideEventDetector

文件：`control/gait_phase_control/stride_event_detector.py`

职责：

- 判断何时出现有效 stride。
- 触发 `recenter()`。
- 在初始化阶段阻止过早输出错误相位。

## 14. 设计特点总结

1. **IMU 姿态融合在仓库外部完成**

   本仓库接收的是 `angle_rad` 和 `velocity_rad_per_sec`，不是最低层 IMU 原始三轴数据。

2. **左右腿完全独立估计相位**

   每条腿有自己的 `WalkOnController` 和 `GaitController`，相位状态不共享。

3. **默认使用 SOGI-FLL 处理周期步态信号**

   SOGI-FLL 同时产生平滑角度、速度代理和频率估计，很适合周期性步态信号。

4. **相位来自相平面，而不是简单时间积分**

   相位通过 `atan2(velocity_steady_state, angle_steady_state)` 得到，因此能跟随真实运动状态变化。

5. **需要有效 stride 后才输出真实相位**

   初始化阶段返回 `0.0`，避免极值和中心值尚未稳定时产生错误相位。

6. **使用策略模式便于替换算法**

   drift removal、angle filtering、velocity estimation 都可通过配置切换。

7. **有多层抗异常机制**

   包括 baseline 静止采样、timestamp 检查、dropout 处理、pause detection、FLL cooldown、velocity drift removal 等。

## 15. 简化伪代码

```python
def step(raw_signal):
    update_pause_detector(raw_signal.velocity_rad_per_sec)

    if config.filtered:
        signal = raw_signal
    else:
        signal = preprocessor.filter(raw_signal)

    gait_phase = gait_controller.update_and_compute(signal)
    return gait_phase


def preprocessor_filter(raw_signal):
    angle = baseline_removal.apply(
        raw_signal.angle_rad,
        raw_signal.velocity_rad_per_sec,
    )

    dt = raw_signal.timestamp - prev_timestamp
    check_dt_or_handle_dropout(dt)

    angle_no_drift = drift_removal.filter(angle, dt)
    angle_filtered = angle_filtering.filter(angle_no_drift, dt)

    if velocity_method == SOGI:
        velocity = sogi_filter.last_quadrature
    else:
        velocity = velocity_estimator.filter(...)

    velocity = velocity_drift_removal.filter(velocity, dt)

    return SensorSignal(
        timestamp=raw_signal.timestamp,
        angle_rad=angle_filtered,
        velocity_rad_per_sec=velocity,
    )


def gait_update_and_compute(signal):
    prev_signal = curr_signal
    curr_signal = signal

    state = motion_state_machine.update_motion_state(prev_signal, curr_signal)

    if state is not None:
        steady_state_tracker.update_extrema(state, curr_signal)

    stride_event = stride_event_detector.is_valid_stride_event(...)

    if rising_edge(stride_event):
        steady_state_tracker.recenter()

    steady_state_tracker.update_steady_state(curr_signal)

    if not stride_event_detector.valid_stride:
        return 0.0

    return atan2(
        steady_state_tracker.vel_steady_state,
        steady_state_tracker.ang_steady_state,
    )
```

## 16. 最终一句话总结

本项目的相位估计流程可以概括为：

```text
上游 IMU 姿态输出的髋角度/角速度
 -> 去零偏、去漂移、SOGI-FLL 滤波
 -> 检测周期极值与 stride event
 -> 用极值更新相平面中心和尺度
 -> atan2(归一化速度, 归一化角度)
 -> 输出单腿 gait phase
```


