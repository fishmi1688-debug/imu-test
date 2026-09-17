# 步态相位估计目录说明

这个目录当前包含训练/离线推理、实时 UI 推理脚本和已经导出的模型工件，用于基于双脚 IMU 数据估计左腿步态相位。

## 文件概览

| 文件 | 作用 | 备注 |
| --- | --- | --- |
| `gait_phase_estimation.py` | 主脚本，负责特征提取、模型训练、测试评估、结果绘图，以及离线推理接口 | 既能训练，也能加载模型做推理 |
| `multi_imu_gait_phase_live.py` | 双脚 IMU + 鞋垫实时采集、Keras 模型预测和 Tk UI 显示 | 默认使用 `gait_phase_model.keras` |
| `gait_phase_model.keras` | 已训练好的 Keras 模型 | HDF5 格式，模型名为 `GaitPhaseNet` |
| `feature_scaler.pkl` | 已保存的特征标准化参数 | 支持 `StandardScaler` 或 `mean/scale/n_features_in` 字典格式 |

当前文件大小大致为：

- `gait_phase_estimation.py`：34 KB
- `gait_phase_model.keras`：42 KB
- `feature_scaler.pkl`：2.0 KB

## 脚本做了什么

### 1. 输入与特征

脚本假设输入是 CSV 文件，并至少包含以下列：

- `time`
- `gait_phase_left`
- `left_imu_Euler_Y`
- `left_imu_Euler_X`
- `left_imu_Gyr_X`
- `left_imu_Gyr_Y`
- `left_imu_Gyr_Z`
- `left_imu_Acc_X`
- `left_imu_Acc_Y`
- `left_imu_Acc_Z`

`time` 支持三类格式：

- 带时区的标准时间戳
- 纯秒数
- `MM:SS.s` / `HH:MM:SS.s` 形式

脚本使用长度为 `27` 的滑动窗口，在 `30 Hz` 采样率下相当于 `900 ms`。对每个通道提取 7 个统计特征：

- 最大值
- 最小值
- 均值
- 标准差
- 首值
- 中值
- 末值

当前真正启用的 IMU 通道数是 `10` 个，因此输入维度是：

`10 通道 × 7 特征 = 70 维`

### 2. 标签设计

标签不是直接回归相位，而是回归：

- `cos(2πφ)`
- `sin(2πφ)`
- 步频 `r`

这样做的目的是避免步态相位在 `0` 和 `1` 附近的跳变不连续问题。步频通过检测相位从高值跳回低值的事件来估计。

### 3. 网络结构

从模型文件和脚本两边看，当前模型结构一致：

- 输入层：`70`
- 隐藏层 1：`Dense(10, relu)`
- 隐藏层 2：`Dense(20, relu)`
- 输出层：`Dense(3, linear)`

训练配置保存在模型文件中：

- 后端：`tensorflow`
- Keras 版本：`2.10.0`
- 损失函数：`mse`
- 优化器：`Adam`
- 学习率：约 `5e-4`

### 4. 训练与评估流程

执行 `main()` 时，脚本会：

1. 从 `data/train`、`data/val`、`data/test` 读取 CSV。
2. 提取 70 维窗口特征。
3. 用 `StandardScaler` 标准化。
4. 训练全连接回归网络。
5. 在测试集上恢复相位，并根据预测的步频做相位速率约束。
6. 输出图像和 CSV 结果。

训练输出默认写入 `model_output/`：

- `model_output/gait_phase_model.keras`
- `model_output/feature_scaler.pkl`
- `model_output/gait_phase_result.png`
- `model_output/gait_phase_result_cos.png`
- `model_output/prediction_results.csv`

### 5. 推理接口

脚本提供 `predict_from_file(csv_path, model_path=None, scaler_path=None)` 用于离线推理。

推理流程是：

1. 加载模型和 scaler。
2. 对新 CSV 做同样的窗口特征提取。
3. 预测 `[cos, sin, r]`。
4. 把 `[cos, sin]` 还原成步态相位。

返回值是：

- `phase_pred`
- `r_pred`
- `phase_true`

## 两个工件的实际内容

### `gait_phase_model.keras`

这是一个已经训练好的 Keras 模型文件，内部除了网络权重，还包含：

- 模型结构定义
- 优化器状态
- 训练配置

可确认到的关键信息：

- 模型名：`GaitPhaseNet`
- 输入形状：`[None, 70]`
- 层结构：`70 -> 10 -> 20 -> 3`

### `feature_scaler.pkl`

当前目录中的 `feature_scaler.pkl` 是轻量字典格式，包含训练好的标准化统计量：

- `n_features_in = 70`
- `mean` 形状：`(70,)`
- `scale` 形状：`(70,)`

这说明它和当前模型是配套的，输入必须仍然是同样的 70 维特征顺序。实时脚本会兼容这种字典格式和标准 `StandardScaler` 对象格式。

## 运行方式

### 训练

前提是目录下存在如下数据结构：

```text
data/
  train/
  val/
  test/
```

运行：

```bash
python3 gait_phase_estimation.py
```

### 使用当前目录中的已训练工件做推理

因为 `predict_from_file()` 默认去 `model_output/` 下找模型和 scaler，而当前目录里的两个工件放在根目录，所以建议显式传路径：

```python
from gait_phase_estimation import predict_from_file

phase_pred, r_pred, phase_true = predict_from_file(
    "your_data.csv",
    model_path="gait_phase_model.keras",
    scaler_path="feature_scaler.pkl",
)
```

### 实时预测和 UI 显示

默认会加载当前目录下的 `gait_phase_model.keras` 和 `feature_scaler.pkl`：

```bash
python3 multi_imu_gait_phase_live.py
```

无蓝牙设备时可以用已有 CSV 做终端回放验证：

```bash
python3 multi_imu_gait_phase_live.py \
  --headless \
  --replay-left-foot data/20260402_130221/left_foot_imu.csv \
  --replay-right-foot data/20260402_130221/right_foot_imu.csv \
  --replay-speed 0
```

## 代码层面的注意事项

### 1. 注释与实际输入维度不完全一致

脚本里早期版本曾使用不同通道数量；当前 `IMU_CHANNELS` 配置实际启用的是 `10` 个通道，所以真实输入维度是 `70`。

### 2. `predict_from_file()` 与测试评估流程不完全一致

在测试评估里，预测相位之后还会调用 `apply_phase_rate_limit()` 做后处理；而 `predict_from_file()` 当前直接返回恢复后的相位，没有应用这一步。因此：

- 训练脚本里的测试结果
- 推理接口直接返回的结果

两者可能存在轻微差异。

### 3. `main()` 中保留了一段不可达旧代码

`main()` 在保存结果后已经执行了 `return model, scaler, err0`，后面还残留一整段旧版绘图和保存逻辑，但实际上不会运行。这不影响当前结果，只是说明脚本里还有历史代码未清理。

## 依赖

脚本依赖以下 Python 库：

- `numpy`
- `pandas`
- `matplotlib`
- `scikit-learn`
- `joblib`
- `tensorflow` / `keras`
- `h5py`
- `tkinter`（实时 UI）

## 总结

这个目录可以理解为“一个完整的步态相位估计最小交付单元”：

- `gait_phase_estimation.py` 是训练和离线推理入口
- `multi_imu_gait_phase_live.py` 是实时预测和 UI 入口
- `gait_phase_model.keras` 是训练好的神经网络
- `feature_scaler.pkl` 是与该模型严格配套的输入标准化器

如果只想复用现有模型做推理，这三个文件已经足够；如果要重新训练，还需要补齐 `data/train`、`data/val`、`data/test` 目录及对应 CSV 数据。
