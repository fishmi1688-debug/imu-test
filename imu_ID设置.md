# MI1/HiPNUC IMU CAN 参数与 ID 设置

目标配置：

- CAN 波特率：`1 Mbit/s`
- 左脚 IMU 节点 ID：`0x01`
- 右脚 IMU 节点 ID：`0x02`
- 输出频率：`50 Hz`，即周期 `20 ms`
- 保留数据：时间、加速度、角速度、四元数
- 关闭数据：磁场、欧拉角、航向角、温度、倾角仪、CANFD83

> 下面命令按 HiPNUC J1939 配置协议整理。默认假设新 IMU 当前节点 ID 为 `0x08`，主机地址为 `0x55`，所以配置请求 CAN ID 为 `0x0CEF0855`。如果 IMU 当前节点不是 `0x08`，把命令里的 `0CEF0855` 改成 `0CEF[当前ID]55`。
>
> 两个新 IMU 如果当前 ID 相同，必须一次只接一个 IMU 进行配置，避免两个设备同时响应并被写成同一个 ID。

## 1. 主机 CAN 口设为 1M

普通 SocketCAN 接口：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

如果使用 `slcand` USB-CAN，`s8` 表示 `1 Mbit/s`：

```bash
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
```

## 2. 左脚 IMU：设置为 ID 0x01

只连接左脚 IMU，然后执行：

```bash
# 打开周期输出总开关
cansend can0 0CEF0855#9D00060001000000

# 50 Hz 输出：时间 / 加速度 / 角速度 / 四元数，周期 20 ms = 0x14
cansend can0 0CEF0855#2F01060014000000
cansend can0 0CEF0855#3401060014000000
cansend can0 0CEF0855#3701060014000000
cansend can0 0CEF0855#4601060014000000

# 关闭不需要的数据：磁场 / 欧拉角 / 航向角 / 温度 / 倾角仪 / CANFD83
cansend can0 0CEF0855#3A01060000000000
cansend can0 0CEF0855#3D01060000000000
cansend can0 0CEF0855#4101060000000000
cansend can0 0CEF0855#4301060000000000
cansend can0 0CEF0855#4A01060000000000
cansend can0 0CEF0855#5B01060000000000

# IMU CAN 波特率设置为 1M：0 = 1000k
cansend can0 0CEF0855#9A00060000000000

# 节点 ID 设置为 0x01
cansend can0 0CEF0855#9C00060001000000

# 保存配置并复位，复位后用新 ID 0x01、生效波特率 1M
cansend can0 0CEF0855#0000060000000000
cansend can0 0CEF0855#00000600FF000000
```

## 3. 右脚 IMU：设置为 ID 0x02

只连接右脚 IMU，然后执行：

```bash
# 打开周期输出总开关
cansend can0 0CEF0855#9D00060001000000

# 50 Hz 输出：时间 / 加速度 / 角速度 / 四元数，周期 20 ms = 0x14
cansend can0 0CEF0855#2F01060014000000
cansend can0 0CEF0855#3401060014000000
cansend can0 0CEF0855#3701060014000000
cansend can0 0CEF0855#4601060014000000

# 关闭不需要的数据：磁场 / 欧拉角 / 航向角 / 温度 / 倾角仪 / CANFD83
cansend can0 0CEF0855#3A01060000000000
cansend can0 0CEF0855#3D01060000000000
cansend can0 0CEF0855#4101060000000000
cansend can0 0CEF0855#4301060000000000
cansend can0 0CEF0855#4A01060000000000
cansend can0 0CEF0855#5B01060000000000

# IMU CAN 波特率设置为 1M：0 = 1000k
cansend can0 0CEF0855#9A00060000000000

# 节点 ID 设置为 0x02
cansend can0 0CEF0855#9C00060002000000

# 保存配置并复位，复位后用新 ID 0x02、生效波特率 1M
cansend can0 0CEF0855#0000060000000000
cansend can0 0CEF0855#00000600FF000000
```

## 4. 验证配置

两个 IMU 都配置完成后，一起接到 CAN 总线上，主机保持 `1 Mbit/s`，查看输出帧：

```bash
candump can0
```

应能看到以下 J1939 扩展帧：

| IMU | 时间 | 加速度 | 角速度 | 四元数 |
| --- | --- | --- | --- | --- |
| 左脚 ID `0x01` | `0CFF2F01` | `0CFF3401` | `0CFF3701` | `0CFF4601` |
| 右脚 ID `0x02` | `0CFF2F02` | `0CFF3402` | `0CFF3702` | `0CFF4602` |

输出频率应接近 `50 Hz`，也就是每类保留数据约每 `20 ms` 出现一次。

## 5. 常用命令说明

| 配置项 | 数据字节 | 说明 |
| --- | --- | --- |
| `9D 00 06 00 01 00 00 00` | 打开周期输出总开关 | 必须为 `1` 才会周期输出 |
| `2F 01 06 00 14 00 00 00` | 时间输出 20 ms | PGN `0xFF2F` |
| `34 01 06 00 14 00 00 00` | 加速度输出 20 ms | PGN `0xFF34` |
| `37 01 06 00 14 00 00 00` | 角速度输出 20 ms | PGN `0xFF37` |
| `46 01 06 00 14 00 00 00` | 四元数输出 20 ms | PGN `0xFF46` |
| `9A 00 06 00 00 00 00 00` | CAN 波特率 1M | `0 = 1000k`，保存并复位后生效 |
| `9C 00 06 00 01 00 00 00` | 节点 ID 设为 `0x01` | 保存并复位后生效 |
| `9C 00 06 00 02 00 00 00` | 节点 ID 设为 `0x02` | 保存并复位后生效 |
| `00 00 06 00 00 00 00 00` | 保存配置 | 写入非易失配置 |
| `00 00 06 00 FF 00 00 00` | 复位 | 重启后新波特率 / 新 ID 生效 |

参考：HiPNUC 指令与编程手册中的 J1939 配置协议和 PGN 周期配置。
