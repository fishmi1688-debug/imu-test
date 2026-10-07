# Git

```sh
git add .
git commit -m "first commit"
git push -u origin
```

上楼梯、下楼梯、骑自行车、上坡行走、下坡行走这五个模式都使用imu进行相位估计（方法以及imu获取和有线imu相位模式保持一致），其默认的助力参数中将相位偏执都设置为-0.5,其他都保持不变（其他保持不变）




在 test 模式下，程序不使用四元数角度作为主要角度输入，而是直接使用加速度计的重力投影;

由于角度可能在 +180° 和 -180° 之间跳变，test 模式使用 WrappedAngleLowpass，不会直接对角度数值做普通滤波。

普通滤波可能把：

```
+179° → -179°
```

误认为变化了 -358°。

当前处理方式是先计算最短角度差：

```ini
delta = shortest_angle(theta_current - theta_previous)
```

再进行一阶低通：

```makefile
alpha = dt / (tau + dt)

theta_filtered =
    theta_previous + alpha * delta
```

其中：

```ini
tau = 1 / (2πfc)
fc = 6 Hz
程序先把陀螺仪三轴角速度投影到大腿矢状面旋转轴，默认使用陀螺仪的对应轴的轴角速度。
```
