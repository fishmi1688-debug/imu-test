from collections import deque
import numpy as np

class AnalyzerStyleGaitDetector:
    """与 analyze_motor_angles.py 一致的步态开始/结束检测器"""

    def __init__(self, buffer_size=100, dt=0.033, rT=0.4):  # 30Hz采样频率
        self.buffer_size = buffer_size
        self.dt = dt
        self.rT = rT
        self.lhip_buffer = deque(maxlen=buffer_size)
        self.rhip_buffer = deque(maxlen=buffer_size)
        self.gait_state = 0  # 0=停止, 1=步行
        self.timer_r = 0.0

    def reset(self):
        self.gait_state = 0
        self.timer_r = 0.0
        self.lhip_buffer.clear()
        self.rhip_buffer.clear()

    def detect(self, left_angle, right_angle):
        """返回 (gait_state, state_changed, r_value)"""
        self.lhip_buffer.append(left_angle)
        self.rhip_buffer.append(right_angle)

        if len(self.lhip_buffer) < 20 or len(self.rhip_buffer) < 20:
            return self.gait_state, False, 0.0

        left_mean = np.mean(list(self.lhip_buffer)[-20:])
        right_mean = np.mean(list(self.rhip_buffer)[-20:])
        r_value = np.sqrt((left_angle - left_mean) ** 2 + (right_angle - right_mean) ** 2)

        state_changed = False
        if self.gait_state == 0:
            if r_value > self.rT:
                self.gait_state = 1
                self.timer_r = 0.0
                state_changed = True
        else:
            if r_value > self.rT:
                self.timer_r = 0.0
            else:
                self.timer_r += self.dt

            if self.timer_r > 2.5:
                self.gait_state = 0
                self.timer_r = 0.0
                state_changed = True

        return self.gait_state, state_changed, r_value
