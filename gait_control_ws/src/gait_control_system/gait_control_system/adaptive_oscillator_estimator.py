from collections import deque

import numpy as np


class AdaptiveOscillatorEstimator:
    """
    Robust Adaptive Oscillator (RAO).

    Implements Eq.(1)-(9) in:
    Li et al., "Preference-Based Assistance Map Learning With Robust Adaptive Oscillators", TMRB 2022.
    """

    def __init__(self, dt, config):
        self.dt = dt
        self.config = config
        # Harmonic order K in the paper. K=1 means bias + fundamental harmonic.
        self.harmonic_order = max(1, int(self.config.get("OSC_HARMONICS", 1)))
        self._init_filter()
        self.reset()

    def _init_filter(self):
        # Keep filter optional so we can still run with raw encoder difference as in RAO.
        self.dc_buffer = deque(maxlen=100)
        self.filter_enabled = bool(self.config.get("FILTER_ENABLED", False))
        self.sos = None
        self.sos_state = None
        self.sp_signal = None
        if not self.filter_enabled:
            return
        try:
            from scipy import signal as sp_signal

            self.sp_signal = sp_signal
            cutoff = float(self.config.get("FILTER_CUTOFF", 0.3))
            fs = 1.0 / max(self.dt, 1e-6)
            self.sos = sp_signal.butter(2, cutoff, "hp", fs=fs, output="sos")
            self.sos_state = sp_signal.sosfilt_zi(self.sos)
        except Exception as exc:
            print(f"高通滤波器初始化失败，回退到去均值模式: {exc}")
            self.filter_enabled = False
            self.sos = None
            self.sos_state = None
            self.sp_signal = None

    def _reset_state(self):
        size = self.harmonic_order + 1
        self.alpha = np.zeros(size, dtype=float)
        self.phi = np.zeros(size, dtype=float)

        self.alpha[0] = float(self.config.get("OSC_ALPHA0_INIT", 0.0))
        alpha_init = float(self.config.get("OSC_ALPHA_INIT", 0.4))
        if size > 1:
            self.alpha[1:] = alpha_init

        # Paper note: phi_0 is fixed to pi/2 so alpha_0 acts as a bias term.
        self.phi[0] = np.pi / 2.0

        self.omega = float(self.config["OSC_OMEGA_INIT"])
        self.omega_bar = None  # Zero-crossing frequency estimate (Eq.3).

        self.prev_q = None
        self.last_zero_time = None
        self.prev_zero_time = None
        self.time_now = 0.0

        self.phi_aux = 0.0
        self.last_q_hat = 0.0
        self.last_error = 0.0
        self.last_phase = 0.0
        self.last_zero_cross_event = False

        # Compatibility cache: para[j] = [amp, freq, phase]
        self.para = np.zeros((size, 3), dtype=float)
        self._sync_para()

    def reset(self, reset_filter=True):
        """Reset oscillator state and recover initial frequency."""
        self._reset_state()
        if reset_filter:
            self._init_filter()

    def update_dt(self, new_dt):
        """Update sampling period; rebuild filter only if dt changes significantly."""
        if new_dt <= 0:
            return
        rel_change = abs(new_dt - self.dt) / max(self.dt, 1e-6)
        self.dt = float(new_dt)
        if rel_change > 0.05:
            self._init_filter()

    def set_base_frequency(self, omega_init):
        self.config["OSC_OMEGA_INIT"] = float(omega_init)
        self.reset(reset_filter=False)

    def prime_phase(self, phase):
        """Force output phase anchor for stop->walk transition."""
        phase_wrapped = float(np.mod(float(phase), 2.0 * np.pi))
        # phase = mod(phi_aux_c - pi/2, 2pi). Here we directly align phi_aux.
        self.phi_aux = float(np.mod(phase_wrapped + np.pi / 2.0, 2.0 * np.pi))
        self.last_phase = phase_wrapped
        self.last_zero_cross_event = False

    def _filter(self, sample):
        sample = float(sample)
        if self.filter_enabled and self.sos is not None and self.sp_signal is not None:
            try:
                filtered, self.sos_state = self.sp_signal.sosfilt(
                    self.sos, [sample], zi=self.sos_state
                )
                return float(filtered[0])
            except Exception as exc:
                print(f"滤波处理失败，回退到去均值模式: {exc}")
                self.filter_enabled = False

        self.dc_buffer.append(sample)
        if len(self.dc_buffer) >= 10:
            return sample - (sum(self.dc_buffer) / len(self.dc_buffer))
        return sample

    def _safe_denominator(self):
        den = float(np.sum(self.alpha))
        eps = float(self.config.get("OSC_DEN_EPS", 1e-6))
        if abs(den) < eps:
            return eps if den >= 0.0 else -eps
        return den

    def _kw(self, q, dq):
        # Eq.(5): active near zero-crossing from negative to positive.
        if dq <= 0.0:
            return 0.0
        dw = float(self.config.get("RAO_DW", 0.35))
        if q < 0.0 or q > dw:
            return 0.0
        xi_w = float(self.config.get("RAO_XI_W", 12.0))
        sigma_w = max(float(self.config.get("RAO_SIGMA_W", 0.12)), 1e-6)
        return float(xi_w * np.exp(-(q * q) / (2.0 * sigma_w * sigma_w)))

    def _kphi(self, q, dq):
        # Eq.(8): active right before next zero-crossing.
        if dq <= 0.0:
            return 0.0
        dphi = float(self.config.get("RAO_DPHI", 0.35))
        if q < -dphi or q > 0.0:
            return 0.0
        xi_phi = float(self.config.get("RAO_XI_PHI", 1.0))
        sigma_phi = max(float(self.config.get("RAO_SIGMA_PHI", 0.12)), 1e-6)
        return float(xi_phi * np.exp(-(q * q) / (2.0 * sigma_phi * sigma_phi)))

    def _sync_para(self):
        # Maintain legacy matrix interface used by upper-layer code.
        self.para.fill(0.0)
        for i in range(self.harmonic_order + 1):
            self.para[i, 0] = float(self.alpha[i])
            self.para[i, 2] = float(np.mod(self.phi[i], 2.0 * np.pi))
            if i > 0:
                self.para[i, 1] = float(self.omega * i)

    def run_filter_only(self, raw_sample):
        """Update filter states only; do not run oscillator dynamics."""
        q = self._filter(raw_sample)
        self.prev_q = q
        self.last_zero_cross_event = False
        return q

    def _detect_zero_cross(self, prev_q, q, dq, t_now):
        if prev_q is None:
            return False
        if not (prev_q < 0.0 <= q and dq > 0.0):
            return False

        # Linear interpolation for more accurate crossing time.
        if abs(q - prev_q) > 1e-9:
            ratio = np.clip((-prev_q) / (q - prev_q), 0.0, 1.0)
            t_cross = t_now - self.dt + ratio * self.dt
        else:
            t_cross = t_now

        self.prev_zero_time = self.last_zero_time
        self.last_zero_time = float(t_cross)
        self.phi_aux = 0.0  # Eq.(6): reset auxiliary phase at zero-crossing.

        if self.prev_zero_time is not None:
            period = self.last_zero_time - self.prev_zero_time
            if period > 1e-6:
                omega_bar = (2.0 * np.pi) / period
                self.omega_bar = float(
                    np.clip(
                        omega_bar,
                        self.config["OSC_OMEGA_MIN"],
                        self.config["OSC_OMEGA_MAX"],
                    )
                )
        return True

    def step(self, raw_sample, raw_derivative=None):
        """Process one sample and return RAO gait phase in [0, 2pi).

        raw_derivative is optional to keep the original encoder-angle path
        unchanged; IMU paths can pass gyro-derived angular velocity directly.
        """
        q = self._filter(raw_sample)
        dt = max(self.dt, 1e-6)
        t_now = self.time_now + dt

        prev_q = self.prev_q
        if raw_derivative is None:
            dq = 0.0 if prev_q is None else (q - prev_q) / dt
        else:
            dq = float(raw_derivative)
        zero_cross = self._detect_zero_cross(prev_q, q, dq, t_now)

        sin_phi = np.sin(self.phi)
        cos_phi = np.cos(self.phi)
        q_hat = float(np.dot(self.alpha, sin_phi))
        err = q - q_hat
        den = self._safe_denominator()

        v_phi = float(self.config.get("RAO_V_PHI", 8.0))
        v_w = float(self.config.get("RAO_V_W", 3.0))
        eta = float(self.config.get("RAO_ETA", 1.5))

        alpha_dot = eta * err * sin_phi
        phi_dot = np.zeros_like(self.phi)
        common = err / den
        for i in range(1, self.harmonic_order + 1):
            phi_dot[i] = i * self.omega + v_phi * common * cos_phi[i]

        kw = self._kw(q, dq)
        omega_bar = self.omega_bar if self.omega_bar is not None else self.omega
        omega_dot = v_w * common * cos_phi[1] + kw * (omega_bar - self.omega)

        self.alpha += alpha_dot * dt
        self.phi[1:] += phi_dot[1:] * dt
        self.phi[0] = np.pi / 2.0
        self.omega += omega_dot * dt
        self.omega = float(
            np.clip(self.omega, self.config["OSC_OMEGA_MIN"], self.config["OSC_OMEGA_MAX"])
        )

        if zero_cross:
            self.phi_aux = 0.0
        else:
            self.phi_aux += self.omega * dt
        kphi = self._kphi(q, dq)
        phi_aux_c = self.phi_aux + kphi * (2.0 * np.pi - self.phi_aux)

        # Eq.(9): gait cycle phase in [0, 2pi).
        # 项目约定：phase=0 对应左脚跟着地（角度差正过零点前约 1/4 周期）。
        phase = float(np.mod(phi_aux_c - np.pi / 2.0, 2.0 * np.pi))

        self.last_q_hat = q_hat
        self.last_error = float(err)
        self.last_phase = phase
        self.last_zero_cross_event = zero_cross
        self.prev_q = q
        self.time_now = t_now
        self._sync_para()
        return phase
