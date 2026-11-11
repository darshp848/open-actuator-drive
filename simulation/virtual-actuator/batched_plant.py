"""Batched detailed plant on mjbatch (CPU, C++ thread pool), for the speed benchmark.

Mirrors actuator.VirtualActuator + LocalController (torque-sensor condition, kinds 'S' and
'H' with lead, friction feedforward and deflection damping; no disturbance observer) for
N simulations at once: the driver, sensors and controller run as NumPy array operations
and mjbatch steps all physics. Run with simulation/.venv-mjbatch (MuJoCo 3.14).
"""
import math
import numpy as np
import mujoco
from mjbatch import Batch
from actuator import (NOMINAL, plant_xml, DT_LOCAL, DT_ELEC, T_REF, Gains, FrictionMap, drive_commands)


class BatchedRun:
    def __init__(self, plant, n, h=25e-6, threads=0, sensor_noise=True):
        self.p, self.n, self.h = plant, n, h
        self.m = mujoco.MjModel.from_xml_string(plant_xml(plant, h))
        self.b = Batch(self.m, num_sims=n, num_threads=threads)
        self.qpos, self.qvel = self.b.bind('qpos'), self.b.bind('qvel')
        self.qfrc = self.b.bind('qfrc_applied')
        self.per_v = round(DT_ELEC / h)
        self.n_phys = round(DT_LOCAL / h)
        self.noise = sensor_noise

    def shaft(self):
        p = self.p
        delta = self.qpos[:, 0] - self.qpos[:, 1]
        rel = self.qvel[:, 0] - self.qvel[:, 1]
        spring = np.where(delta > p.gap, p.stiffness * (delta - p.gap),
                          np.where(delta < -p.gap, p.stiffness * (delta + p.gap), 0.0))
        return spring + p.damping * rel

    def run(self, kind, gains, fmap, c_drive, seeds):
        """c_drive: (n, n_ms) commands seen at the drive. Returns shaft torque (n, n_ms, n_phys)."""
        p, n, h, g = self.p, self.n, self.h, gains
        n_ms = c_drive.shape[1]
        rngs = np.random.default_rng(seeds[0] + 1)
        # driver state
        i = np.zeros(n); integ_i = np.zeros(n); v = np.zeros(n)
        kp_i, ki_i = NOMINAL.L * 2 * math.pi * 1000, NOMINAL.R * 2 * math.pi * 1000
        E = math.exp(-p.R * h / p.L)
        # controller state
        integ = np.zeros(n); r = np.zeros(n); a_ref = math.exp(-DT_LOCAL / T_REF)
        w_prev = np.zeros(n); th_prev = None; defl_prev = None; ddefl = np.zeros(n)
        a_defl = math.exp(-DT_LOCAL * 2 * math.pi * 200)
        enc_m = 2 * np.pi / (16384 * NOMINAL.ratio); enc_l = 2 * np.pi / 16384
        prev_meas = None
        tau_lim = NOMINAL.i_max * NOMINAL.ratio * NOMINAL.kt
        out = np.empty((n, n_ms, self.n_phys))
        self.qpos[:] = 0.0; self.qvel[:] = 0.0
        for k in range(n_ms):
            # sensors: 1-tick latency, like actuator.Sensors
            meas = dict(tau=self.shaft() + (rngs.normal(0, 0.01, n) if self.noise else 0.0),
                        th_r=np.round(self.qpos[:, 0] / enc_m) * enc_m,
                        th_l=np.round(self.qpos[:, 1] / enc_l) * enc_l)
            m_use = prev_meas if prev_meas is not None else meas
            prev_meas = meas
            tau_hat = m_use['tau']; th = m_use['th_r']
            w_raw = np.zeros(n) if th_prev is None else (th - th_prev) / DT_LOCAL
            th_prev = th
            w_hat = 0.7 * w_prev + 0.3 * w_raw; w_prev = w_hat
            c = c_drive[:, k]
            if kind == 'S':
                target = c; ff = c.copy()
            else:
                r = a_ref * r + (1 - a_ref) * c
                rdot = (c - r) / T_REF
                target = r
                ff = r + g.lead * rdot + g.ff_friction * (fmap.coulomb * np.tanh(w_hat / 0.02) + fmap.viscous * w_hat)
            if g.kd_defl:
                defl = m_use['th_r'] - m_use['th_l']
                if defl_prev is None:
                    defl_prev = defl
                ddefl = a_defl * ddefl + (1 - a_defl) * (defl - defl_prev) / DT_LOCAL
                defl_prev = defl
                ff = ff - g.kd_defl * ddefl
            e = target - tau_hat
            u_unsat = ff + g.kp * e + integ + g.ki * e * DT_LOCAL
            ok = np.abs(u_unsat) < tau_lim
            integ = np.where(ok, np.clip(integ + g.ki * e * DT_LOCAL, -1.0, 1.0), integ)
            i_cmd = np.clip(u_unsat, -tau_lim, tau_lim) / (NOMINAL.ratio * NOMINAL.kt)
            for s in range(self.n_phys):
                w_r = self.qvel[:, 0].copy()
                if s % self.per_v == 0:
                    err = i_cmd - i
                    vv = kp_i * err + integ_i + ki_i * err * DT_ELEC
                    integ_i = np.where(np.abs(vv) < p.v_max, integ_i + ki_i * err * DT_ELEC, integ_i)
                    v = np.clip(vv, -p.v_max, p.v_max)
                i_ss = (v - p.kt * p.ratio * w_r) / p.R
                i_mean = i_ss + (i - i_ss) * (1 - E) * p.L / (p.R * h)
                i = i_ss + (i - i_ss) * E
                tau_m = p.ratio * p.kt * i_mean
                if p.cogging:
                    tau_m = tau_m + p.cogging * np.sin(12 * p.ratio * self.qpos[:, 0])
                hump = p.stribeck * np.exp(-(w_r / p.v_stribeck) ** 2) * np.tanh(w_r / 0.05)
                self.qfrc[:, 0] = tau_m - hump
                self.b.step()
                out[:, k, s] = self.shaft()
        return out
