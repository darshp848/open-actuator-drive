"""Virtual geared actuator for the 21 Oct 2026 simulation plan.

Synthetic, output-referred model of the planned custom driver, frameless motor,
gearbox and pendulum load. No parameter here is a measured product value.

Command path (from the plan):
  torque request (50 Hz) -> transport delay -> [reference model] -> local torque
  controller (1 kHz) -> drive current loop (20 kHz) -> motor -> gearbox -> load

Mechanics run in MuJoCo with two hinge joints: an output-referred rotor and the
pendulum link. The gearbox is a spring-damper (optionally with backlash) applied
as equal and opposite generalized forces. The electrical model is integrated
exactly on a fixed 50 us grid that does not change with the physics step, so a
physics-step sweep leaves every controller and driver clock unchanged.
"""
from dataclasses import dataclass, replace, asdict
import math
import os
import numpy as np
import mujoco as mj

DT_LOCAL = 1e-3          # local torque loop period
DT_ELEC = 50e-6          # drive current loop period / exact electrical update
DT_TASK = 20e-3          # task (policy) command period
T_REF = float(os.environ.get("T_REF_S", "0.030"))   # prescribed reference time constant (env override for the envelope sweep)
D_NOM_MS = 4             # nominal command transport delay
TAU_CAP = 2.0            # torque request limit [Nm]


@dataclass(frozen=True)
class Plant:
    name: str = 'nominal'
    # motor + driver (motor side), all synthetic
    kt: float = 0.08            # Nm/A, also back-EMF constant V s/rad
    R: float = 0.6              # ohm
    L: float = 0.5e-3           # H
    v_max: float = 24.0         # V, driver output limit
    i_max: float = 4.0          # A, driver current limit
    ratio: float = 9.0          # gear ratio
    j_rotor_out: float = 0.00243  # rotor inertia reflected to output, kg m^2
    cogging: float = 0.0        # Nm at output, periodic in motor angle
    # gearbox friction on the rotor side, output-referred
    coulomb: float = 0.10
    stribeck: float = 0.05
    v_stribeck: float = 0.10
    viscous: float = 0.02
    dry: float = 0.0            # MuJoCo frictionloss (sticking) on rotor joint
    # gearbox structure
    stiffness: float = 300.0    # Nm/rad
    damping: float = 0.2        # Nm s/rad
    gap: float = 0.0            # total backlash half-gap [rad]
    # load
    mass: float = 1.0           # pendulum mass, COM at 0.25 m
    # timing
    delay_ms: int = D_NOM_MS
    jitter_ms: int = 0          # per-command uniform jitter 0..jitter_ms added


NOMINAL = Plant()
# Calibration plants: used for controller tuning, friction-map and compact-model fitting.
CALIBRATION = [NOMINAL,
               replace(NOMINAL, name='cal_heavy', mass=1.2),
               replace(NOMINAL, name='cal_friction', coulomb=0.14, viscous=0.03)]
# Held-out plants: never used for any parameter selection.
HELD_OUT = [
    replace(NOMINAL, name='friction_high', coulomb=0.20, stribeck=0.10, viscous=0.04),
    replace(NOMINAL, name='sticky_lowspeed', dry=0.06, stribeck=0.12, v_stribeck=0.05),
    replace(NOMINAL, name='load_light', mass=0.6),
    replace(NOMINAL, name='load_heavy', mass=1.5),
    replace(NOMINAL, name='delay_jitter', delay_ms=2, jitter_ms=8),
    replace(NOMINAL, name='soft_gearbox', stiffness=120.0, damping=0.12),
    replace(NOMINAL, name='backlash', gap=0.004),
    replace(NOMINAL, name='kt_hot', kt=0.072),
    replace(NOMINAL, name='cogging', cogging=0.03),
    replace(NOMINAL, name='combined', coulomb=0.20, stribeck=0.10, viscous=0.04, stiffness=120.0,
            damping=0.12, gap=0.004, kt=0.072, mass=1.3, delay_ms=6),
]


def stribeck_hump(v, p):
    """Low-speed friction excess over Coulomb. Coulomb (frictionloss) and viscous (damping)
    friction are native MuJoCo joint properties so the integrator handles them implicitly."""
    return p.stribeck * math.exp(-(v / p.v_stribeck) ** 2) * math.tanh(v / 0.05)


def friction(v, p):
    """Smooth equivalent of the total rotor friction, used for reporting only."""
    return (p.coulomb + p.dry) * math.tanh(v / 0.01) + stribeck_hump(v, p) + p.viscous * v


def transmitted(delta, rel_v, p):
    """Gearbox torque from rotor to load: spring with a backlash dead band plus a bilateral
    damper (lubricant drag also acts inside the gap). Mirrors the MuJoCo fixed tendon."""
    if delta > p.gap:
        spring = p.stiffness * (delta - p.gap)
    elif delta < -p.gap:
        spring = p.stiffness * (delta + p.gap)
    else:
        spring = 0.0
    return spring + p.damping * rel_v


def plant_xml(p, h):
    j = p.j_rotor_out
    return f'''<mujoco model="virtual_actuator">
<option timestep="{h}" integrator="implicitfast" gravity="0 0 -9.81" iterations="100" tolerance="1e-12"/>
<worldbody>
 <body name="rotor"><inertial pos="0 0 0" mass="0.1" diaginertia="{j} {j} {j}"/>
  <joint name="rotor" type="hinge" axis="0 1 0" damping="{p.viscous}" frictionloss="{p.coulomb + p.dry}" solreffriction=".002 1" solimpfriction=".999 .999 .001"/>
  <geom type="sphere" size=".01" contype="0" conaffinity="0"/></body>
 <body name="link"><inertial pos="0 0 -.25" mass="{p.mass}" diaginertia="{p.mass/48} {p.mass/48} {p.mass/1000}"/>
  <joint name="load" type="hinge" axis="0 1 0" damping="0.002"/>
  <geom type="capsule" fromto="0 0 0 0 0 -.5" size=".02" contype="0" conaffinity="0"/></body>
</worldbody>
<tendon><fixed name="gearbox" stiffness="{p.stiffness}" damping="{p.damping}" springlength="{-p.gap} {p.gap}">
 <joint joint="rotor" coef="1"/><joint joint="load" coef="-1"/></fixed></tendon></mujoco>'''


class Driver:
    """PI current loop at 20 kHz over an exactly integrated R-L winding."""
    def __init__(self, p):
        self.p = p
        self.i = 0.0
        self.integ = 0.0
        wc = 2 * math.pi * 1000.0      # ~1 kHz current-loop design bandwidth (nominal R, L)
        self.kp = NOMINAL.L * wc
        self.ki = NOMINAL.R * wc
        self.v = 0.0

    def update_voltage(self, i_cmd):
        """PI update, once per 50 us drive period (zero-order-hold voltage)."""
        p = self.p
        err = i_cmd - self.i
        v = self.kp * err + self.integ + self.ki * err * DT_ELEC
        if abs(v) < p.v_max:
            self.integ += self.ki * err * DT_ELEC
        self.v = max(-p.v_max, min(p.v_max, v))

    def advance(self, dt, omega_motor):
        """Exact R-L response over dt at constant voltage and speed; returns the mean current."""
        p = self.p
        E = math.exp(-p.R * dt / p.L)
        i_ss = (self.v - p.kt * omega_motor) / p.R
        i0 = self.i
        self.i = i_ss + (i0 - i_ss) * E
        return i_ss + (i0 - i_ss) * (1 - E) * p.L / (p.R * dt)

    def substep(self, i_cmd, omega_motor):
        self.update_voltage(i_cmd)
        return self.advance(DT_ELEC, omega_motor)


class VirtualActuator:
    """Detailed plant. step_ms(i_cmd) advances 1 ms of the local-loop clock."""
    def __init__(self, p, h=100e-6, q0=0.0):
        if h >= DT_ELEC:
            assert abs(h / DT_ELEC - round(h / DT_ELEC)) < 1e-9, 'h must be a multiple or divisor of 50 us'
            self.n_elec, self.per_v = round(h / DT_ELEC), 1
        else:
            assert abs(DT_ELEC / h - round(DT_ELEC / h)) < 1e-9, 'h must be a multiple or divisor of 50 us'
            self.n_elec, self.per_v = 1, round(DT_ELEC / h)   # voltage held across several physics steps
        assert abs(DT_LOCAL / h - round(DT_LOCAL / h)) < 1e-9
        self.p, self.h = p, h
        self.n_phys = round(DT_LOCAL / h)
        self.m = mj.MjModel.from_xml_string(plant_xml(p, h))
        self.d = mj.MjData(self.m)
        self.d.qpos[:] = q0
        mj.mj_forward(self.m, self.d)
        self.drv = Driver(p)
        self.shaft_trace = []      # exact shaft torque at every physics step
        self.t = 0.0

    def shaft(self):
        d = self.d
        return transmitted(float(d.qpos[0] - d.qpos[1]), float(d.qvel[0] - d.qvel[1]), self.p)

    def step_ms(self, i_cmd):
        p, d = self.p, self.d
        out = np.empty((self.n_phys, 2))   # shaft torque, load velocity after each physics step
        for k in range(self.n_phys):
            w_r = float(d.qvel[0])
            if self.per_v == 1:
                i_mean = 0.0
                for _ in range(self.n_elec):
                    i_mean += self.drv.substep(i_cmd, p.ratio * w_r)
                i_mean /= self.n_elec
            else:
                if k % self.per_v == 0:
                    self.drv.update_voltage(i_cmd)
                i_mean = self.drv.advance(self.h, p.ratio * w_r)
            tau_m = p.ratio * p.kt * i_mean
            if p.cogging:
                tau_m += p.cogging * math.sin(12 * p.ratio * float(d.qpos[0]))
            d.qfrc_applied[0] = tau_m - stribeck_hump(w_r, p)
            mj.mj_step(self.m, d)
            out[k, 0] = self.shaft()
            out[k, 1] = d.qvel[1]
        self.t += DT_LOCAL
        return out

    def ok(self):
        d = self.d
        return bool(np.all(np.isfinite(np.r_[d.qpos, d.qvel])) and abs(self.drv.i) < 1e3)


# ---------------------------------------------------------------- command signals
def command_signal(kind, amp, seconds=4.0, seed=0):
    """Task torque requests at 50 Hz (zero-order hold)."""
    n = round(seconds / DT_TASK)
    t = np.arange(n) * DT_TASK
    rng = np.random.default_rng(seed)
    if kind == 'reversal':
        c = amp * np.sin(2 * np.pi * 0.25 * t)
    elif kind == 'steps':
        pattern = [1, -1, .5, -.5, 1, 0, -1, 0]
        c = amp * np.array([pattern[int(x / 0.5) % 8] for x in t])
    elif kind == 'chirp':
        f0, f1 = 0.5, 8.0
        c = amp * np.sin(2 * np.pi * (f0 * t + (f1 - f0) * t ** 2 / (2 * seconds)))
    elif kind == 'random':
        levels = rng.uniform(-amp, amp, n // 5 + 1)
        c = np.repeat(levels, 5)[:n]
    elif kind == 'multisine':
        fr = [0.3, 1.1, 2.7, 5.3]
        ph = rng.uniform(0, 2 * np.pi, len(fr))
        c = sum(np.sin(2 * np.pi * f * t + a) for f, a in zip(fr, ph))
        c = amp * c / np.max(np.abs(c))
    else:
        raise ValueError(kind)
    c[:5] = 0.0      # start at rest
    return np.clip(c, -TAU_CAP, TAU_CAP)


def drive_commands(c_task, p, seed=0):
    """Command visible at the drive at each 1 ms tick after transport delay (+ jitter)."""
    rng = np.random.default_rng(10_000 + seed)
    per = round(DT_TASK / DT_LOCAL)
    n = len(c_task) * per
    out = np.zeros(n)
    arrival = []
    for j, c in enumerate(c_task):
        dly = p.delay_ms + (int(rng.integers(0, p.jitter_ms + 1)) if p.jitter_ms else 0)
        arrival.append((j * per + dly, c))
    arrival.sort()
    cur, a = 0.0, 0
    for k in range(n):
        while a < len(arrival) and arrival[a][0] <= k:
            cur = arrival[a][1]; a += 1
        out[k] = cur
    return out


def nominal_delayed(c_task, delay_ms=D_NOM_MS):
    per = round(DT_TASK / DT_LOCAL)
    ms = np.repeat(c_task, per)
    return np.r_[np.zeros(delay_ms), ms[:len(ms) - delay_ms]]


# ---------------------------------------------------------------- sensing
class Sensors:
    """Measurements available to a local controller. 1-tick (1 ms) latency."""
    def __init__(self, mode, seed):
        self.mode = mode
        self.rng = np.random.default_rng(seed)
        self.prev = None
        self.enc_m = 2 * np.pi / (16384 * NOMINAL.ratio)
        self.enc_l = 2 * np.pi / 16384

    def sample(self, act):
        d = act.d
        m = dict(
            tau=act.shaft() + self.rng.normal(0, 0.01),
            i=act.drv.i + self.rng.normal(0, 0.02),
            th_r=np.round(float(d.qpos[0]) / self.enc_m) * self.enc_m,
            th_l=np.round(float(d.qpos[1]) / self.enc_l) * self.enc_l,
        )
        out, self.prev = self.prev, m
        return out if out is not None else m


@dataclass
class FrictionMap:
    """Rotor-side friction identified on calibration plants (bench torque sensor)."""
    coulomb: float = 0.0
    viscous: float = 0.0

    def __call__(self, v):
        return self.coulomb * math.tanh(v / 0.02) + self.viscous * v


class TorqueEstimator:
    """Sensing condition 2: motor current + encoders, no output torque sensor."""
    def __init__(self, fmap):
        self.fmap = fmap
        self.th = None
        self.w = 0.0
        self.a = 0.0
        self.aw = math.exp(-DT_LOCAL * 2 * math.pi * 150)
        self.aa = math.exp(-DT_LOCAL * 2 * math.pi * 40)

    def __call__(self, m):
        th = m['th_r']
        if self.th is None:
            self.th = th
        w_raw = (th - self.th) / DT_LOCAL
        self.th = th
        w_new = self.aw * self.w + (1 - self.aw) * w_raw
        a_raw = (w_new - self.w) / DT_LOCAL
        self.w = w_new
        self.a = self.aa * self.a + (1 - self.aa) * a_raw
        return NOMINAL.ratio * NOMINAL.kt * m['i'] - self.fmap(self.w) - NOMINAL.j_rotor_out * self.a, self.w


# ---------------------------------------------------------------- local controllers
@dataclass
class Gains:
    kp: float = 0.5
    ki: float = 30.0
    lead: float = 0.0        # s, reference-derivative feedforward (shaping only)
    ff_friction: float = 0.0  # scale on identified friction feedforward (shaping only)
    kd_defl: float = 0.0      # Nm s/rad, active damping on gearbox deflection rate (both encoders)
    dob_hz: float = 0.0       # disturbance-observer cutoff; needs the torque sensor, replaces the friction map


class LocalController:
    """Conventional ('S'): PI on (c - tau_hat). Shaping ('H'): tracks the fixed reference r."""
    def __init__(self, kind, gains, fmap, sensing, seed):
        self.kind, self.g, self.fmap = kind, gains, fmap
        self.sensors = Sensors(sensing, seed)
        self.est = TorqueEstimator(fmap) if sensing == 'estimator' else None
        self.sensing = sensing
        self.integ = 0.0
        self.r = 0.0
        self.a_ref = math.exp(-DT_LOCAL / T_REF)
        self.w_prev = 0.0
        self.th_prev = None
        self.defl_prev = None
        self.ddefl = 0.0
        self.a_defl = math.exp(-DT_LOCAL * 2 * math.pi * 200)
        self.dob = 0.0
        self.u_prev = 0.0
        self.w_dob = None
        self.a_dob = math.exp(-DT_LOCAL * 2 * math.pi * gains.dob_hz) if gains.dob_hz else 0.0

    def reset_reference(self):
        self.r = 0.0

    def __call__(self, c, act):
        m = self.sensors.sample(act)
        if self.est is not None:
            tau_hat, w_hat = self.est(m)
        else:
            tau_hat = m['tau']
            th = m['th_r']
            w_hat = 0.0 if self.th_prev is None else (th - self.th_prev) / DT_LOCAL
            self.th_prev = th
            w_hat = 0.7 * self.w_prev + 0.3 * w_hat
            self.w_prev = w_hat
        g = self.g
        if self.kind == 'S':
            target, ff = c, c
        else:
            # exact first-order reference, consistent with the training model at tick instants
            r_next = self.a_ref * self.r + (1 - self.a_ref) * c
            rdot = (c - r_next) / T_REF
            self.r = r_next
            target = r_next
            ff = r_next + g.lead * rdot + g.ff_friction * self.fmap(w_hat)
        if g.dob_hz:
            # lumped rotor-side disturbance: commanded motor torque minus measured shaft torque minus rotor inertia
            if self.w_dob is None:
                self.w_dob = w_hat
            acc = (w_hat - self.w_dob) / DT_LOCAL
            self.w_dob = w_hat
            d_raw = self.u_prev - tau_hat - NOMINAL.j_rotor_out * acc
            self.dob = self.a_dob * self.dob + (1 - self.a_dob) * d_raw
            ff = ff - g.ff_friction * self.fmap(w_hat) + self.dob if self.kind != 'S' else ff + self.dob
        if g.kd_defl:
            defl = m['th_r'] - m['th_l']
            if self.defl_prev is None:
                self.defl_prev = defl
            self.ddefl = self.a_defl * self.ddefl + (1 - self.a_defl) * (defl - self.defl_prev) / DT_LOCAL
            self.defl_prev = defl
            ff = ff - g.kd_defl * self.ddefl
        e = target - tau_hat
        u_unsat = ff + g.kp * e + self.integ + g.ki * e * DT_LOCAL
        tau_lim = NOMINAL.i_max * NOMINAL.ratio * NOMINAL.kt
        if abs(u_unsat) < tau_lim:
            self.integ += g.ki * e * DT_LOCAL
            self.integ = max(-1.0, min(1.0, self.integ))
        u = max(-tau_lim, min(tau_lim, u_unsat))
        self.u_prev = u
        return u / (NOMINAL.ratio * NOMINAL.kt), target


# ---------------------------------------------------------------- one closed-loop run
def run(plant, kind, gains, fmap, sensing, c_task, h=100e-6, seed=0, keep=False):
    act = VirtualActuator(plant, h)
    ctl = LocalController(kind, gains, fmap, sensing, seed)
    c_drive = drive_commands(c_task, plant, seed)
    n = len(c_drive)
    shaft = np.empty((n, act.n_phys)); w_fine = np.empty((n, act.n_phys))
    th_l = np.empty(n); targ = np.empty(n); i_cmd = np.empty(n); w_r = np.empty(n)
    ok = True
    for k in range(n):
        ic, targ[k] = ctl(c_drive[k], act)
        i_cmd[k] = ic
        o = act.step_ms(ic)
        shaft[k], w_fine[k] = o[:, 0], o[:, 1]
        th_l[k] = act.d.qpos[1]; w_r[k] = act.d.qvel[0]
        if not act.ok() or not np.all(np.isfinite(o)) or abs(act.d.qvel[1]) > 200:
            ok = False
            shaft[k:] = np.nan; th_l[k:] = np.nan; w_fine[k:] = np.nan
            break
    return dict(ok=ok, shaft=shaft, th_l=th_l, w_fine=w_fine, w_r=w_r, target=targ, c_drive=c_drive,
                i_cmd=i_cmd, h=h, n_phys=act.n_phys,
                warnings=int(sum(int(w.number) for w in act.d.warning)))
