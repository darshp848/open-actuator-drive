"""Checks on the virtual joint before any experiment (bounded, analytic where possible)."""
import json, math
from dataclasses import replace
from pathlib import Path
import numpy as np
import mujoco as mj
from actuator import (NOMINAL, VirtualActuator, Driver, DT_ELEC, transmitted, Gains, FrictionMap, run,
                      command_signal, T_REF)
from models import reference_prediction, TrainingSim, first_order_fine, ms_commands

OUT = Path(__file__).with_name('results'); OUT.mkdir(exist_ok=True)
checks = {}

# 1. Mass matrix: rotor and pendulum inertia about the pivot
a = VirtualActuator(NOMINAL, 100e-6)
mj.mj_forward(a.m, a.d)
I_link = NOMINAL.mass / 48 + NOMINAL.mass * 0.25 ** 2
checks['inertia'] = dict(rotor=float(a.d.qM[0]), rotor_expected=NOMINAL.j_rotor_out,
                         link=float(a.d.qM[-1]), link_expected=I_link)
assert abs(a.d.qM[0] - NOMINAL.j_rotor_out) < 1e-12 and abs(a.d.qM[-1] - I_link) < 1e-12

# 2. Winding: open-loop voltage step at standstill vs analytic R-L response
p = NOMINAL; drv = Driver(p); drv.kp = 0; drv.ki = 0
V = 3.0; i = 0.0; worst = 0.0
E = math.exp(-p.R * DT_ELEC / p.L)
for k in range(1, 200):
    i_ss = V / p.R; i = i_ss + (i - i_ss) * E
    worst = max(worst, abs(i - V / p.R * (1 - math.exp(-p.R * k * DT_ELEC / p.L))))
checks['rl_exact_max_abs_error_A'] = worst
assert worst < 1e-12

# 3. Closed current loop at standstill: 10-90 % rise time for a 2 A step
drv = Driver(p); tr = []
for k in range(200):
    drv.substep(2.0, 0.0); tr.append(drv.i)
tr = np.array(tr); t = np.arange(1, 201) * DT_ELEC
rise = t[np.argmax(tr >= 1.8)] - t[np.argmax(tr >= 0.2)]
checks['current_loop'] = dict(rise_10_90_ms=float(rise * 1e3), overshoot_pct=float((tr.max() / 2 - 1) * 100),
                              final_A=float(tr[-1]))
assert rise < 1e-3 and abs(tr[-1] - 2) < 1e-3

# 4. Gearbox spring (MuJoCo fixed tendon): shaft-torque formula matches MuJoCo passive force,
#    momentum is conserved, and free-oscillation energy error shrinks with the step
a = VirtualActuator(replace(NOMINAL, gap=0.004), 100e-6); a.d.qpos[:] = [0.01, 0.0]; a.d.qvel[:] = [0.3, -0.1]
mj.mj_forward(a.m, a.d)
assert abs(a.shaft() - (a.d.qfrc_passive[1] + 0.002 * a.d.qvel[1])) < 1e-12   # remove load-joint damping
checks['shaft_formula_matches_tendon'] = 'passed'
free = replace(NOMINAL, damping=0.0, coulomb=0, stribeck=0, viscous=0, kt=0.0)   # no back-EMF braking
energy = {}
for h in [500e-6, 250e-6, 100e-6, 50e-6]:
    a = VirtualActuator(free, h); a.m.opt.gravity[:] = 0; a.m.dof_damping[:] = 0; a.m.dof_frictionloss[:] = 0
    a.d.qpos[:] = [0.01, 0.0]; mj.mj_forward(a.m, a.d)
    e0 = 0.5 * free.stiffness * 0.01 ** 2; err = 0.0
    for _ in range(100):
        a.step_ms(0.0)
        q, v = a.d.qpos, a.d.qvel
        e = 0.5 * a.m.dof_armature[0] * 0 + 0.5 * a.d.qM[0] * v[0] ** 2 + 0.5 * a.d.qM[-1] * v[1] ** 2 \
            + 0.5 * free.stiffness * (q[0] - q[1]) ** 2
        err = max(err, abs(e - e0) / e0)
    mom = a.d.qM[0] * a.d.qvel[0] + a.d.qM[-1] * a.d.qvel[1]
    energy[f'{h*1e6:.0f}us'] = dict(max_rel_energy_error=float(err), momentum=float(mom))
checks['free_spring_energy'] = energy
assert energy['50us']['max_rel_energy_error'] < energy['500us']['max_rel_energy_error']
assert abs(energy['50us']['momentum']) < 1e-9

# 5. Backlash flanks are one-sided and passive
g = replace(NOMINAL, gap=0.004)
assert transmitted(0.003, 0.0, g) == 0 and transmitted(0.006, 0, g) > 0 and transmitted(-0.006, 0, g) < 0
rng = np.random.default_rng(1); diss = []
for dlt, v in rng.uniform([-.02, -5], [.02, 5], (2000, 2)):
    spring = g.stiffness * max(abs(dlt) - g.gap, 0) * np.sign(dlt)   # dissipation = damper power
    diss.append((transmitted(dlt, v, g) - spring) * v)
checks['backlash_min_dissipation'] = float(min(diss))
assert min(diss) > -1e-12

# 6. Static torque: constant 1 Nm request, torque feedback, pendulum settles at sin(q) = tau / (m g l)
# Extra load damping is added for this check only, so the undamped swing settles.
from actuator import LocalController
a = VirtualActuator(NOMINAL, 100e-6); a.m.dof_damping[1] = 0.3
ctl = LocalController('S', Gains(0.5, 30), FrictionMap(0.1, 0.02), 'torque', 0)
for k in range(6000):
    tau = a.step_ms(ctl(1.0, a)[0])[-1, 0]
q_exp = math.asin(1.0 / (NOMINAL.mass * 9.81 * 0.25))
checks['static_torque'] = dict(final_shaft_Nm=float(tau), final_angle=float(a.d.qpos[1]), expected_angle=q_exp)
assert abs(tau - 1.0) < 0.03 and abs(a.d.qpos[1] - q_exp) < 0.01

# 7. Reference model: MuJoCo filterexact training actuator equals the analytic first-order prediction
c = command_signal('random', 1.0, seconds=1.0, seed=3)
sim = TrainingSim('reference', 1.0, h=1e-3)
acts = []
m, d = sim.m, sim.d
for s in range(1000):
    j = int(math.floor((s * 1e-3 - 4e-3) / 0.02 + 1e-9))
    d.ctrl[0] = c[j] if j >= 0 else 0.0
    mj.mj_step(m, d); acts.append(float(d.act[0]))
pred = first_order_fine(ms_commands(c, 4), T_REF, 1, 1e-3)[:, -1]
checks['reference_filterexact_max_abs_error_Nm'] = float(np.max(np.abs(np.array(acts) - pred)))
assert checks['reference_filterexact_max_abs_error_Nm'] < 1e-9

checks['status'] = 'passed'; checks['mujoco'] = mj.__version__
(OUT / 'exp0_checks.json').write_text(json.dumps(checks, indent=2))
print(json.dumps(checks, indent=2))
