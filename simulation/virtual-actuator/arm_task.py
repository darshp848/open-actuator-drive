"""Task B: two-joint planar arm reaches a puck and pushes it to a goal (contact task).

Shared definition used by the GPU trainer (arm_gpu_es.py) and the detailed-plant deployment
(arm_plant.py). Everything that must be identical between training and deployment lives here:
geometry, contact parameters, task parameters, initial states, observation, reward, success and
the policy.

The arm moves in a horizontal plane (joint axes along z), so gravity only loads the puck's
sliding friction. Both joints use the same virtual actuator as the pendulum (actuator.Plant);
the fitted compact model of that actuator is therefore reused per joint.

Generalised coordinates
  train mode    : qpos = [q1, q2, px, py]            (arm joints driven by MuJoCo motors)
  detailed mode : qpos = [r1, r2, q1, q2, px, py]    (r = output-referred rotor angles; gearbox
                                                      tendons couple r_j to q_j)
q2 is the elbow angle relative to link 1. The elbow rotor is mounted to the world rather than
to link 1 (its reaction torque and gyroscopic coupling on link 1 are neglected; both are small
because the reflected rotor inertia is 0.00243 kg m^2).
"""
from dataclasses import dataclass, replace
import math
import numpy as np

# ---------------------------------------------------------------- geometry and timing
L1, L2 = 0.30, 0.25            # link lengths [m]
M1, M2 = 0.5, 0.3              # link masses [kg], uniform rods
TIP_R = 0.015                  # fingertip sphere radius
PUCK_R, PUCK_HH = 0.03, 0.02   # puck cylinder radius, half height
GOAL = (0.30, -0.10)
DT_TASK = 0.02                 # policy period (50 Hz)
EP_S = 4.0
N_STEPS = round(EP_S / DT_TASK)        # 200 policy steps
TAIL_STEPS = 25                        # final 0.5 s used for success
SUCCESS_TOL = 0.03                     # puck centre within 3 cm of goal for the whole tail
A_MAX = 1.0                            # per-joint torque request limit [Nm]
TAU_CAP = 2.0
HID = 32
N_OBS = 14
N_ACT = 2
P = HID * N_OBS + HID + N_ACT * HID + N_ACT     # 546 policy parameters

# (q1, q2, puck x, puck y): tip starts 14 to 20 cm above (+y) the puck, push direction is -y.
# Six starts spread over a 6 x 5 cm puck box and three arm postures, shared by training and deployment.
INITS = [(1.20, -1.00, 0.30, 0.15),
         (1.30, -1.20, 0.27, 0.13),
         (1.10, -0.85, 0.33, 0.13),
         (1.30, -1.20, 0.33, 0.17),
         (1.10, -0.85, 0.27, 0.17),
         (1.25, -1.10, 0.30, 0.12)]


@dataclass(frozen=True)
class Task:
    """Task-level (non-actuator) parameters."""
    payload: float = 0.2       # point mass at the fingertip [kg]
    puck_mass: float = 0.2
    puck_mu: float = 0.3       # puck-table Coulomb coefficient (slide-joint frictionloss = mu m g)


NOMINAL_TASK = Task()
# training randomisation ranges (deployment "new plants" draw inside them)
RAND = dict(payload=(0.0, 0.5), puck_mass=(0.1, 0.4), puck_mu=(0.15, 0.45), delay_ms=(2, 8))


# ---------------------------------------------------------------- MJCF
def _arm_body(task, jattr='damping="0.002"'):
    pm = max(task.payload, 1e-4)
    return f'''
 <body name="link1">
  <joint name="q1" type="hinge" axis="0 0 1" {jattr}/>
  <inertial pos="{L1/2} 0 0" mass="{M1}" diaginertia="1e-5 {M1*L1*L1/12} {M1*L1*L1/12}"/>
  <geom type="capsule" fromto="0 0 0 {L1} 0 0" size=".015" contype="0" conaffinity="0"/>
  <body name="link2" pos="{L1} 0 0">
   <joint name="q2" type="hinge" axis="0 0 1" {jattr}/>
   <inertial pos="{L2/2} 0 0" mass="{M2}" diaginertia="1e-5 {M2*L2*L2/12} {M2*L2*L2/12}"/>
   <geom type="capsule" fromto="0 0 0 {L2} 0 0" size=".012" contype="0" conaffinity="0"/>
   <body name="payload" pos="{L2} 0 0">
    <inertial pos="0 0 0" mass="{pm}" diaginertia="1e-6 1e-6 1e-6"/>
    <geom name="tip" type="sphere" size="{TIP_R}" contype="1" conaffinity="1" mass="0"
          friction="0.3 0.005 0.0001" condim="3"/>
   </body>
  </body>
 </body>'''


def _puck_body(task):
    fl = task.puck_mu * task.puck_mass * 9.81
    return f'''
 <body name="puck">
  <joint name="px" type="slide" axis="1 0 0" damping="0.01" frictionloss="{fl}"/>
  <joint name="py" type="slide" axis="0 1 0" damping="0.01" frictionloss="{fl}"/>
  <inertial pos="0 0 0" mass="{task.puck_mass}" diaginertia="1e-4 1e-4 1e-4"/>
  <geom name="puck" type="cylinder" size="{PUCK_R} {PUCK_HH}" contype="1" conaffinity="1" mass="0"
        friction="0.3 0.005 0.0001" condim="3"/>
 </body>'''


def train_xml(task=NOMINAL_TASK, h=1e-3, compact=None):
    """Training simulator: rigid arm, torques applied by motors on q1, q2 (ctrl). For the compact
    route the fitted rotor armature and viscous friction are added to each arm joint (Coulomb
    friction is applied by the trainer through qfrc_applied, as in gpu_es.py)."""
    jattr = 'damping="0.002"'
    if compact is not None:
        jattr = f'armature="{compact["armature"]}" damping="{0.002 + compact["viscous"]}"'
    body = _arm_body(task, jattr)
    return f'''<mujoco model="arm_train"><option timestep="{h}" integrator="implicitfast" gravity="0 0 -9.81"/>
<worldbody>{body}{_puck_body(task)}</worldbody>
<actuator><motor joint="q1" gear="1" ctrllimited="true" ctrlrange="-{TAU_CAP} {TAU_CAP}"/>
<motor joint="q2" gear="1" ctrllimited="true" ctrlrange="-{TAU_CAP} {TAU_CAP}"/></actuator></mujoco>'''


def detailed_xml(task, p1, p2, h):
    """Detailed plant: two output-referred rotors (actuator.Plant p1, p2) coupled to q1, q2 by
    gearbox tendons with stiffness, damping and backlash, exactly as actuator.plant_xml does
    for the pendulum. Rotor forces are applied through qfrc_applied[0:2]."""
    rotors, tendons = '', ''
    for k, p in enumerate((p1, p2), 1):
        j = p.j_rotor_out
        rotors += f'''
 <body name="rotor{k}"><inertial pos="0 0 0" mass="0.1" diaginertia="{j} {j} {j}"/>
  <joint name="r{k}" type="hinge" axis="0 0 1" damping="{p.viscous}" frictionloss="{p.coulomb + p.dry}" solreffriction=".002 1" solimpfriction=".999 .999 .001"/>
  <geom type="sphere" size=".01" contype="0" conaffinity="0"/></body>'''
        tendons += f'''
 <fixed name="gearbox{k}" stiffness="{p.stiffness}" damping="{p.damping}" springlength="{-p.gap} {p.gap}">
  <joint joint="r{k}" coef="1"/><joint joint="q{k}" coef="-1"/></fixed>'''
    return f'''<mujoco model="arm_detailed">
<option timestep="{h}" integrator="implicitfast" gravity="0 0 -9.81" iterations="100" tolerance="1e-12"/>
<worldbody>{rotors}{_arm_body(task)}{_puck_body(task)}</worldbody>
<tendon>{tendons}</tendon></mujoco>'''


# ---------------------------------------------------------------- task functions
def tip_xy(q1, q2):
    return (L1 * math.cos(q1) + L2 * math.cos(q1 + q2), L1 * math.sin(q1) + L2 * math.sin(q1 + q2))


def observe(q1, q2, w1, w2, px, py, vx, vy, u1, u2):
    tx, ty = tip_xy(q1, q2)
    return np.array([math.cos(q1), math.sin(q1), math.cos(q2), math.sin(q2), w1 / 5.0, w2 / 5.0,
                     (px - tx) / 0.2, (py - ty) / 0.2, (GOAL[0] - px) / 0.2, (GOAL[1] - py) / 0.2,
                     vx / 0.5, vy / 0.5, u1 / A_MAX, u2 / A_MAX])


def reward(q1, q2, w1, w2, px, py, u1, u2):
    """Per policy step. Distance terms are in metres; ES ranks returns, so only ratios matter."""
    tx, ty = tip_xy(q1, q2)
    d_pg = math.hypot(px - GOAL[0], py - GOAL[1])
    d_tp = math.hypot(px - tx, py - ty)
    return -(2.0 * d_pg + 0.5 * max(d_tp - (TIP_R + PUCK_R), 0.0)
             + 0.01 * (w1 * w1 + w2 * w2) + 0.02 * (u1 * u1 + u2 * u2)) * DT_TASK


def goal_dist(px, py):
    return math.hypot(px - GOAL[0], py - GOAL[1])


def act(theta, obs):
    """MLP 14-32-2, tanh hidden, A_MAX * tanh output. Parameter layout (shared with the GPU
    kernel): W1 (HID, N_OBS) row-major, b1 (HID), W2 (N_ACT, HID) row-major, b2 (N_ACT)."""
    i = 0
    W1 = theta[i:i + HID * N_OBS].reshape(HID, N_OBS); i += HID * N_OBS
    b1 = theta[i:i + HID]; i += HID
    W2 = theta[i:i + N_ACT * HID].reshape(N_ACT, HID); i += N_ACT * HID
    b2 = theta[i:i + N_ACT]
    return A_MAX * np.tanh(W2 @ np.tanh(W1 @ obs + b1) + b2)
