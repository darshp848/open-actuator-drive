"""Pinned stock OpenArm v2 model access and explicit named joint interfaces."""
import hashlib
import json
from pathlib import Path

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parent
VENDOR = ROOT / 'vendor' / 'openarm_mujoco'
MODEL = VENDOR / 'v2' / 'pedestal' / 'pedestal.xml'
CONTACT_MODEL = VENDOR / 'v2' / 'pedestal' / 'move_puck_scene.xml'
REFERENCE = ROOT / 'reference' / 'v2.urdf'
ARMS = [f'openarm_{side}_joint{i}' for side in ('left', 'right') for i in range(1, 8)]
COMMAND_JOINTS = [f'openarm_{side}_{joint}' for side in ('left', 'right')
                  for joint in [*(f'joint{i}' for i in range(1, 8)), 'finger_joint1']]
FINGER_ALIASES = {f'openarm_{side}_ee_link{i}': f'openarm_{side}_ee_{kind}_finger'
                  for side in ('left', 'right') for i, kind in [(1, 'inner'), (2, 'outer')]}


def sha256(file_path):
    return hashlib.sha256(Path(file_path).read_bytes()).hexdigest()


def verify_assets():
    manifest = json.loads((ROOT / 'assets-manifest.json').read_text())
    for name, expected in manifest['files_sha256'].items():
        assert sha256(VENDOR / name) == expected, f'Changed vendor asset: {name}'
    assert sha256(REFERENCE) == manifest['reference_urdf_sha256']
    assert mj.__version__ == '3.6.0', 'Use the separate pinned .venv-openarm environment'
    return manifest


def load(contact=False, step=0.001):
    model = mj.MjModel.from_xml_path(str(CONTACT_MODEL if contact else MODEL))
    model.opt.timestep = step
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, 0)
    for side, opening in [('left', 0.35), ('right', -0.35)]:
        for finger in (1, 2):
            joint = model.joint(f'openarm_{side}_finger_joint{finger}')
            data.qpos[joint.qposadr[0]] = opening
    sync_position_commands(model, data)
    mj.mj_forward(model, data)
    return model, data


def sync_position_commands(model, data, positions=None):
    positions = data.qpos if positions is None else positions
    for i in range(model.nu):
        joint = model.joint(int(model.actuator_trnid[i, 0]))
        data.ctrl[i] = positions[joint.qposadr[0]]


def joint_mapping(model):
    mapping = []
    for name in COMMAND_JOINTS:
        joint = model.joint(name)
        actuators = np.flatnonzero(model.actuator_trnid[:, 0] == joint.id)
        assert len(actuators) == 1, f'Ambiguous actuator for {name}'
        actuator = int(actuators[0])
        mapping.append(dict(joint=name, qpos=int(joint.qposadr[0]), dof=int(joint.dofadr[0]),
                            actuator=actuator, actuator_name=model.actuator(actuator).name,
                            position_limits_rad=model.jnt_range[joint.id].tolist(),
                            output_torque_limits_nm=model.actuator_forcerange[actuator].tolist()))
    return mapping


def warning_count(data):
    return int(sum(int(item.number) for item in data.warning))
