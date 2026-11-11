"""Independent URDF Jacobian checks against pinned native OpenArm MuJoCo model."""
import argparse
import json
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

import robot


def values(element, key, default='0 0 0'):
    return np.fromstring(element.get(key, default), sep=' ')


def transform(origin):
    result = np.eye(4)
    if origin is not None:
        result[:3, :3] = Rotation.from_euler('xyz', values(origin, 'rpy')).as_matrix()
        result[:3, 3] = values(origin, 'xyz')
    return result


class UrdfReference:
    def __init__(self):
        root = ET.parse(robot.REFERENCE).getroot()
        self.links = {item.get('name'): item for item in root.findall('link')}
        self.joints = root.findall('joint')
        self.moving = [j.get('name') for j in self.joints if j.get('type') == 'revolute']
        self.index = {name: i for i, name in enumerate(self.moving)}

    def evaluate(self, q):
        poses = {'world': np.eye(4)}
        ancestors = {'world': []}
        axes = {}
        for joint in self.joints:
            name = joint.get('name')
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            pose = poses[parent] @ transform(joint.find('origin'))
            ancestry = ancestors[parent].copy()
            if name in self.index:
                axis = values(joint.find('axis'), 'xyz')
                axes[name] = (pose[:3, 3].copy(), pose[:3, :3] @ axis)
                turn = np.eye(4)
                turn[:3, :3] = Rotation.from_rotvec(axis * q[self.index[name]]).as_matrix()
                pose = pose @ turn
                ancestry.append(name)
            poses[child], ancestors[child] = pose, ancestry
        n = len(self.moving)
        mass_matrix, gravity = np.zeros((n, n)), np.zeros(n)
        source_mass = 0.0
        for name, link in self.links.items():
            inertial = link.find('inertial')
            if inertial is None:
                continue
            mass = float(inertial.find('mass').get('value'))
            source_mass += mass
            center = poses[name] @ transform(inertial.find('origin'))
            a = inertial.find('inertia').attrib
            inertia = np.array([[float(a['ixx']), float(a['ixy']), float(a['ixz'])],
                                [float(a['ixy']), float(a['iyy']), float(a['iyz'])],
                                [float(a['ixz']), float(a['iyz']), float(a['izz'])]])
            eigenvalues = np.linalg.eigvalsh(inertia)
            assert mass > 0 and eigenvalues.min() > 0
            assert 2 * eigenvalues.max() <= eigenvalues.sum() + 1e-10
            jv, jw = np.zeros((3, n)), np.zeros((3, n))
            for joint in ancestors[name]:
                origin, axis = axes[joint]
                index = self.index[joint]
                jv[:, index] = np.cross(axis, center[:3, 3] - origin)
                jw[:, index] = axis
            world_inertia = center[:3, :3] @ inertia @ center[:3, :3].T
            mass_matrix += mass * jv.T @ jv + jw.T @ world_inertia @ jw
            gravity += mass * 9.81 * jv[2]
        return poses, mass_matrix, gravity, source_mass


def validate(output):
    manifest = robot.verify_assets()
    model, data = robot.load()
    assert (model.nq, model.nv, model.nu, model.neq) == (18, 18, 16, 2)
    mapping = robot.joint_mapping(model)
    reference = UrdfReference()
    order = np.array([model.joint(name).dofadr[0] for name in reference.moving])
    assert len(set(order)) == 18
    generator = np.random.default_rng(6102)
    poses_to_check = [np.zeros(18)]
    for _ in range(12):
        q = np.array([generator.uniform(*(0.6 * model.joint(name).range +
                         0.4 * model.joint(name).range.mean())) for name in reference.moving])
        for side in ('left', 'right'):
            q[reference.index[f'openarm_{side}_finger_joint2']] = q[reference.index[f'openarm_{side}_finger_joint1']]
        poses_to_check.append(q)
    metrics = dict(position_error_m=0., rotation_error=0., gravity_error_nm=0., mass_matrix_relative_error=0.)
    per_pose = []
    for q in poses_to_check:
        for name, angle in zip(reference.moving, q):
            data.qpos[model.joint(name).qposadr[0]] = angle
        data.qvel[:] = 0
        mj.mj_forward(model, data)
        poses, expected_m, expected_g, source_mass = reference.evaluate(q)
        full_m = np.zeros((model.nv, model.nv))
        mj.mj_fullM(model, full_m, data.qM)
        actual_m = (full_m - np.diag(model.dof_armature))[np.ix_(order, order)]
        actual_g = data.qfrc_bias[order]
        position_error, rotation_error = 0., 0.
        for name, pose in poses.items():
            if name in ('world', 'openarm_body_link0'):
                continue
            body = model.body(robot.FINGER_ALIASES.get(name, name))
            position_error = max(position_error, float(np.max(np.abs(data.xpos[body.id] - pose[:3, 3]))))
            rotation_error = max(rotation_error, float(np.max(np.abs(data.xmat[body.id].reshape(3, 3) - pose[:3, :3]))))
        row = dict(position_error_m=position_error, rotation_error=rotation_error,
                   gravity_error_nm=float(np.max(np.abs(actual_g - expected_g))),
                   mass_matrix_relative_error=float(np.linalg.norm(actual_m - expected_m) / np.linalg.norm(expected_m)))
        for key, value in row.items():
            metrics[key] = max(metrics[key], value)
        assert np.linalg.eigvalsh(full_m).min() > 0
        assert np.max(np.abs(full_m[np.ix_(order[:9], order[9:])])) < 1e-12
        per_pose.append(row)
    tolerances = dict(position_error_m=2e-6, rotation_error=1e-5,
                      gravity_error_nm=1e-4, mass_matrix_relative_error=1e-4)
    passed = all(metrics[key] <= threshold for key, threshold in tolerances.items())
    result = dict(passed=passed, stage='mechanics', mujoco=mj.__version__, model_commit=manifest['commit'],
                  nq=model.nq, nv=model.nv, independent_commands=16, neq=model.neq,
                  visual_collision_geometries=model.ngeom, meshes=model.nmesh,
                  urdf_mass_including_fixed_torso_kg=source_mass,
                  mjcf_arm_gripper_mass_kg=float(model.body_mass.sum()),
                  fixed_base_com_discrepancy_m=0.005063, stock_position_interface=True,
                  tested_poses=len(poses_to_check), worst_errors=metrics, tolerances=tolerances,
                  source_sha256={name: robot.sha256(robot.ROOT / name) for name in
                                 ['robot.py', 'validate_mechanics.py', 'assets-manifest.json']},
                  mapping=mapping, pose_results=per_pose,
                  model_accuracy_qualified=False, custom_gim_drives=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps({key: result[key] for key in ['passed', 'tested_poses', 'worst_errors', 'nq', 'independent_commands']}))
    assert passed, 'Independent mechanics check failed; inspect retained JSON before a solve'
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=robot.Path, default=robot.ROOT / 'results' / 'mechanics.json')
    validate(parser.parse_args().output.resolve())
