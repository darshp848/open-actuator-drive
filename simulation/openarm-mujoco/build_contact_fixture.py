"""Generate and compile a fixed gripper contact fixture without stepping."""
import json
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

import robot
from run_diagnostics import require_mechanics


def build():
    require_mechanics()
    base, initial = robot.load()
    wrist = base.body('openarm_right_ee_base_link').id
    center = initial.xpos[wrist] + initial.xmat[wrist].reshape(3, 3) @ np.array([-.005, 0., -.16])
    root = ET.Element('mujoco', model='openarm_v2_gripper_contact_fixture')
    ET.SubElement(root, 'option', timestep='0.001', integrator='implicitfast', cone='elliptic', impratio='10')
    assets = ET.SubElement(root, 'asset')
    ET.SubElement(assets, 'model', name='arm_pedestal',
                  file='../vendor/openarm_mujoco/v2/pedestal/openarm_pedestal.xml')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1', directional='true')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='2 2 .05', rgba='.22 .25 .3 1')
    ET.SubElement(world, 'attach', model='arm_pedestal', prefix='')
    ET.SubElement(world, 'geom', name='clamp_probe', type='sphere', size='.015',
                  pos=' '.join(f'{v:.17g}' for v in center), rgba='.95 .35 .12 1',
                  friction='1 .01 .01', solref='.005 1', solimp='.95 .99 .001', condim='4')
    keys = ET.SubElement(root, 'keyframe')
    ET.SubElement(keys, 'key', name='home', qpos=' '.join(f'{v:.17g}' for v in initial.qpos),
                  ctrl=' '.join(f'{v:.17g}' for v in initial.ctrl))
    ET.indent(root)
    target = robot.ROOT / 'models' / 'gripper_fixture.xml'
    target.parent.mkdir(exist_ok=True)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    model = mj.MjModel.from_xml_path(str(target))
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, 0)
    mj.mj_forward(model, data)
    object_id = model.geom('clamp_probe').id
    object_contacts = [float(c.dist) for c in data.contact if object_id in (c.geom1, c.geom2)]
    assert not object_contacts, 'Contact fixture begins with unexpected penetration'
    result = dict(compiled=True, no_simulation_run=True, probe_center_m=center.tolist(), radius_m=.015,
                  source_sha256=robot.sha256(target), initial_probe_contacts=len(object_contacts),
                  model_nq=model.nq, model_nu=model.nu)
    (robot.ROOT / 'results' / 'fixture.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    build()
