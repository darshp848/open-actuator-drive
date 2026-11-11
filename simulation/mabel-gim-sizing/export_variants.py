"""Write one MuJoCo robot model per actuator set: MABEL's own mabel_full.xml with each joint's actuator swapped.
Run: MABEL_DESIGNS=designs_coupled python -I -B export_variants.py
Each file keeps MABEL's kinematics, meshes, lift, hands, neck and swerve base unchanged. For the torso and 14 arm joints it carries the
set's body mass, centre of mass and inertia (including gearbox mass), armature (CAD rotor + gearbox input, reflected), Coulomb and viscous
friction, and the actuator force limit. Actuators stay as position servos with MABEL's gains: the electrical model (current loop, 48 V limit)
lives in rig.py / rig2.py and is not in the XML. Reads the MABEL XML only; never edits it."""
import json, os, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mujoco as mj
import numpy as np
import rig2
from rig import JN, XML

OUT = HERE / 'variants'
OUT.mkdir(exist_ok=True)
NOTE = """<!-- Actuator variant '{name}' of MABEL (Qilong Cheng, Rooholla Khorrambakht, Mohamed Yassine Kabouri, Quang Nam Nguyen, Luca Macesanu,
     Haoran Zhou, Yipeng Wang, Ludovic Righetti; arXiv 2611.04083, github.com/robotmabel/open_mabel commit 2f9229f, hardware CERN-OHL-S-2.0).
     Only torso and arm actuator mass, inertia, armature, friction and force limit differ from the original; gearbox mass and input inertia included.
     The swerve drive base is unchanged: it was outside the design scope of this feasibility study.
     Actuator lump centre of mass sits at the joint's child body origin, axis on the joint axis. geoms named housing_* are visual cylinders (OD x length). {kind}. -->
"""
mdir = Path(XML).parent
COL = dict(ndfeb=[0.80, 0.25, 0.20, 1], ferrite=[0.20, 0.40, 0.80, 1])
ORDER = dict(Torso='Torso', **{'%s Arm %d' % (sd, i): 'Arm%d' % i for sd in ('Left', 'Right') for i in range(1, 8)})
DESIGN = {}


def add_housings(spec, m, D, rgba):
    """Visual-only cylinder per actuator (OD x length on the joint axis), attached to the joint's parent body. No mass, no contact."""
    for jn in JN:
        e = D['joints'][ORDER[jn]]
        jid = m.joint(jn).id
        child = m.jnt_bodyid[jid]
        parent = m.body_parentid[child]
        R = np.zeros(9)
        mj.mju_quat2Mat(R, m.body_quat[child])
        R = R.reshape(3, 3)
        p = m.body_pos[child] + R @ m.jnt_pos[jid]
        ax = R @ m.jnt_axis[jid]
        q = np.zeros(4)
        mj.mju_quatZ2Vec(q, ax / np.linalg.norm(ax))
        spec.body(m.body(int(parent)).name).add_geom(name='housing_' + jn.replace(' ', '_'), type=mj.mjtGeom.mjGEOM_CYLINDER,
            size=[e['OD_mm'] / 2e3, e['length_mm'] / 2e3, 0], pos=p, quat=q, rgba=rgba, contype=0, conaffinity=0, group=2, mass=0)


rows = []
names = ['stock_gim_nominal'] + sorted(p.stem for p in Path(rig2.DESIGNS).glob('*.json') if isinstance(json.loads(p.read_text()).get('joints') if isinstance(json.loads(p.read_text()), dict) else None, dict))
for name in names:
    if name != 'stock_gim_nominal':
        DESIGN[name] = json.loads((Path(rig2.DESIGNS) / (name + '.json')).read_text())
    a = rig2.get_set(name)
    r = rig2.SetRig(a)
    m = r.m
    spec = mj.MjSpec.from_file(XML)
    spec.compiler.balanceinertia = 1
    if name in DESIGN:
        add_housings(spec, m, DESIGN[name], COL[DESIGN[name]['magnet']])
    viol = []
    for jj, jn in enumerate(JN):
        b = int(m.jnt_bodyid[m.joint(jn).id]); bn = m.body(b).name
        sb = spec.body(bn)
        sb.mass, sb.ipos, sb.inertia, sb.iquat = float(m.body_mass[b]), m.body_ipos[b].copy(), m.body_inertia[b].copy(), m.body_iquat[b].copy()
        sb.explicitinertial = 1
        A = np.sort(m.body_inertia[b])
        if A[0] + A[1] < A[2]:
            viol.append((bn, A.round(5).tolist()))
        sj = spec.joint(jn)
        d = m.joint(jn).dofadr[0]
        sj.armature, sj.damping, sj.frictionloss = float(m.dof_armature[d]), float(m.dof_damping[d]), float(m.dof_frictionloss[d])
        sa = spec.actuator(jn)
        t = float(r.P['tau_max'][jj])
        sa.forcerange = [-t, t]
    xml = spec.to_xml()
    new = Path(os.path.relpath(mdir, OUT)).as_posix()
    assert '<compiler ' in xml and 'meshdir=' not in xml
    xml = xml.replace('<compiler ', f'<compiler meshdir="{new}" balanceinertia="true" ', 1)
    path = OUT / f'{name}.xml'
    i = xml.index('\n') + 1 if xml.startswith('<?xml') else 0
    path.write_text(xml[:i] + NOTE.format(name=name, kind=a.get('meta', {}).get('case', 'stock GIM at the nominal 0.25 / 0.14 kg m2 inertia')) + xml[i:], encoding='utf-8')
    # check: reload and compare
    m2 = mj.MjModel.from_xml_path(str(path))
    d2 = mj.MjData(m2); mj.mj_resetDataKeyframe(m2, d2, 0)
    for _ in range(200):
        mj.mj_step(m2, d2)
    ok_finite = bool(np.isfinite(d2.qpos).all())
    errs = [abs(m2.body_mass[m2.joint(j).bodyid[0]] - m.body_mass[m.joint(j).bodyid[0]]) for j in JN]
    arm = [abs(m2.dof_armature[m2.joint(j).dofadr[0]] - m.dof_armature[m.joint(j).dofadr[0]]) for j in JN]
    rows.append(dict(set=name, total_mass_kg=round(float(m2.body_mass.sum()), 3), nominal_rig_total_kg=round(r.total_mass, 3), max_body_mass_err=float(max(errs)), max_armature_err=float(max(arm)), stable_200_steps=ok_finite, nu=int(m2.nu), inertia_balanced=viol))
    print(rows[-1], flush=True)
(OUT / 'index.json').write_text(json.dumps(rows, indent=1))
