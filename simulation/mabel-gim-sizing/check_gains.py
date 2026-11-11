"""PD stability check for the GIM layer on MABEL: small position steps, one joint at a time, at the worst pose.
Writes results/<timestamp>_gains/gains.json. Run: python -I -B mabel_gim/check_gains.py"""
import json, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np
from rig import *


def pose(torso):
    q = np.zeros(15)
    for n, v in WORST.items():
        q[JN.index(n)] = v
    q[0] = torso
    return q


def step_test(payload, j, amp, torso=-0.6, T=1.2, t_step=0.3, kp=None, kd=None):
    rig = Rig(payload, 'gim', Config(kp=kp, kd=kd) if kp is not None else None)
    q0 = pose(torso)
    rig.set_pose(q0)
    g = rig.inverse_torque(q0, 0 * q0, 0 * q0, 0.3175)
    rig.preload(q0, g)
    for k in range(round(T / 1e-3)):
        qr = q0.copy()
        if k * 1e-3 >= t_step:
            qr[j] += amp
        gg = rig.inverse_torque(qr, 0 * qr, 0 * qr, 0.3175)
        rig.step_ms(qr, 0 * qr, gg, 0.3175)
    L = rig.log()
    t = L['t']
    y = (L['q'][:, j] - q0[j]) / amp
    after = t >= t_step
    over = float(max(0., y[after].max() - 1.))
    tail = t > T - 0.3
    ripple = float(np.std(L['q'][tail, j] - L['q'][tail, j].mean()))
    final = float(y[tail].mean())
    # 2 % settle time
    out = np.abs(y - final) > 0.02 * max(abs(final), 1e-9)
    idx = np.flatnonzero(out & after)
    settle = float(t[idx[-1]] - t_step) if len(idx) else 0.
    # rise: 10-90 %
    try:
        t10 = t[after][np.argmax(y[after] >= 0.1 * final)]
        t90 = t[after][np.argmax(y[after] >= 0.9 * final)]
        rise = float(t90 - t10)
    except Exception:
        rise = float('nan')
    return dict(overshoot_frac=over, settle_2pct_s=settle, rise_10_90_s=rise, final_frac=final, tail_ripple_rad=ripple,
                peak_iq_over_imax=float(np.max(np.abs(L['iq'][:, j])) / rig.P['i_max'][j]))


def main():
    out = HERE / 'results' / (time.strftime('%Y%m%d_%H%M%S') + '_gains')
    out.mkdir(parents=True)
    res = dict(torso_kp=TORSO_KP, torso_kd=TORSO_KD, note='step 0.05 rad torso, 0.02 rad arms; worst-pose arms; torso at -0.6 rad',
               cases={})
    r0 = Rig(0., 'gim')
    res['arm_kp'] = dict(zip(JN[1:], r0.kp[1:].round(1).tolist()))
    res['arm_kd'] = dict(zip(JN[1:], r0.kd[1:].round(2).tolist()))
    for payload in (0., 3.):
        for j, n in enumerate(JN):
            amp = 0.05 if j == 0 else 0.02
            r = step_test(payload, j, amp)
            res['cases'][f'{payload:g}kg/{n}'] = r
            print(f'{payload:g}kg {n:12s} OS {r["overshoot_frac"]*100:5.1f}%  settle {r["settle_2pct_s"]*1e3:6.0f} ms  rise {r["rise_10_90_s"]*1e3:6.0f} ms  ripple {r["tail_ripple_rad"]*1e6:7.1f} urad  iq/imax {r["peak_iq_over_imax"]:.2f}')
    (out / 'gains.json').write_text(json.dumps(res, indent=2))
    print(out)


if __name__ == '__main__':
    main()
