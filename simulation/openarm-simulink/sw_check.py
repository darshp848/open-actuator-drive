"""Sanity check of sw_shoulder.plecs: 100 rad/s motor speed, iq_ref 0 -> 5 -> -5 A, 48 V bank.
Checks current-loop tracking, d-axis alignment (id ~ 0) and power balance: board power = shaft power + copper + FET."""
import csv
import os
import time
import xmlrpc.client as x

import numpy as np

pp, lam, Rs, ron = 21, 0.01083, 0.49, 9e-3
s = x.ServerProxy('http://localhost:1080/RPC2')
try:
    s.plecs.close('sw_shoulder')
except Exception:
    pass
s.plecs.load(os.path.abspath('sw_shoulder.plecs'))
mv = {'ref_t': [0.0, 0.01, 0.010001, 0.02, 0.020001, 0.03], 'ref_wm': [100.0] * 6,
      'ref_iq': [0.0, 0, 5, 5, -5, -5], 'ref_vb': [48.0] * 6, 'wm0': 100.0, 'raw_ts': 1e-5}
t0 = time.time()
s.plecs.simulate('sw_shoulder', {'ModelVars': mv, 'SolverOpts': {'StopTime': 0.03}})
print('wall %.1f s for 30 ms' % (time.time() - t0))
A = np.array(list(csv.reader(open('sw_avg.csv')))[1:], float)
t, I, dq, Ts = A[:, 0], A[:, 1:6], A[:, 6:9], 1e-5
dI = np.diff(I, axis=0) / Ts
for a, b in [(0.005, 0.0095), (0.015, 0.0195), (0.025, 0.0295)]:
    k = (t[1:] >= a) & (t[1:] < b)
    iq = dq[1:][k, 1].mean()
    p_b, ia_rms = dI[k, 1].mean(), np.sqrt(dI[k, 3].mean())
    shaft = 1.5 * pp * lam * iq * 100
    cu = 3 * ia_rms ** 2 * (Rs + ron)
    print(f'{a:.3f}-{b:.4f} s: iq_ref {dq[1:][k, 2].mean():5.2f} iq {iq:6.3f} id {dq[1:][k, 0].mean():6.3f} | '
          f'board {p_b:7.2f} W = shaft {shaft:7.2f} + copper/FET {cu:5.2f} + rest {p_b - shaft - cu:6.2f} | '
          f'C_local rms {np.sqrt(dI[k, 2].mean()):5.2f} A, board i rms {np.sqrt(dI[k, 4].mean()):5.2f} A')
