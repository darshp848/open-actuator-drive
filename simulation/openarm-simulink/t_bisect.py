"""Builds variants of sw_shoulder.plecs from chosen generator sections and reports whether each simulates."""
import os, re, subprocess, sys, xmlrpc.client as x
L = open('plecs_shoulder_model.py').read().split('\n')
mark = {n: next(i for i, l in enumerate(L) if l.startswith(n)) for n in
        ['# ---------------- power stage', '# ---------------- controller', '# ---------------- carrier',
         '# ---------------- measurements', 'init = ']}
sec = {'power': (mark['# ---------------- power stage'], mark['# ---------------- controller']),
       'ctrl': (mark['# ---------------- controller'], mark['# ---------------- carrier']),
       'pwm': (mark['# ---------------- carrier'], mark['# ---------------- measurements']),
       'meas': (mark['# ---------------- measurements'], mark['init = '])}
const_gates = '''for k, (ph, g) in enumerate([('a', (1, 0)), ('b', (0, 1)), ('c', (0, 1))]):
    comp('Constant', f'gH_{ph}', (0,0), [('Value', str(g[0])), ('DataType', '10')])
    comp('Constant', f'gL_{ph}', (0,0), [('Value', str(g[1])), ('DataType', '10')])
    conn('Signal', (f'gH_{ph}', 1), (f'Q{ph}H', 3))
    conn('Signal', (f'gL_{ph}', 1), (f'Q{ph}L', 3))'''
const_src = '''comp('Constant', 'vb_const', (0,0), [('Value', '48'), ('DataType', '10')])
conn('Signal', ('vb_const', 1), ('Vbank', 3))
comp('Constant', 'wm_const', (0,0), [('Value', '100'), ('DataType', '10')])
conn('Signal', ('wm_const', 1), ('Spd', 2))'''
s = x.ServerProxy('http://localhost:1080/RPC2')
for variant in sys.argv[1:]:
    parts = variant.split('+')
    body = L[:mark['# ---------------- power stage']]
    for p in parts:
        body += L[sec[p][0]:sec[p][1]]
    if 'pwm' not in parts:
        body += const_gates.split('\n')
    if 'ctrl' not in parts:
        body += const_src.split('\n')
    body += L[mark['init = ']:]
    open('t_var.py', 'w').write('\n'.join(body))
    subprocess.run([sys.executable, 't_var.py', 't_var.plecs'], check=True, capture_output=True)
    t = open('t_var.plecs').read().replace('Name          "sw_shoulder"', 'Name          "t_var"', 1)
    open('t_var.plecs', 'w').write(t)
    s.plecs.load(os.path.abspath('t_var.plecs'))
    try:
        s.plecs.simulate('t_var', {'SolverOpts': {'StopTime': 2e-4}})
        print(variant, 'OK')
    except x.Fault as e:
        print(variant, 'FAULT', re.sub(r'\s+', ' ', e.faultString)[:300])
    s.plecs.close('t_var')
