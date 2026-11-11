"""Writes sw_shoulder.plecs: a standalone PLECS model of the switching-level J1 (shoulder, GIM8115) actuator board.

Power stage: bank voltage -> one daisy-chain hop (r_hop, l_hop) -> board node with C_local (+ ESR) -> six MOSFETs with
body diodes (ron each) -> phase ammeters -> PMSM -> torque sensor -> imposed shaft speed.
Control: C-Script "FOC" sampled once per PWM period at the carrier valley: Park transform of the sampled phase currents,
dq PI with decoupling and back-EMF feedforward (same gains as ctrl_fcn.m), SVPWM by min-max injection, angle advanced
1.5 periods; the duty is applied one period after it is sampled. It also replays the averaged model's J1 motor speed,
iq_ref and bank voltage from tables. Triangle carrier 0..1 at fsw; for each phase the high FET is on while
d - td*fsw > carrier and the low FET while carrier > d + td*fsw, so every edge has a dead time td.
Top-level outputs (returned by plecs.simulate): 1 integrals of [i_board, v_board*i_board, tau_m, i_cap^2, ia^2,
i_board^2]; 2 [id iq iq_ref]; 3 raw [ia ib ic i_board v_board i_cap tau_m].
All parameters are initialization variables, overridden through ModelVars (run_sw_shoulder.py).
Usage: python plecs_shoulder_model.py <out.plecs>
"""
import sys

INIT = {  # defaults: GIM8115 J1, 100 kHz, 100 ns dead time; replaced from openarm_params by the runner
    'pp': 21, 'Rs': 0.49, 'Ls': 2.75e-4, 'lam': 0.01083, 'Jm': 1e-6, 'wm0': 0, 'kp_i': 1.728, 'ki_i': 3079,
    'v_frac': 0.5485, 'ron': 9e-3, 'fsw': 100e3, 'td': 100e-9, 'r_hop': 0.01, 'l_hop': 2e-7, 'c_local': 76e-6,
    'esr_local': 5e-3, 'v0': 48, 'ref_t': '[0 1]', 'ref_wm': '[0 0]', 'ref_iq': '[0 0]', 'ref_vb': '[48 48]',
}

comps, nets, where = [], [], {}


def q(s):
    """PLECS string literal; long strings are split the way PLECS writes them."""
    s = s.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n').replace('\t', '\\t')
    parts = [s[i:i + 70] for i in range(0, len(s), 70)] or ['']
    return '\n'.join(f'"{p}"' for p in parts)


def grid():
    """Components sit on a golden-angle spiral with irregular integer coordinates, so no straight wire between two of
    them runs exactly over a third one's terminals (PLECS rejects overlapped terminals). The schematic is legible only
    as a netlist."""
    import math
    k = len(comps)
    r, a = 600 + 97 * k, 2.39996323 * k
    return int(4000 + r * math.cos(a)) * 2 + 1, int(4000 + r * math.sin(a)) * 2 + 1


def place(name):
    where[name] = grid()
    return where[name]


def comp(ctype, name, pos, params=(), extra=''):
    pos = place(name)
    p = ''.join(f'      Parameter {{\n        Variable      "{k}"\n        Value         {q(str(v))}\n'
                f'        Show          off\n      }}\n' for k, v in params)
    comps.append(f'    Component {{\n      Type          {ctype}\n      Name          "{name}"\n      Show          on\n'
                 f'      Position      [{pos[0]}, {pos[1]}]\n      Direction     up\n      Flipped       off\n'
                 f'      LabelPosition east\n{extra}{p}    }}\n')


def conn(kind, *terms):
    """One net: every (component, terminal) pair given joins the same node or signal. Repeated calls that share a
    terminal are merged, because PLECS keeps one wire per terminal and joins nodes with Branches."""
    terms = list(terms)
    hit = [n for n in nets if any(t in n[1] for t in terms)]
    merged = (kind, [])
    for n in hit:
        nets.remove(n)
        merged[1].extend(t for t in n[1] if t not in merged[1])
    merged[1].extend(t for t in terms if t not in merged[1])
    nets.append(merged)


def emit_nets():
    out = []
    for kind, terms in nets:
        (src, st), rest = terms[0], terms[1:]
        if len(rest) == 1:
            body = f'      DstComponent  "{rest[0][0]}"\n      DstTerminal   {rest[0][1]}\n'
        else:
            x, y = where[src]
            body = f'      Points        [{x + 45}, {y + 45}]\n' + ''.join(f'      Branch {{\n        DstComponent  "{c}"\n        DstTerminal   {t}\n      }}\n'
                           for c, t in rest)
        out.append(f'    Connection {{\n      Type          {kind}\n      SrcComponent  "{src}"\n'
                   f'      SrcTerminal   {st}\n{body}    }}\n')
    return ''.join(out)


# ---------------- power stage (two-terminal parts: 1 = +, 2 = -, 3 = signal; FET: 1 drain, 2 source, 3 gate)
comp('VoltageSource', 'Vbank', (100, 450), [('DiscretizationBehavior', '2'), ('StateSpaceInlining', '1')])
comp('Ground', 'Gnd', (100, 560))
comp('Ammeter', 'Am_board', (160, 400))
comp('Resistor', 'R_hop', (220, 400), [('R', 'r_hop')])
comp('Inductor', 'L_hop', (280, 400), [('L', 'l_hop'), ('i_init', '0')])
comp('Voltmeter', 'Vm_board', (360, 470))
comp('Capacitor', 'C_local', (420, 450), [('C', 'c_local'), ('v_init', 'v0')])
comp('Resistor', 'R_esr', (420, 500), [('R', 'esr_local')])
comp('Ammeter', 'Am_cap', (420, 540))
GND = ('Gnd', 1)
NODE = ('L_hop', 2)
conn('Wire', ('Vbank', 1), ('Am_board', 1))
conn('Wire', ('Am_board', 2), ('R_hop', 1))
conn('Wire', ('R_hop', 2), ('L_hop', 1))
conn('Wire', NODE, ('Vm_board', 1), ('C_local', 1))
conn('Wire', ('C_local', 2), ('R_esr', 1))
conn('Wire', ('R_esr', 2), ('Am_cap', 1))
conn('Wire', GND, ('Vbank', 2), ('Vm_board', 2), ('Am_cap', 2))
fet = [('Ron', 'ron'), ('s_init', '0'), ('thermal', ''), ('Rth', '0'), ('Np', '1'), ('T_init', '')]
for k, ph in enumerate('abc'):
    x = 520 + 90 * k
    comp('MosfetWithDiode', f'Q{ph}H', (x, 300), fet)
    comp('MosfetWithDiode', f'Q{ph}L', (x, 420), fet)
    comp('Ammeter', f'Am_{ph}', (x + 40, 360))
    conn('Wire', NODE, (f'Q{ph}H', 1))
    conn('Wire', (f'Q{ph}H', 2), (f'Q{ph}L', 1), (f'Am_{ph}', 1))
    conn('Wire', GND, (f'Q{ph}L', 2))
comp('Constant', 'vb_const', (0,0), [('Value', '48'), ('DataType', '10')])
conn('Signal', ('vb_const', 1), ('Vbank', 3))
for k, (ph, g) in enumerate([('a', (1, 0)), ('b', (0, 1)), ('c', (0, 1))]):
    comp('Constant', f'gH_{ph}', (0,0), [('Value', str(g[0])), ('DataType', '10')])
    comp('Constant', f'gL_{ph}', (0,0), [('Value', str(g[1])), ('DataType', '10')])
    conn('Signal', (f'gH_{ph}', 1), (f'Q{ph}H', 3))
    conn('Signal', (f'gL_{ph}', 1), (f'Q{ph}L', 3))
    comp('Resistor', f'Rload_{ph}', (0,0), [('R', '1')])
    conn('Wire', (f'Am_{ph}', 2), (f'Rload_{ph}', 1))
    conn('Wire', (f'Rload_{ph}', 2), ('Rload_a', 2))
comp('ToFile', 'tofile', (0,0), [('Filename', 't_stage.csv'), ('FileType', '1'), ('WriteSignalNames', '2'), ('SampleTime', '1e-4')])
comp('SignalMux', 'm', (0,0), [('Width', '3')])
conn('Signal', ('Vm_board', 3), ('m', 2)); conn('Signal', ('Am_a', 3), ('m', 3)); conn('Signal', ('Am_board', 3), ('m', 4))
comp('SignalMux', 'm2', (0,0), [('Width', '2')])
conn('Signal', ('Vm_board', 3), ('m2', 2)); conn('Signal', ('Am_a', 3), ('m2', 3))
comp('Function', 'fsum', (0,0), [('f(u)', 'u[1]+1000*u[2]')])
conn('Signal', ('m2', 1), ('fsum', 1))
comp('SignalMux', 'm3', (0,0), [('Width', '[3 1]')])
conn('Signal', ('m', 1), ('m3', 2)); conn('Signal', ('fsum', 2), ('m3', 3))
conn('Signal', ('m3', 1), ('tofile', 1))
init = '\n'.join(f'{k} = {v};' for k, v in INIT.items())
HEADER = f'''Plecs {{
  Name          "sw_shoulder"
  Version       "5.0"
  CircuitModel  "ContStateSpace"
  StartTime     "0.0"
  TimeSpan      "0.03"
  Timeout       ""
  Solver        "dopri"
  MaxStep       "1e-6"
  InitStep      "-1"
  FixedStep     "1e-3"
  Refine        "1"
  ZCStepSize    "1e-9"
  RelTol        "1e-3"
  AbsTol        "-1"
  TurnOnThreshold "1e-3"
  SyncFixedStepTasks "2"
  UseSingleCommonBaseRate "2"
  LossVariableLimitExceededMsg "3"
  NegativeSwitchLossMsg "3"
  DivisionByZeroMsg "2"
  StiffnessDetectionMsg "2"
  MaxConsecutiveZCs "1000"
  AlgebraicLoopWithStateMachineMsg "2"
  AssertionAction "1"
  InitializationCommands {q(init)}
  InitialState  "1"
  SystemState   ""
  TaskingMode   "1"
  TaskConfigurations ""
  CodeGenParameterInlining "2"
  CodeGenFloatingPointFormat "2"
  CodeGenAbsTimeUsageMsg "3"
  CodeGenBaseName ""
  CodeGenOutputDir ""
  CodeGenExtraOpts ""
  CodeGenTarget "Generic"
  CodeGenTargetSettings ""
  ExtendedMatrixPrecision "1"
  MatrixSignificanceCheck "2"
  EnableStateSpaceSplitting "2"
  DisplayStateSpaceSplitting "1"
  DiscretizationMethod "2"
  ExternalModeSettings ""
  AlgebraicLoopMethod "1"
  AlgebraicLoopTolerance "1e-6"
  ScriptsDialogGeometry ""
  ScriptsDialogSplitterPos "0"
  Schematic {{
    Location      [0, 53; 1300, 700]
    ZoomFactor    1
    SliderPosition [0, 0]
    ShowBrowser   off
    BrowserWidth  100
'''
open(sys.argv[1], 'w', newline='\n').write(HEADER + ''.join(comps) + emit_nets() + '  }\n}\n')
print(f'{len(comps)} components, {len(nets)} nets')
