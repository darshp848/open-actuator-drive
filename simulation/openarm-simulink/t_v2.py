"""Writes sw_shoulder.plecs: a standalone PLECS model of the switching-level J1 (shoulder, GIM8115) actuator board.

Power stage: bank voltage -> one daisy-chain hop (r_hop, l_hop) -> board node with C_local (+ ESR) -> six MOSFETs with
body diodes (ron each) -> phase ammeters -> PMSM -> torque sensor -> imposed shaft speed.
Control: C-Script "FOC" sampled once per PWM period at the carrier valley: Park transform of the sampled phase currents,
dq PI with decoupling and back-EMF feedforward (same gains as ctrl_fcn.m), SVPWM by min-max injection, angle advanced
1.5 periods; the duty is applied one period after it is sampled. It also replays the averaged model's J1 motor speed,
iq_ref and bank voltage from tables. Triangle carrier 0..1 at fsw; for each phase the high FET is on while
d - td*fsw > carrier and the low FET while carrier > d + td*fsw, so every edge has a dead time td.
Logged to CSV next to the model: sw_avg.csv each PWM period = integrals of [i_board, v_board*i_board, i_cap^2, ia^2,
i_board^2] and the controller's sampled [id iq iq_ref]; sw_raw.csv every raw_ts = [ia ib ic i_board v_board i_cap
theta_m]. Motor torque is 1.5 p lambda iq (Ld = Lq): per period from the sampled iq (the valley sample of symmetric
center-aligned PWM is the period average), and its ripple from the raw currents and angle.
All parameters are initialization variables, overridden through ModelVars (run_sw_shoulder.py).
Usage: python plecs_shoulder_model.py <out.plecs>
"""
import sys

INIT = {  # defaults: GIM8115 J1, 100 kHz, 100 ns dead time; replaced from openarm_params by the runner
    'pp': 21, 'Rs': 0.49, 'Ls': 2.75e-4, 'lam': 0.01083, 'Jm': 1e-4, 'c_couple': 1e4, 'wm0': 0, 'kp_i': 1.728, 'ki_i': 3079,
    'v_frac': 0.5485, 'ron': 9e-3, 'fsw': 100e3, 'td': 100e-9, 'r_hop': 0.01, 'l_hop': 2e-7, 'c_local': 76e-6,
    'esr_local': 5e-3, 'v0': 48, 'raw_ts': 1e-6, 'ref_t': '[0 1]', 'ref_wm': '[0 0]', 'ref_iq': '[0 0]', 'ref_vb': '[48 48]',
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
    """One net: every (component, terminal) pair given joins the same node or signal; for a signal the first pair is
    the driver. Calls that share a terminal are merged into one net."""
    terms = list(terms)
    hit = [n for n in nets if any(t in n[1] for t in terms)]
    merged = (kind, [])
    for n in hit:
        nets.remove(n)
        merged[1].extend(t for t in n[1] if t not in merged[1])
    merged[1].extend(t for t in terms if t not in merged[1])
    nets.append(merged)


def wire(kind, a, b):
    return (f'    Connection {{\n      Type          {kind}\n      SrcComponent  "{a[0]}"\n      SrcTerminal   {a[1]}\n'
            f'      DstComponent  "{b[0]}"\n      DstTerminal   {b[1]}\n    }}\n')


def tag_params(tag, missing):
    return [('Tag', tag), ('Visibility', '2'), ('NoMatchingCounterpartAction', missing), ('CustomMessage', ''),
            ('HighlightLevel', '0')]


def emit_nets():
    """Two-terminal nets are one wire. Larger electrical nets join through Electrical Labels and larger signal nets
    through one Signal Goto (at the driver) and Signal Froms, each with its own two-point wire: PLECS did not resolve
    branched connections in this generated (non-drawn) schematic reliably. Rotational nets keep branches."""
    out = []
    for n, (kind, terms) in enumerate(nets):
        if len(terms) == 2:
            out.append(wire(kind, terms[0], terms[1]))
        elif kind == 'Wire':
            for k, t in enumerate(terms):
                comp('Label', f'net{n}_{k}', (0, 0), tag_params(f'net{n}', '2'))
                out.append(wire(kind, t, (f'net{n}_{k}', 1)))
        elif kind == 'Signal':
            comp('Goto', f'net{n}_go', (0, 0), tag_params(f'sig{n}', '2'))
            out.append(wire(kind, terms[0], (f'net{n}_go', 1)))
            for k, t in enumerate(terms[1:]):
                comp('From', f'net{n}_{k}', (0, 0), tag_params(f'sig{n}', '1'))
                out.append(wire(kind, (f'net{n}_{k}', 1), t))
        else:
            (src, st), rest = terms[0], terms[1:]
            x, y = where[src]
            body = f'      Points        [{x + 45}, {y + 45}]\n' + ''.join(
                f'      Branch {{\n        DstComponent  "{c}"\n        DstTerminal   {t}\n      }}\n' for c, t in rest)
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
    conn('Wire', (f'Am_{ph}', 2), ('PMSM', k + 1))
comps.append('    Component {\n      Type          Reference\n'
             '      SrcComponent  "Components/Electrical/Machines/Perm.-Magnet SM"\n      Name          "PMSM"\n'
             '      Show          on\n      Position      [%d, %d]\n' % place('PMSM')
             + '      Direction     up\n      Flipped       off\n'
             '      LabelPosition east\n      Frame         [-25, -25; 25, 35]\n'
             + ''.join(f'      Parameter {{\n        Variable      "{k}"\n        Value         "{v}"\n'
                       f'        Show          off\n      }}\n'
                       for k, v in [('configuration', '2'), ('R', 'Rs'), ('L', '[Ls Ls]'), ('phi', 'lam'), ('J', 'Jm'),
                                    ('F', '0'), ('p', 'pp'), ('wm0', 'wm0'), ('thm0', '0'), ('is0', '[0 0]')])
             + ''.join(f'      Terminal {{\n        Type          {t}\n        Position      [{p}]\n'
                       f'        Direction     {d}\n      }}\n'
                       for t, p, d in [('Port', '-30, -10', 'left'), ('Port', '-30, 0', 'left'),
                                       ('Port', '-30, 10', 'left'), ('Rotational', '30, 30', 'right')])
             + '    }\n')
# imposed speed: a speed source (referenced to the frame, SecondFlange 1: terminal 1 flange, 2 signal) drives the
# rotor through a stiff damper, since PLECS does not allow a speed source directly on the rotor inertia state.
# Speed error = torque / c_couple (7 Nm / 1e4 Nms = 0.0007 rad/s); this is why the solver is Radau.
comp('ControlledRotationalSpeed', 'Spd', (980, 450),
     [('SecondFlange', '1'), ('DiscretizationBehavior', '1'), ('StateSpaceInlining', '1')])
comp('RotationalDamper', 'couple', (0, 0), [('D', 'c_couple'), ('theta0', '')])
comp('AngleSensor', 'Ang', (920, 300), [('SecondFlange', '1'), ('theta0', '0')])
comp('AngleSensor', 'Ang_ctrl', (0, 0), [('SecondFlange', '1'), ('theta0', '0')])   # controller's encoder
conn('Rotational', ('PMSM', 4), ('couple', 1), ('Ang', 1), ('Ang_ctrl', 1))
conn('Rotational', ('couple', 2), ('Spd', 1))

# ---------------- controller: C-Script, inputs [ia ib ic theta v_board], outputs [da db dc wm vbank iq_ref id iq]
DECL = r'''#include <math.h>
#define P(i) ParamRealData(i, 0)
#define PP P(0)
#define RS P(1)
#define LS P(2)
#define LAM P(3)
#define KP P(4)
#define KI P(5)
#define VFRAC P(6)
#define TS P(7)
#define NREF ParamNumElements(8)
#define REFT(k) ParamRealData(8, k)
#define REFW(k) ParamRealData(9, k)
#define REFI(k) ParamRealData(10, k)
#define REFV(k) ParamRealData(11, k)
static double zd, zq, dn[3], out_wm, out_vb, out_iqr, out_id, out_iq;
static int kref;
static void lookup(double t, double *w, double *iqr, double *vb)
{
    while (kref < NREF - 2 && REFT(kref + 1) <= t) kref++;
    double t0 = REFT(kref), t1 = REFT(kref + 1);
    double f = (t1 > t0) ? (t - t0)/(t1 - t0) : 0.0;
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    *w = REFW(kref) + f*(REFW(kref + 1) - REFW(kref));
    *iqr = REFI(kref) + f*(REFI(kref + 1) - REFI(kref));
    *vb = REFV(kref) + f*(REFV(kref + 1) - REFV(kref));
}
'''
START = r'''zd = 0; zq = 0; kref = 0; dn[0] = dn[1] = dn[2] = 0.5; out_id = 0; out_iq = 0;
lookup(0.0, &out_wm, &out_iqr, &out_vb);
'''
OUTPUT = r'''OutputSignal(0, 0) = dn[0]; OutputSignal(0, 1) = dn[1]; OutputSignal(0, 2) = dn[2];
OutputSignal(0, 3) = out_wm; OutputSignal(0, 4) = out_vb; OutputSignal(0, 5) = out_iqr;
OutputSignal(0, 6) = out_id; OutputSignal(0, 7) = out_iq;
'''
UPDATE = r'''double ia = InputSignal(0, 0), ib = InputSignal(0, 1), ic = InputSignal(0, 2);
double thm = InputSignal(0, 3), vb = InputSignal(0, 4);
double t = CurrentTime;
lookup(t + TS, &out_wm, &out_iqr, &out_vb);
double th = PP*thm, we = PP*out_wm, a = 2.0*M_PI/3.0;
double id = 2.0/3.0*(ia*cos(th) + ib*cos(th - a) + ic*cos(th + a));
double iq = -2.0/3.0*(ia*sin(th) + ib*sin(th - a) + ic*sin(th + a));
double ed = -id, eq = out_iqr - iq;
double vd = KP*ed + zd - we*LS*iq;
double vq = KP*eq + zq + we*LAM + we*LS*id;
double vmax = VFRAC*(vb > 1 ? vb : 1);
double vm = sqrt(vd*vd + vq*vq);
double s = (vm > vmax) ? vmax/vm : 1.0;
vd *= s; vq *= s;
if (s >= 0.999) { zd += KI*ed*TS; zq += KI*eq*TS; }
double tha = th + 1.5*we*TS;
double v[3], vmx, vmn;
v[0] = vd*cos(tha) - vq*sin(tha);
v[1] = vd*cos(tha - a) - vq*sin(tha - a);
v[2] = vd*cos(tha + a) - vq*sin(tha + a);
vmx = fmax(v[0], fmax(v[1], v[2])); vmn = fmin(v[0], fmin(v[1], v[2]));
for (int k = 0; k < 3; k++) {
    double d = 0.5 + (v[k] - 0.5*(vmx + vmn))/(vb > 1 ? vb : 1);
    dn[k] = d < 0 ? 0 : (d > 1 ? 1 : d);
}
out_id = id; out_iq = iq;
'''
comp('CScript', 'FOC', (300, 150), [
    ('DialogGeometry', ''), ('NumInputs', '5'), ('NumOutputs', '8'), ('NumContStates', '0'), ('NumDiscStates', '1'),
    ('NumZCSignals', '0'), ('DirectFeedthrough', '0'), ('Ts', '1/fsw'), ('TerminalBasedSampleTimes', '1'),
    ('Parameters', 'pp, Rs, Ls, lam, kp_i, ki_i, v_frac, 1/fsw, ref_t, ref_wm, ref_iq, ref_vb'),
    ('LangStandard', '2'), ('GnuExtensions', '2'), ('RuntimeCheck', '2'), ('HighlightLevel', '0'),
    ('Declarations', DECL), ('StartFcn', START), ('OutputFcn', OUTPUT), ('UpdateFcn', UPDATE),
    ('DerivativeFcn', ''), ('TerminateFcn', ''), ('StoreCustomStateFcn', ''), ('RestoreCustomStateFcn', '')])
comp('SignalMux', 'meas_mux', (0, 0), [('Width', '[1 1 1 1 1]')])
comp('SignalDemux', 'foc_dm', (400, 150), [('Width', '8')])
for k, (c, t) in enumerate([('Am_a', 3), ('Am_b', 3), ('Am_c', 3), ('Ang_ctrl', 2), ('Vm_board', 3)]):
    conn('Signal', (c, t), ('meas_mux', k + 2))
conn('Signal', ('meas_mux', 1), ('FOC', 1))
comp('Constant', 'FOCk', (0,0), [('Value', '[0.5 0.5 0.5 100 48 0 0 0]'), ('DataType', '10')])
conn('Signal', ('FOCk', 1), ('foc_dm', 1))
comp('ToFile', 'focf', (0,0), [('Filename', 't_foc.csv'), ('FileType', '1'), ('WriteSignalNames', '2'), ('SampleTime', '1e-5')])
conn('Signal', ('FOC', 2), ('focf', 1))
conn('Signal', ('foc_dm', 5), ('Spd', 2))
conn('Signal', ('foc_dm', 6), ('Vbank', 3))

for k, (ph, g) in enumerate([('a', (1, 0)), ('b', (0, 1)), ('c', (0, 1))]):
    comp('Constant', f'gH_{ph}', (0,0), [('Value', str(g[0])), ('DataType', '10')])
    comp('Constant', f'gL_{ph}', (0,0), [('Value', str(g[1])), ('DataType', '10')])
    conn('Signal', (f'gH_{ph}', 1), (f'Q{ph}H', 3))
    conn('Signal', (f'gL_{ph}', 1), (f'Q{ph}L', 3))
init = '\n'.join(f'{k} = {v};' for k, v in INIT.items())
HEADER = f'''Plecs {{
  Name          "sw_shoulder"
  Version       "5.0"
  CircuitModel  "ContStateSpace"
  StartTime     "0.0"
  TimeSpan      "0.03"
  Timeout       ""
  Solver        "radau"
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
wires = emit_nets()   # adds the label / goto / from components first
open(sys.argv[1], 'w', newline='\n').write(HEADER + ''.join(comps) + wires + '  }\n}\n')
print(f'{len(comps)} components, {len(nets)} nets')
