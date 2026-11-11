# COPY of simulation/openarm_v2/gim/params.py (copied unchanged below this header so that the hash-pinned
# openarm_v2 folder is never imported, which would write bytecode into it). Source SHA-256 of the original at
# copy time: see mabel_gim/README.md. Only edit the original, then re-copy.
"""GIM actuator parameters per OpenArm v2 joint (J1..J7, repeated for left and right arm).
Sources follow Codex_June26/.../system_model/openarm_params.m: SteadyWin product pages read 2026-10-25;
everything marked ASSUMED is not published."""
import numpy as np

# per joint J1..J7
N = np.array([36, 36, 36, 36, 40, 40, 40.])
KT_OUT = np.array([11.05, 11.05, 7.32, 7.32, 1.79, 1.79, 1.79])        # listed Nm per motor A at the output
I_STALL = np.array([21, 21, 19.8, 19.8, 17.1, 17.1, 17.1])
TAU_LISTED = np.array([228, 228, 109.5, 109.5, 30.59, 30.59, 30.59])    # listed stall torque, Nm
R_LL = np.array([0.98, 0.98, 0.22, 0.22, 0.27, 0.27, 0.27])
L_LL = np.array([0.55, 0.55, 0.21, 0.21, 0.8, 0.8, 0.8]) * 1e-3
PP = np.array([21, 21, 21, 21, 14, 14, 14.])
RPM_PER_V_OUT = np.array([0.75, 0.75, 1.25, 1.25, 3.58, 3.58, 3.58])    # listed output rpm per volt
W_MAX_LISTED = np.array([36, 36, 60, 60, 86, 86, 86.]) * 2 * np.pi / 60
V_RATED_MAX = np.array([60, 60, 60, 60, 40, 40, 40.])                    # GIM4310 rated 12-40 V; others not flagged
J_ROTOR = np.array([1.2e-4, 1.2e-4, 1.0e-4, 1.0e-4, 2.0e-5, 2.0e-5, 2.0e-5])   # ASSUMED kg m^2
TAU_COULOMB = np.array([0.8, 0.8, 0.6, 0.6, 0.15, 0.15, 0.15])          # ASSUMED, output Nm
B_VISC = np.array([0.5, 0.5, 0.4, 0.4, 0.05, 0.05, 0.05])               # ASSUMED, output Nm s/rad
K_GEAR = np.array([3000, 3000, 2000, 2000, 400, 400, 400.])             # ASSUMED gearbox stiffness, Nm/rad
MOTOR_MASS = np.array([0.920, 0.920, 0.720, 0.720, 0.337, 0.337, 0.337])
STOCK_MASS = np.array([0.80, 0.80, 0.36, 0.36, 0.30, 0.30, 0.30])       # ASSUMED stock Damiao masses
BACKLASH_RAD = 15 / 60 * np.pi / 180                                     # 15 arcmin full width (GIM8115 listing; ASSUMED for the rest)
R_INVERTER = 9e-3                                                        # ohm, board conduction loss for the DC power log
MOTOR_NAMES = ['GIM8115-36'] * 2 + ['GIM8108-36'] * 2 + ['GIM4310-40'] * 3
TILE = lambda a: np.tile(a, 2)


def arm_params(eta=0.9, lambda_source='kt'):
    """Return 14-vectors (left J1..J7 then right J1..J7) in robot.ARMS order."""
    assert lambda_source in ('kt', 'kv')
    kt_m = KT_OUT / (N * eta)
    if lambda_source == 'kt':
        lam = kt_m / (1.5 * PP)
    else:
        kv = N * RPM_PER_V_OUT * 2 * np.pi / 60                          # motor rad/s per volt
        lam = 1 / (np.sqrt(3) * kv) / PP                                 # pp*lambda = 1/(sqrt(3) Kv)
    i_max = np.minimum(I_STALL, TAU_LISTED / KT_OUT)                    # current limit so output torque stays <= listed
    p = dict(N=N, eta=np.full(7, eta), kt_m=kt_m, lam=lam, pp=PP, R=R_LL / 2, L=L_LL / 2, i_max=i_max,
             tau_max=KT_OUT * i_max, J_rotor=J_ROTOR, tau_c=TAU_COULOMB, b=B_VISC, k_g=K_GEAR,
             motor_mass=MOTOR_MASS, stock_mass=STOCK_MASS, w_max_listed=W_MAX_LISTED, v_rated_max=V_RATED_MAX)
    return {k: TILE(v) for k, v in p.items()}
