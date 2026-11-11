"""Export rig-ready actuator parameters from the per_joint study (read-only use of the repo).
Run: python3 -B export_params.py   (writes only into this folder)"""
import sys, json, math, subprocess
sys.dont_write_bytecode = True
REPO = "Motor_Control_NN_Pipeline"
sys.path.insert(0, REPO + "/robot_rotor/per_joint")
import numpy as np
import actuator as A
from pathlib import Path
RES = A.RES
motors = {m.name: m for m in A.load_motors()}
opt = json.loads((RES / "optimum.json").read_text())
alt = json.loads((RES / "alternatives.json").read_text())
REQ = A.requirements()
commit = subprocess.run(["git", "-C", REPO, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
RAD_PER_ARCMIN = math.pi / 180 / 60
CAD = json.loads((Path(REPO) / "robot_rotor/cad/per_joint/index.json").read_text())
CAD_BY = {}
CAD_ALL = {}   # an index case can list more than one entry (stale builds); pick by geometry
for _k, _v in CAD.items():
    for _c in _v["cases"]:
        CAD_BY[(_v["joint"], _v["technology"], _c)] = (_k, _v)
        CAD_ALL.setdefault((_v["joint"], _v["technology"], _c), []).append((_k, _v))


def cad_block(jgroup, fam, case, x):
    """CAD data for this pick (None if the index has no entry). jgroup is the joint whose index entry is used."""
    # coupled.* cases are already index case names (export_coupled.py); main/alternatives keep the old mapping
    key = case if case.startswith("coupled.") else ("optimum.fitted_board" if case == "main" else "alternatives." + case)
    cands = [kv for kv in CAD_ALL.get((jgroup, fam, key), [])
             if kv[1]["gear"] == x["gear"] and kv[1]["ratio"] == x["ratio"] and abs(kv[1]["stack_mm"] - x["L"]) < 1e-6
             and kv[1]["motor_design"] == x["motor"]]
    if not cands:
        return None
    k, v = cands[0]
    # same geometry; coupled re-sizing can move the gearbox/structure mass by ~1 %, so scale CAD mass to the pick
    ms = x["mass"] / v["mass_kg"]
    assert abs(ms - 1) < 0.02, (k, x["motor"], ms)
    g = v["gearbox"]
    return {"index_key": k, "index_joint": v["joint"], "mass_kg": x["mass"], "cad_mass_scale": ms, "com_m": v["com_m"],
            "inertia_about_com_kgm2": [[c * ms for c in row] for row in v["inertia_about_com_kgm2"]], "rotor_inertia_cad_kgm2": v["rotor_inertia_kgm2"],
            "rotor_inertia_model_kgm2": v["rotor_inertia_model_kgm2"],
            "reflected_inertia_output_cad_kgm2": v["reflected_inertia_output_kgm2"],
            "reflected_inertia_output_model_kgm2": v["reflected_inertia_output_model_kgm2"],
            "gearbox": {"mass_kg": g["mass_kg"], "com_m": g["com_m"], "inertia_about_com_kgm2": g["inertia_about_com_kgm2"],
                        "inertia_motor_side_kgm2": g["inertia_motor_side_kgm2"], "inertia_reflected_output_kgm2": g["inertia_reflected_output_kgm2"]},
            "note": "CAD index of the Inspiron thread; reflected = CAD rotor + gearbox input, output side; the old per_joint model values are kept in J_rotor / J_reflected"}
VMAX = 0.95 * 48 / math.sqrt(3)


def psi_d_interp(mot, J, L):
    pts = mot.d["points"]
    Js = np.array([0.0] + [p["J_rms"] for p in pts])
    ps = np.array([mot.d["cal"]["psi_pm"]] + [p["psi_d"] for p in pts])
    return float(np.interp(J, Js, ps)) * max(L - mot.dL, 0) / 8.0


def export(x, jkey, jgroup=None, fam=None, case=None, req=None):
    RQ = (req or REQ)[jkey]
    mot = motors[x["motor"]]
    L, N, Ip, ratio = x["L"], x["N_turns"], x["I_peak"], x["ratio"]
    pp = mot.p
    Tm_p = mot.T(x["J_p"], L)
    kt_m = Tm_p / Ip
    lam = N * psi_d_interp(mot, x["J_p"], L)
    R = x["Pcu_peak"] / (1.5 * Ip ** 2)
    Rc = x["Pcu_cont"] / (3 * x["I_cont_rms"] ** 2)
    Lph = N * N * mot.Lq8 * max(L - mot.dL, 0) / 8.0
    J_rot_code = mot.Jrot(L)
    J_rot_refl = x["J_reflected"] / ratio ** 2
    chk_T = Tm_p * ratio * x["eta_gear"]
    w = RQ["w_peak"]
    we = pp * ratio * w
    V = math.hypot(R * Ip + we * lam, we * Lph * Ip)
    return {
        "motor_name": x["motor"], "magnet": x["family"], "R_outer_mm": x["R"], "L_stack_mm": L,
        "OD_mm": x["OD"], "length_mm": x["length"], "mass_kg": x["mass"],
        "mass_breakdown_kg": {k: x[k] for k in ("m_active", "m_magnet", "m_gear", "m_struct", "m_board")},
        "gearbox": {
            "type": x["gear"], "ratio": ratio, "eta": x["eta_gear"], "mass_kg": x["m_gear"],
            "mass_note": "m_gear is inside mass_kg (actuator total); the rig mass change uses mass_kg, so the gearbox mass is included",
            "own_inertia": "NOT MODELLED in the per_joint code (no gearbox inertia in actuator.py); J_reflected = J_rotor * ratio^2 is rotor inertia only, so gearbox inertia is not lumped anywhere",
            "backlash_arcmin": x["backlash_arcmin"], "backlash_rad": x["backlash_arcmin"] * RAD_PER_ARCMIN,
            "stiffness_Nm_arcmin": x["stiffness_Nm_arcmin"], "k_gear_Nm_rad": x["stiffness_Nm_arcmin"] / RAD_PER_ARCMIN,
            "backdrive_Nm_output": x["backdrive_Nm"]},
        "N": ratio, "eta": x["eta_gear"], "pp": pp,
        "kt_m": kt_m, "kt_m_note": "motor torque at J_p divided by I_peak (Nm per A peak phase, motor side); includes saturation at the peak point",
        "kt_m_from_lambda": 1.5 * pp * lam,
        "lam": lam, "lam_note": "N_turns * psi_d(J_p), 8 mm FEMM table interpolated and scaled by (L-dL)/8, the code's own flux basis (analysis.py, pf definition)",
        "R": R, "R_note": "Pcu_peak / (1.5 I_peak^2), star phase; same from the continuous point: %.4f ohm" % Rc,
        "L": Lph, "L_note": "N^2 * Lq8 * (L-dL)/8, q-axis inductance per phase (actuator.py)",
        "i_max": Ip, "i_cont_rms": x["I_cont_rms"],
        "J_rotor": J_rot_code, "J_rotor_note": "direct value Motor.Jrot(L) = Jrot8*L/8; J_reflected/N^2 = %.4e (identical)" % J_rot_refl,
        "J_reflected": x["J_reflected"],
        "tau_coulomb_out": x["backdrive_Nm"], "tau_coulomb_note": "backdrive_Nm (gear start torque, output side) used as the Coulomb term; the rig adds (1-eta)|tau_m| load-dependent friction as for GIM",
        "b_visc_out": None, "b_visc_note": "ASSUMED: the code has no viscous term; the rig sets it from the GIM-class ratio b/tau_c",
        "w_motor_peak": x["w_motor_peak"], "w_out_design_peak": w,
        "T_peak_out_Nm": chk_T, "T_peak_required_Nm": RQ["T_peak"], "T_cont_required_Nm": RQ["T_cont"],
        "design_point_V_over_vmax": V / VMAX,
        "N_turns": N, "J_p_Apmm2": x["J_p"],
        "iron_loss_and_cogging": "not in the per_joint model",
        "cad": cad_block(jgroup, fam, case, x) if jgroup else None,
        "source_fields": x}


