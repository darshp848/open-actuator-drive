"""Size the X1 brake resistor board against assumed regen profiles.

    python brake_sizing.py            # prints the tables used in README.md

The chopper on the support board switches 2.5 ohm (4 x 10 ohm) across BUS+
between 57.1 V (on) and 55.0 V (off). While it conducts the bank takes about
1.3 kW, but only in short bursts: averaged over time the bank dissipates the
regenerated power, nothing more. So the bank is sized by three things:

  1. each chopper burst against the resistor's single-pulse energy curve,
  2. regen events (a few hundred W for under a second) against the element's
     transient thermal impedance, starting from a hot heatsink,
  3. the average regen power against the heatsinks, with the 85 C thermostat.

Every motor and duty number below is an assumption; the sources are noted in
place. Nothing here is a measured regen profile.
"""
import math

# ---------------------------------------------------------------- chopper
R_BANK = 2.5              # ohm, 4 x LTO100 10 ohm in parallel
N_RES = 4
V_ON, V_OFF = 57.1, 55.0  # support-board primary channel (bus_protection_d05.py)
V_BACKUP = 61.13          # bus-powered backup channel threshold
C_BUS = 1.78e-3           # F: 1.248 mF minimum bank + 7 x 76 uF on the boards

# ---------------------------------------------------------------- LTO100
# Vishay LTO 100, doc 50051 rev 29-Apr-2026.
TJ_MAX = 175.0
RTH_JC = 1.5              # C/W
RTH_CS = 1.0              # C/W, silicone grease, Vishay's own example value
C_CASE = 1.5              # J/K, assumed: 3.5 g part, mostly alumina and mould
# Energy curve (p. 4), single pulse on a cold part, read off the log plot.
PULSE_E = [(1e-4, 0.9), (1e-3, 2.8), (1e-2, 9.0), (1e-1, 30.0), (0.2, 50.0)]
# Foster fit to Zth(t) = 150 K / (E(t)/t), conservative at 0.2 s
# (0.73 vs 0.60 C/W from the curve) and going to RTH_JC at steady state.
FOSTER = [(0.15, 0.003), (0.45, 0.08), (0.90, 1.0)]

# ---------------------------------------------------------------- heatsink
# Ohmite RA-T2X-64E, one per resistor. Catalog 3.1 C/W natural convection.
# Mounting-surface rise from the R-series H = 2.5 in curve, read off the plot:
# about 60 K at 20 W; the power law follows the curve's shape within ~10 %.
C_SINK = 63 * 0.90        # J/K: 63 g of 6063 aluminium
NEIGHBOUR_DERATE = 1.15   # assumed: four sinks in a 2 x 2 block, 9-12 mm apart


def sink_rise(p):
    return NEIGHBOUR_DERATE * 60.0 * (max(p, 0.0) / 20.0) ** 0.83


# ---------------------------------------------------------------- thermostat
# Sensata Airpax 67L085P on HS1: opens at 85 +/- 5 C, resets >= 55 C.
T_THERMO_MIN = 80.0
T_AMB = 40.0              # design ambient inside the robot


def pulse_energy_limit(t):
    """Interpolate the cold single-pulse energy curve (log-log)."""
    pts = PULSE_E
    if t <= pts[0][0]:
        return pts[0][1] * (t / pts[0][0]) ** 0.5
    for (t0, e0), (t1, e1) in zip(pts, pts[1:]):
        if t <= t1:
            k = math.log(e1 / e0) / math.log(t1 / t0)
            return e0 * (t / t0) ** k
    return None  # beyond the curve: use the thermal model


def chopper_burst():
    """One chopper burst with no regen current: the bus falls V_ON -> V_OFF."""
    p_on = V_ON ** 2 / R_BANK
    e_bank = 0.5 * C_BUS * (V_ON ** 2 - V_OFF ** 2)
    t = e_bank / ((V_ON + V_OFF) ** 2 / 4 / R_BANK)
    return p_on, t, e_bank / N_RES


def steady(p_total, t_amb=T_AMB):
    p = p_total / N_RES
    ts = t_amb + sink_rise(p)
    tc = ts + p * RTH_CS
    return ts, tc, tc + p * RTH_JC


def max_continuous(t_amb=T_AMB):
    lo, hi = 0.0, 400.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if steady(mid, t_amb)[0] <= T_THERMO_MIN:
            lo = mid
        else:
            hi = mid
    return lo


def simulate(profile, t_end, p_background, dt=1e-3):
    """Per-resistor transient. profile(t) -> total bank power (W).
    Starts at the steady state of p_background. Returns peak Tj, peak Ts."""
    p0 = p_background / N_RES
    ts, tc, _ = steady(p_background)
    foster = [p0 * r for r, _ in FOSTER]
    tj_max, ts_max = 0.0, ts
    t = 0.0
    while t < t_end:
        p = profile(t) / N_RES
        q_cs = (tc - ts) / RTH_CS
        tc += (p - q_cs) * dt / C_CASE
        q_sa = sink_rise_inverse(ts - T_AMB)
        ts += (q_cs - q_sa) * dt / C_SINK
        foster = [f + (p * r - f) * dt / tau for f, (r, tau) in zip(foster, FOSTER)]
        tj = tc + sum(foster)
        tj_max = max(tj_max, tj)
        ts_max = max(ts_max, ts)
        t += dt
    return tj_max, ts_max


def sink_rise_inverse(dT):
    if dT <= 0:
        return 0.0
    return 20.0 * (dT / (NEIGHBOUR_DERATE * 60.0)) ** (1 / 0.83)


# ---------------------------------------------------------------- profiles
# Motor data: system_model/openarm_params.m (GIM) and
# operating-envelopes-20261023.md (WGG8108-8 catalog, 24 V test condition).
GIM8115 = dict(kt_out=11.05, rpm_per_v=0.75, r_ph=0.49, ratio=36)
WGG8108 = dict(kt_out=10.2 / 17.1, r_ph=0.03, ratio=8,
               rpm_cat=390, j_rotor=1.5e-4)   # rotor inertia ASSUMED (unpublished)


def lowering_power(m=6.0, arm=0.6, extra_nm=10.0, w=3.0, eta=0.9, board_w=3.0):
    """Electrical power one GIM8115 joint returns while lowering at speed w."""
    tau = m * 9.81 * arm + extra_nm
    i = tau / GIM8115["kt_out"]
    return eta * tau * w - 1.5 * i ** 2 * GIM8115["r_ph"] - board_w


def wgg_rotor_ke(rpm_out):
    w_m = rpm_out * 2 * math.pi / 60 * WGG8108["ratio"]
    return 0.5 * WGG8108["j_rotor"] * w_m ** 2


def pulses(p, on, period):
    return lambda t: p if (t % period) < on else 0.0


def cases():
    p_low = lowering_power()
    # Average over a 90 degree lowering: gravity torque averages 2/pi of peak.
    p_low_avg = lowering_power(m=6.0 * 2 / math.pi, extra_nm=10.0 * 2 / math.pi)
    rotor_390 = wgg_rotor_ke(WGG8108["rpm_cat"])
    rotor_48v = wgg_rotor_ke(819)   # naive Kv x 50.4 V, not a qualified speed
    return [
        # name, description, average W, transient (profile, seconds) or None
        ("A1", "P96 arm lift6 cycle (system model): 28 J every 3.5 s",
         28 / 3.5, None),
        ("A2", f"P96 arm fast pick-and-place: J1+J2 lower 6 kg at 3 rad/s, "
               f"2 x {p_low_avg:.0f} W for 0.52 s every 2 s",
         2 * p_low_avg * 0.52 / 2.0, None),
        ("A3", "P96 gates-off fall, J1+J2+J4 (brake-resistor-check-20261025): "
               "140 J in 0.65 s, from A2 hot",
         None, (lambda t: 215.0 if t < 0.65 else 0.0, 3.0, "A2")),
        ("A4", "P96 arm e-stop from the J1 speed limit: arm + payload KE about "
               "30 J returned in 0.1 s, from A2 hot",
         None, (lambda t: 300.0 if t < 0.1 else 0.0, 2.0, "A2")),
        ("W1", "3 x WGG8108 gait-like braking: each 160 W for 0.1 s per 1 s "
               "step, two in phase",
         2 * 160 * 0.1 / 1.0, None),
        ("W2", f"3 x WGG8108 stop from 819 rpm: rotor KE {3 * rotor_48v:.0f} J "
               f"+ 50 J load (requirements regen_event_j), at the 1.3 kW "
               "chopper limit, from W1 hot",
         None, (lambda t, e=3 * rotor_48v + 50: (V_ON ** 2 / R_BANK)
                if t < e / (V_ON ** 2 / R_BANK) else 0.0, 2.0, "W1")),
        ("W3", "1 x WGG8108 continuous braking at nominal 10.2 Nm and 390 rpm "
               "(dyno, winch): bounding, NOT supported",
         0.85 * 10.2 * 390 * 2 * math.pi / 60 - 1.5 * 17.1 ** 2 * 0.03, None),
        ("F1", "Support-board forced discharge on RUN open: 3 mF from 54 V",
         None, (lambda t: 54 ** 2 / R_BANK * math.exp(-2 * t / (R_BANK * 3e-3)),
                0.1, None)),
    ], rotor_390, rotor_48v


def main():
    p_on, t_burst, e_burst = chopper_burst()
    lim = pulse_energy_limit(t_burst)
    print(f"Chopper on: {p_on:.0f} W bank, {p_on / N_RES:.0f} W per resistor, "
          f"{V_ON / R_BANK:.1f} A")
    print(f"One burst with no regen: {t_burst * 1e3:.2f} ms, {e_burst * 1e3:.0f} mJ "
          f"per resistor; cold limit {lim:.2f} J ({e_burst / lim * 100:.0f} %)")
    print(f"Backup channel: {V_BACKUP ** 2 / R_BANK:.0f} W bank")
    print()
    pmax = max_continuous()
    pmax25 = max_continuous(25.0)
    ts, tc, tj = steady(pmax)
    print(f"Continuous rating (heatsink <= {T_THERMO_MIN:.0f} C, thermostat lowest "
          f"opening): {pmax:.0f} W at {T_AMB:.0f} C ambient, {pmax25:.0f} W at 25 C")
    print(f"  at {pmax:.0f} W: sink {ts:.0f} C, case {tc:.0f} C, element {tj:.0f} C; "
          f"LTO100 allows {100 * (TJ_MAX - tc) / 150:.0f} W at that case temperature")
    print()
    table, rotor_390, rotor_48v = cases()
    print(f"WGG rotor KE (J_rotor {WGG8108['j_rotor']:.1e} kg m2 assumed): "
          f"{rotor_390:.1f} J at 390 rpm, {rotor_48v:.1f} J at 819 rpm")
    print()
    print("| Case | Profile | Average W | Peak element C | Peak sink C | Verdict |")
    print("|---|---|---|---|---|---|")
    avg = {}
    for name, desc, p_avg, tr in table:
        if tr is None:
            ts, tc, tj = steady(p_avg)
            avg[name] = p_avg
            ok = ts <= T_THERMO_MIN and tj < TJ_MAX
            verdict = "fits" if ok else "exceeds rating"
            print(f"| {name} | {desc} | {p_avg:.0f} | {tj:.0f} | {ts:.0f} | {verdict} |")
        else:
            prof, t_end, base = tr
            p_bg = avg.get(base, 0.0)
            tj, ts = simulate(prof, t_end, p_bg)
            e = sum(prof(i * 1e-3) * 1e-3 for i in range(int(t_end * 1e3)))
            verdict = "fits" if tj < TJ_MAX - 10 else "too hot"
            print(f"| {name} | {desc} | event {e:.0f} J | {tj:.0f} | {ts:.0f} | {verdict} |")
    # Event capability: the highest constant bank power that an event of a given
    # energy may arrive at, starting from a sink already at the continuous rating.
    print()
    print(f"Event limit from a bank already at the {pmax:.0f} W continuous rating "
          f"(element <= {TJ_MAX - 10:.0f} C):")
    print("| Event energy | Max bank power | Shortest duration |")
    print("|---|---|---|")
    for e in (10, 25, 50, 100, 150, 300):
        p = event_power_limit(e, pmax)
        print(f"| {e} J | {p:.0f} W | {e / p:.2f} s |")


def event_power_limit(e, p_background):
    lo, hi = 1.0, V_ON ** 2 / R_BANK
    for _ in range(40):
        mid = (lo + hi) / 2
        tj, _ = simulate(lambda t, m=mid: m if t < e / m else 0.0,
                         e / mid + 0.5, p_background)
        lo, hi = (mid, hi) if tj < TJ_MAX - 10 else (lo, mid)
    return lo

if __name__ == "__main__":
    main()
