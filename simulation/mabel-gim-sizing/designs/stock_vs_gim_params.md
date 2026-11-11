# Stock reference: per_joint code vs rig gim_params.py

The per_joint code has no electrical parameters for the stock actuators (no turns, R, L, lambda): only published ratio, torque, mass, size and a modelled reflected inertia. So only these fields can be compared. The rig values (R, L, kt, lambda, pp) come from the SteadyWin listings in gim_params.py and are not cross-checked by the per_joint code.

| field | actuator | per_joint code | rig gim_params | difference |
|---|---|---|---|---|
| ratio | SteadyWin GIM8115-36 (Y, 48 V, GDS810) (J1/J2) | 36 | 36 | +0 |
| peak/listed torque Nm | SteadyWin GIM8115-36 (Y, 48 V, GDS810) | 228 | 228.0 | +0.00 |
| bare mass kg | SteadyWin GIM8115-36 (Y, 48 V, GDS810) | 0.92 | 0.92 | +0.000 |
| mass with driver kg | SteadyWin GIM8115-36 (Y, 48 V, GDS810) | 0.96 | not in gim_params (rig2 uses this) | |
| reflected rotor inertia kg m2 | SteadyWin GIM8115-36 (Y, 48 V, GDS810) | 0.318 (modelled, model) | 0.156 (ASSUMED J_rotor x N^2) | 2.04x |
| rated (continuous) torque Nm | SteadyWin GIM8115-36 (Y, 48 V, GDS810) | 54 published; model 52.1 | not in rig | |
| ratio | SteadyWin GIM8108-36 (J3/J4) | 36 | 36 | +0 |
| peak/listed torque Nm | SteadyWin GIM8108-36 | 109.5 | 109.5 | +0.00 |
| bare mass kg | SteadyWin GIM8108-36 | 0.72 | 0.72 | +0.000 |
| mass with driver kg | SteadyWin GIM8108-36 | 0.76 | not in gim_params (rig2 uses this) | |
| reflected rotor inertia kg m2 | SteadyWin GIM8108-36 | 0.170 (modelled, model) | 0.130 (ASSUMED J_rotor x N^2) | 1.31x |
| rated (continuous) torque Nm | SteadyWin GIM8108-36 | 32.76 published; model 32.8 | not in rig | |
| ratio | SteadyWin GIM4310-40 (J5-J7) | 40 | 40 | +0 |
| peak/listed torque Nm | SteadyWin GIM4310-40 | 30.59 | 30.59 | +0.00 |
| bare mass kg | SteadyWin GIM4310-40 | 0.3367 | 0.337 | -0.000 |
| mass with driver kg | SteadyWin GIM4310-40 | 0.3645 | not in gim_params (rig2 uses this) | |
| reflected rotor inertia kg m2 | SteadyWin GIM4310-40 | 0.030 (modelled, model) | 0.032 (ASSUMED J_rotor x N^2) | 0.94x |
| rated (continuous) torque Nm | SteadyWin GIM4310-40 | 8.5 published; model 7.3 | not in rig | |
| torso actuator | DM-J10422P | 22:1, 2.7 kg, J_refl 0.2952540507418959 (bench) | rig torso is a GIM8115-36 (36:1, 0.156 kg m2) | torso stock differs by design choice |
| stock Damiao masses in MJCF | arms | DM-J8009P 0.896, DM-J4340 0.362, DM-J4310 0.30 kg | STOCK_MASS 0.80, 0.36, 0.30 (ASSUMED) | J1/J2 0.096 kg; rig2 uses the per_joint (published) values |
