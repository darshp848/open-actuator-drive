# Virtual actuator simulation

Implements the 21 October 2026 plan "Research question and simulation prototype". Synthetic motor, driver, gearbox and pendulum; no hardware interface.

## Environment

Python 3.11, MuJoCo 3.3.7, NumPy 2.4.6, SciPy 1.17.1, Matplotlib 3.11.0, pandas. Run every script from this folder.

## Run order

1. `exp0_checks.py`: analytic checks on the virtual joint.
2. `exp1_numerics.py`: closed-loop accuracy versus physics step (chooses 25 us).
3. `exp2_accuracy.py`: friction map, equal gain grids, compact fits, held-out evaluation (about 15 min on 18 processes).
4. `exp2c_equal_speed.py`: equal-speed plain PI control (case E).
5. `exp2b_refine.py`: recheck of final cases at 12.5 us.
6. `exp3_cost.py`: simulation cost and admissible step. Run with nothing else busy.
7. `analyze.py`: tables, paired statistics and figures.
8. Follow-up 1: `exp5_diagnose.py`, `exp5b_damping.py`, `exp5c_damping_wide.py`, `exp5d_envelope.py` (diagnosis, active damping, wider grid, envelope).
9. Follow-up 3: `rl_pilot.py` (CPU, main venv) and `gpu_es.py` (GPU, `.venv-warp`); `cont.py` holds the second-order discretisation.
10. Follow-up 4: `gpu_es.py <kind> <pairs> <gens> [warm-start json]` then `rl_deploy.py <reference json> <compact json> <tag>`.
11. Follow-ups 5 to 7: randomised and reference-speed runs via `gpu_es.py` (args: kind pairs gens [init|-] [rand|nominal] [seeds], env `T_REF_S`), `rl_deploy.py ... confirm`; `confirm_prereg.md`; `batched_plant.py` and `bench_mjbatch.py` (run in `.venv-mjbatch`).

## Files

- `actuator.py`: plant, driver, sensors, estimator and local controllers.
- `models.py`: reference, compact (first and second order) predictions and training simulators.
- `results/`: every output, including `exp2_rows.csv` (one row per scored case) and `summary.json`.
- `figures/`: report figures.
- `reports/`: report and follow-up PDFs; `reports/html/` holds the HTML sources (`report_body.html` etc.).
