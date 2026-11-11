# Task B (arm push) confirmation, fixed before running (23 Oct 2026)

Pilot (seeds 100-101, not reused): with exact velocities in training the reference route transferred at 7%
because deployment estimates velocities; with the deployment sensing model in training (puck position noise
1 mm, 20 ms filtered differences for all velocities) R+H3 51% vs A2+S 98% on new plants. The reference route
also trained worse on its own model (randomised training success 36-46% vs 81-86%). The confirmation keeps the
pilot settings unchanged.

- Task: arm_task.py as of this date (6 starts, 4 s episodes, success = puck within 3 cm of goal for the final
  0.5 s). Sensing model in training (`sensing=est`). ES 256 pairs on the GPU, sigma 0.15, lr 0.05.
- Seeds 0 to 7 per route, one process per stage (24,576 worlds).
- Reference route, T = 30 ms: 300 generations from scratch (nominal), 300 more (nominal), 300 randomised
  (payload 0-0.5 kg, puck 0.1-0.4 kg, puck friction 0.15-0.45, delay 2-8 ms).
- Compact route: start from the reference route's 300-generation policies, 300 generations on the compact
  model (nominal), 300 randomised. Equal 900-generation budgets.
- Secondary route, reference T = 15 ms: same three stages as the 30 ms reference route; deployed with the same
  H3 and E3 gains and T_REF_S = 0.015 on the detailed plant.
- Deployment: detailed plant, torque sensor, 25 us physics; cases R+H3, R+E3, A2+S (and R15+H3, R15+E3).
  Plants: nominal, 10 held-out, 30 new (seed 2026/2027), 12 out-of-range; 6 starts. Final policy of every seed,
  no selection, failed seeds counted.
- Primary test: R+H3 minus A2+S success on the 40 new plants (held-out + new), non-inferiority margin -5
  points, two-level bootstrap (seeds and plants), 5000 draws, 95%.
- Secondary (reported, not gated): out-of-range difference; R+H3 minus R+E3; R15+H3 minus A2+S and minus
  R+H3; training success of each route on its own randomised model; compact route trained from scratch
  (pilot only).
