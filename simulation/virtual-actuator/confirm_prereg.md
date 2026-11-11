# Confirmation run, fixed before running (22 Oct 2026)

- Reference T = 30 ms; 8 seeds (0 to 7) per route; ES 512 candidates per generation on the GPU.
- Reference route: 300 generations from scratch (nominal), 300 more (nominal), 300 with load 0.6 to 1.5 kg and delay 2 to 8 ms randomisation.
- Compact route: start from the reference route's 300-generation scratch policies, 300 generations on the compact model (nominal), 300 with the same randomisation. Equal total budget: 900 generations each.
- Deployment: detailed plant, torque sensor, cases R+H3, R+E3, A2+S. Plants: nominal, 10 held-out, 20 fresh (seed 2026), 12 out-of-range (6 from Follow-up 5 plus 0.3 kg, 1.7 kg, 2.0 kg, 12 ms delay, 0.4 kg with 10 ms delay, 1.8 kg with soft gearbox). 3 initial states.
- Primary test: R+H3 minus A2+S success on the 30 new plants, non-inferiority margin −5 points. Interval: bootstrap resampling seeds and plants together (two-level), 5000 draws, 95%.
- Secondary (reported, not gated): the same difference on the 12 out-of-range plants; R+H3 minus R+E3.
