# PPO cross-check, fixed before training (22 Oct 2026)

- Library: rsl_rl 5.5.1 (PPO used by Isaac Lab), PyTorch 2.11 + CUDA 12.8, MuJoCo Warp training simulator.
- Settings (rsl_rl defaults where they exist): actor and critic 64-64 ELU, initial action std 0.6, 4096 environments,
  32 steps per environment per iteration, 5 epochs, 4 minibatches, clip 0.2, entropy 0.005, gamma 0.99, lambda 0.95,
  adaptive learning rate starting at 1e-3 with desired KL 0.01. Action = 1 Nm x tanh(policy output). Same observation,
  reward, episode, initial states, actuator models and 30 ms reference as the ES confirmation (Follow-up 7).
- Sanity runs (seed 0) chose these settings: a fixed 3e-4 learning rate never learned in 1,500 iterations; the
  adaptive default did. PPO from scratch on the compact model did not learn in 1,000 iterations.
- Reference route: 500 iterations nominal from scratch (checkpoint kept), 500 more nominal, 500 with load 0.6 to 1.5 kg
  and delay 2 to 8 ms randomisation.
- Compact route: start from the reference route's 500-iteration checkpoint (same seed), 500 iterations nominal on the
  compact model, 500 randomised. Equal 1,500-iteration budgets.
- 8 seeds (0 to 7). The final policy of every seed is deployed (no checkpoint selection), deterministic action.
- Deployment and plants as in Follow-up 7 (43 plants, 3 starts). Cases R+H3, R+E3, A2+S.
- Primary test: R+H3 minus A2+S on the 30 new plants, non-inferiority margin -5 points, two-level bootstrap
  (seeds and plants), 5000 draws, 95%. Secondary: out-of-range difference, R+H3 minus R+E3, agreement with the
  ES result, and learnability of the compact model from scratch.
