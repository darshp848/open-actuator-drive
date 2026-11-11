"""PPO cross-check with an established implementation: rsl_rl (ETH Zurich / NVIDIA, the PPO used by
Isaac Lab) on the existing MuJoCo Warp training simulator (gpu_es.GpuTrainer in external-action mode).

One seed per call. Same task, observation, reward, action limit, episode, initial states and
actuator models as the evolution-strategies runs; the action is A_MAX * tanh(policy output).
Training: <nominal_iters> iterations at the nominal load and delay, then <rand_iters> with load
(0.6 to 1.5 kg) and delay (2 to 8 ms) randomisation, redrawn every episode.

Run with simulation/.venv-warp:  python rslrl_train.py <reference|compact2> <seed> <nominal_iters> <rand_iters> <tag>
"""
import json, math, sys, time
from pathlib import Path
import numpy as np
import torch
import warp as wp
from tensordict import TensorDict
from rsl_rl.env import VecEnv
from rsl_rl.runners import OnPolicyRunner
import gpu_es as G

OUT = Path(__file__).with_name('results')
N_ENVS = 4096
import os
LR = float(os.environ.get('PPO_LR', '3e-4'))
SCHEDULE = os.environ.get('PPO_SCHEDULE', 'fixed')
EVAL_EVERY = 50


class WarpPendulumEnv(VecEnv):
    def __init__(self, kind, compact, n, seed):
        self.num_envs, self.num_actions, self.max_episode_length = n, 1, G.N_BLOCKS
        self.device = 'cuda:0'; self.cfg = {}
        self.tr = G.GpuTrainer(kind, n, compact, external=True)
        self.rng = np.random.default_rng(seed)
        self.randomise = False
        self.episode_length_buf = torch.zeros(n, dtype=torch.long, device=self.device)
        self.qpos, self.qvel = wp.to_torch(self.tr.d.qpos), wp.to_torch(self.tr.d.qvel)
        self.u_prev, self.u_new = wp.to_torch(self.tr.u_prev), wp.to_torch(self.tr.u_new)
        self.tail = wp.to_torch(self.tr.tail)
        self.success_hist = []
        self._reset()

    def _reset(self):
        n = self.num_envs
        if self.randomise:
            self.tr.set_randomisation(self.rng.uniform(0.6, 1.5, n), self.rng.integers(-2, 5, n))
        else:
            self.tr.set_randomisation(np.ones(n), np.zeros(n, dtype=int))
        self.tr.reset(self.rng.choice(G.INITS, n))
        self.episode_length_buf.zero_()

    def get_observations(self):
        q, w = self.qpos[:, 0], self.qvel[:, 0]
        pol = torch.stack([torch.cos(q), torch.sin(q), w / 8.0, self.u_new / G.A_MAX], -1)
        t = (self.episode_length_buf.float() / G.N_BLOCKS)[:, None]
        return TensorDict({'policy': pol, 'time': t}, batch_size=[self.num_envs])

    def step(self, actions):
        a = G.A_MAX * torch.tanh(actions[:, 0].float())
        self.u_prev.copy_(self.u_new); self.u_new.copy_(a)
        # Warp and PyTorch use different CUDA streams: order the hand-over explicitly
        torch.cuda.synchronize()
        wp.capture_launch(self.tr.graph)
        wp.synchronize()
        q, w = self.qpos[:, 0], self.qvel[:, 0]
        e = torch.remainder(q - math.pi + math.pi, 2 * math.pi) - math.pi
        rew = -(e * e + 0.05 * w * w + 0.02 * a * a) * 0.02
        self.episode_length_buf += 1
        done = self.episode_length_buf >= G.N_BLOCKS
        extras = {'time_outs': done.clone(), 'log': {}}
        if bool(done[0]):                       # all episodes are synchronised
            succ = float((self.tail < 0.3).float().mean())
            self.success_hist.append(succ)
            extras['log']['/success'] = succ
            self._reset()
        return self.get_observations(), rew, done.float(), extras


def train_cfg(seed):
    return {
        'seed': seed, 'num_steps_per_env': 32, 'save_interval': 100000, 'logger': 'tensorboard',
        'obs_groups': {'actor': ['policy'], 'critic': ['policy', 'time']},
        'actor': {'class_name': 'rsl_rl.models.MLPModel', 'hidden_dims': [64, 64], 'activation': 'elu',
                  'distribution_cfg': {'class_name': 'rsl_rl.modules.distribution.GaussianDistribution', 'init_std': 0.6}},
        'critic': {'class_name': 'rsl_rl.models.MLPModel', 'hidden_dims': [64, 64], 'activation': 'elu'},
        'algorithm': {'class_name': 'rsl_rl.algorithms.PPO', 'num_learning_epochs': 5, 'num_mini_batches': 4,
                      'clip_param': 0.2, 'gamma': 0.99, 'lam': 0.95, 'value_loss_coef': 1.0, 'entropy_coef': 0.005,
                      'learning_rate': LR, 'max_grad_norm': 1.0, 'schedule': SCHEDULE, 'desired_kl': 0.01},
    }


def export_actor(runner):
    """Linear layers of the deterministic actor, for the CPU deployment code (ELU hidden activations)."""
    pol = runner.get_inference_policy(device='cpu')
    mods = [m for m in pol.modules() if isinstance(m, torch.nn.Linear)]
    return dict(activation='elu', W=[m.weight.detach().numpy().T.tolist() for m in mods],
                b=[m.bias.detach().numpy().tolist() for m in mods])


def main():
    kind, seed, n_nom, n_rand, tag = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), sys.argv[5]
    init_ckpt = sys.argv[6] if len(sys.argv) > 6 else None      # warm start: a checkpoint saved by an earlier run
    torch.manual_seed(seed)
    cfg = json.loads((OUT / 'exp2_config.json').read_text())['cfg']
    env = WarpPendulumEnv(kind, cfg['compact2']['torque'] if kind == 'compact2' else None, N_ENVS, seed)
    log_dir = OUT / 'rslrl_logs' / f'{tag}_{kind}_s{seed}'
    runner = OnPolicyRunner(env, train_cfg(seed), log_dir=str(log_dir), device='cuda:0')
    if init_ckpt:
        runner.load(str(OUT / init_ckpt))
    def det_eval(randomise):
        """Deterministic policy, one synchronised episode on the training model (nominal or randomised)."""
        keep = env.randomise
        env.randomise = randomise; env._reset()
        pol = runner.get_inference_policy(device='cuda:0')
        with torch.inference_mode():
            obs = env.get_observations()
            for _ in range(G.N_BLOCKS):
                obs, _, _, _ = env.step(pol(obs))
        env.randomise = keep; env._reset()
        return env.success_hist.pop()

    curve = []; t0 = time.perf_counter(); done = 0
    for phase, iters in (('nominal', n_nom), ('random', n_rand)):
        env.randomise = phase == 'random'; env._reset()
        left = iters
        while left > 0:
            k = min(EVAL_EVERY, left)
            runner.learn(k); left -= k; done += k
            curve.append(dict(iter=done, phase=phase, det_success_nominal=det_eval(False),
                              det_success_random=det_eval(True), wall_s=time.perf_counter() - t0))
            print(tag, kind, seed, curve[-1], flush=True)
    res = dict(kind=kind, seed=seed, n_envs=N_ENVS, nominal_iters=n_nom, rand_iters=n_rand, lr=LR, schedule=SCHEDULE,
               transitions=(n_nom + n_rand) * 32 * N_ENVS, wall_s=time.perf_counter() - t0, curve=curve,
               final_det_success_nominal=curve[-1]['det_success_nominal'],
               final_det_success_random=curve[-1]['det_success_random'], actor=export_actor(runner))
    runner.save(str(OUT / f'rslrl_{tag}_{kind}_s{seed}.pt'))
    (OUT / f'rslrl_{tag}_{kind}_s{seed}.json').write_text(json.dumps(res))
    print(json.dumps({k: v for k, v in res.items() if k != 'actor'}), flush=True)


if __name__ == '__main__':
    main()
