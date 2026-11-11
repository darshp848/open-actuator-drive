"""Tables, paired comparisons and figures for the report."""
import json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from actuator import NOMINAL, HELD_OUT, Gains, FrictionMap, run, command_signal
from models import reference_prediction, Compact2

OUT = Path(__file__).with_name('results')
FIG = Path(__file__).with_name('figures'); FIG.mkdir(exist_ok=True)
# reference categorical palette (light): slot order fixed
BLUE, ORANGE, AQUA, YELLOW, MAGENTA, GREEN = '#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300'
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
VIOLET = '#6250d6'
CASE_COLOR = {'A': AQUA, 'A2': BLUE, 'B': YELLOW, 'C': MAGENTA, 'D': ORANGE, 'D0': GREEN, 'E': VIOLET}
CASE_LABEL = {'A': 'A  compact, 1st order', 'A2': 'A2 compact, 2nd order', 'B': 'B  detailed nominal model',
              'C': 'C  reference, conventional loop', 'D': 'D  reference, shaping loop', 'D0': 'D0 shaping, no friction FF',
              'E': 'E  reference, plain PI (equal speed)'}
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'xtick.color': INK2,
                     'ytick.color': INK2, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.facecolor': SURF, 'axes.facecolor': SURF, 'grid.color': GRID})


def cluster_bootstrap(diff_by_plant, n=5000, seed=0):
    """Mean paired difference with plants resampled as clusters."""
    rng = np.random.default_rng(seed)
    groups = list(diff_by_plant.values())
    stats = []
    for _ in range(n):
        pick = rng.integers(0, len(groups), len(groups))
        stats.append(np.mean(np.concatenate([groups[i] for i in pick])))
    return float(np.mean(np.concatenate(groups))), float(np.percentile(stats, 2.5)), float(np.percentile(stats, 97.5))


def main():
    rows = pd.read_csv(OUT / 'exp2_rows.csv')
    rows = pd.concat([rows, pd.read_csv(OUT / 'exp2c_rows.csv')], ignore_index=True)
    cfg = json.loads((OUT / 'exp2_config.json').read_text())
    held = rows[rows.plant != 'nominal']
    cases = ['A', 'A2', 'B', 'C', 'E', 'D0', 'D']
    summary = {}
    for sens in ['torque', 'estimator']:
        x = held[held.sensing == sens]
        tab = x.groupby('case').agg(pass_rate=('passed', 'mean'), rms_median=('rms', 'median'),
                                    rms_p90=('rms', lambda s: s.quantile(.9)), peak_median=('peak', 'median'),
                                    angle_median=('angle_rmse', 'median'), n=('passed', 'size')).loc[cases]
        per_plant = x.pivot_table(index='plant', columns='case', values='passed', aggfunc='mean')[cases]
        per_plant_rms = x.pivot_table(index='plant', columns='case', values='rms', aggfunc='median')[cases]
        per_amp = x.pivot_table(index='amp', columns='case', values='passed', aggfunc='mean')[cases]
        per_sig = x.pivot_table(index='signal', columns='case', values='passed', aggfunc='mean')[cases]
        paired = {}
        for base in ['A', 'A2', 'B', 'C', 'E']:
            p = x[x.case == 'D'].merge(x[x.case == base], on=['plant', 'signal', 'amp'], suffixes=('_d', '_b'))
            diff = {pl: (g.rms_b - g.rms_d).to_numpy() for pl, g in p.groupby('plant')}
            mean, lo, hi = cluster_bootstrap(diff)
            paired[f'{base}_minus_D_rms'] = dict(mean=mean, ci95=[lo, hi], d_better_frac=float(np.mean(p.rms_d < p.rms_b)),
                                                 d_pass_only=int(np.sum(p.passed_d & ~p.passed_b)),
                                                 base_pass_only=int(np.sum(p.passed_b & ~p.passed_d)))
        summary[sens] = dict(table=json.loads(tab.to_json(orient='index')),
                             per_plant_pass=json.loads(per_plant.to_json(orient='index')),
                             per_plant_rms=json.loads(per_plant_rms.to_json(orient='index')),
                             per_amp_pass=json.loads(per_amp.to_json(orient='index')),
                             per_signal_pass=json.loads(per_sig.to_json(orient='index')),
                             paired=paired,
                             nominal_plant_pass=rows[(rows.plant == 'nominal') & (rows.sensing == sens)]
                             .groupby('case').passed.mean().to_dict())
        print(sens); print(tab.round(3)); print(per_amp.round(2)); print(per_sig.round(2)); print(json.dumps(paired, indent=1))
    (OUT / 'summary.json').write_text(json.dumps(summary, indent=2))

    # Figure 1: pass rate by case and sensing
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 3.2), sharey=True)
    for ax, sens in zip(axes, ['torque', 'estimator']):
        t = summary[sens]['table']
        vals = [t[c]['pass_rate'] * 100 for c in cases]
        bars = ax.bar(range(len(cases)), vals, color=[CASE_COLOR[c] for c in cases], width=0.7,
                      edgecolor=SURF, linewidth=2)
        for i, v in enumerate(vals):
            ax.text(i, v + 1.5, f'{v:.0f}%', ha='center', color=INK, fontsize=8)
        ax.set_xticks(range(len(cases))); ax.set_xticklabels(cases)
        ax.set_title('Output torque sensor' if sens == 'torque' else 'Current + encoders (no torque sensor)',
                     color=INK, fontsize=9, loc='left')
        ax.yaxis.grid(True); ax.set_axisbelow(True); ax.set_ylim(0, 105)
    axes[0].set_ylabel('Held-out cases passing\nRMS ≤ 0.1 Nm and peak ≤ 0.3 Nm (%)')
    fig.legend([plt.Rectangle((0, 0), 1, 1, color=CASE_COLOR[c]) for c in cases], [CASE_LABEL[c] for c in cases],
               loc='lower center', ncol=3, frameon=False, fontsize=8, bbox_to_anchor=(0.5, -0.16))
    fig.tight_layout(); fig.savefig(FIG / 'fig1_pass_rate.png', dpi=180, bbox_inches='tight'); plt.close(fig)

    # Figure 2: median RMS per held-out plant, A2 vs D vs B (torque sensing)
    t = pd.DataFrame(summary['torque']['per_plant_rms']).T
    plants = t.sort_values('D').index.tolist()
    fig, ax = plt.subplots(figsize=(8, 3.4))
    w = 0.26
    for k, c in enumerate(['A2', 'B', 'D']):
        ax.bar(np.arange(len(plants)) + (k - 1) * w, t.loc[plants, c], w, color=CASE_COLOR[c], label=CASE_LABEL[c],
               edgecolor=SURF, linewidth=1)
    ax.axhline(0.1, color=INK2, lw=1, ls='--'); ax.text(len(plants) - 0.5, 0.103, 'RMS limit 0.1 Nm', ha='right',
                                                          color=INK2, fontsize=8)
    ax.set_xticks(range(len(plants))); ax.set_xticklabels(plants, rotation=30, ha='right')
    ax.set_ylabel('Median RMS torque error (Nm)'); ax.yaxis.grid(True); ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=8, loc='upper left')
    ax.set_title('Held-out plants, output torque sensing', color=INK, fontsize=9, loc='left')
    fig.tight_layout(); fig.savefig(FIG / 'fig2_rms_by_plant.png', dpi=180); plt.close(fig)

    # Figure 3: example traces on two plants
    g = cfg['cfg']['gains']['torque']; fm = FrictionMap(**cfg['cfg']['fmap'])
    cp2 = Compact2(**cfg['cfg']['compact2']['torque'])
    fig, axes = plt.subplots(2, 1, figsize=(8, 4.6), sharex=True)
    for ax, plant in zip(axes, [p for p in HELD_OUT if p.name in ('soft_gearbox', 'backlash')]):
        c = command_signal('steps', 1.0)
        rS = run(plant, 'S', Gains(**g['S']), fm, 'torque', c, h=25e-6, seed=0)
        rH = run(plant, 'H', Gains(**g['H']), fm, 'torque', c, h=25e-6, seed=0)
        ref = reference_prediction(c, rS['n_phys'], 25e-6)
        p2 = cp2.predict(c, rS['w_fine'], rS['n_phys'], 25e-6)
        tt = (np.arange(rS['shaft'].size) + 1) * 25e-6
        sl = slice(0, int(1.6 / 25e-6))
        ax.plot(tt[sl], rS['shaft'].ravel()[sl], color=BLUE, lw=1.2, alpha=0.55, label='shaft torque, conventional loop')
        ax.plot(tt[sl], p2.ravel()[sl], color=BLUE, lw=1.6, ls='--', label='A2 compact-model prediction')
        ax.plot(tt[sl], rH['shaft'].ravel()[sl], color=ORANGE, lw=1.2, alpha=0.7, label='shaft torque, shaping loop')
        ax.plot(tt[sl], ref.ravel()[sl], color=INK, lw=1.4, ls=':', label='prescribed reference R')
        ax.set_ylabel('Torque (Nm)'); ax.yaxis.grid(True); ax.set_axisbelow(True)
        ax.set_title(f'{plant.name}, 1 Nm steps', color=INK, fontsize=9, loc='left')
    axes[1].set_xlabel('Time (s)')
    axes[0].legend(frameon=False, fontsize=7.5, ncol=4, loc='lower center', bbox_to_anchor=(0.5, 1.12))
    fig.tight_layout(); fig.savefig(FIG / 'fig3_example_traces.png', dpi=180); plt.close(fig)

    # Figure 4: numerical convergence (exp1)
    e1 = json.loads((OUT / 'exp1_numerics.json').read_text())['summary']
    hs = [int(k) for k in e1 if int(k) != 12]
    fig, ax = plt.subplots(figsize=(5.5, 3))
    ax.loglog(hs, [e1[str(h)]['worst_d_rms'] for h in hs], 'o-', color=BLUE, lw=2, ms=6, label='worst change in RMS error')
    ax.loglog(hs, [e1[str(h)]['worst_d_peak'] for h in hs], 's-', color=ORANGE, lw=2, ms=6, label='worst change in peak error')
    ax.axhline(0.01, color=BLUE, lw=1, ls='--'); ax.axhline(0.03, color=ORANGE, lw=1, ls='--')
    ax.set_xlabel('Physics step (µs)'); ax.set_ylabel('Difference from 12.5 µs run (Nm)')
    ax.legend(frameon=False, fontsize=8); ax.grid(True, which='major'); ax.set_axisbelow(True)
    fig.tight_layout(); fig.savefig(FIG / 'fig4_numerics.png', dpi=180); plt.close(fig)

    # Figure 5: cost
    e3 = json.loads((OUT / 'exp3_cost.json').read_text())
    fig, ax = plt.subplots(figsize=(5.5, 3))
    kinds = ['bare', 'reference', 'compact', 'detailed']
    for k, (step, col) in enumerate([('1000us', BLUE), ('25us', ORANGE)]):
        v = [e3['same_step'][step][kd]['wall_per_sim_s'] for kd in kinds]
        ax.bar(np.arange(4) + (k - 0.5) * 0.38, v, 0.38, color=col, edgecolor=SURF, linewidth=1,
               label=f'physics step {step.replace("us", " µs")}')
    ax.set_yscale('log'); ax.set_xticks(range(4)); ax.set_xticklabels(kinds)
    ax.set_ylabel('CPU wall time per simulated second (s)'); ax.legend(frameon=False, fontsize=8)
    ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout(); fig.savefig(FIG / 'fig5_cost.png', dpi=180); plt.close(fig)


if __name__ == '__main__':
    main()
