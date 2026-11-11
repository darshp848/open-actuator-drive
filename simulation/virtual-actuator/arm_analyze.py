"""Preregistered analysis for the arm-push confirmation (see arm_prereg.md).

Usage: python arm_analyze.py <T30 csv> [<T15 csv>] [out json] [--train file1.json file2.json ...]
The T15 csv contributes only R+H3 / R+E3, renamed R15+H3 / R15+E3 (its A2+S rows are checked against T30).
New plants = held_out + new. Two-level bootstrap (seeds, then plants), 5000 draws, seed 0.
"""
import json, sys
from pathlib import Path
import numpy as np
import pandas as pd

NEW, OOD = ['held_out', 'new'], ['out_of_range']


def two_level(df, a, b, sets, n=5000, seed=0):
    d = df[df.plant_set.isin(sets)]
    m = d.groupby(['case', 'seed', 'plant']).success.mean()     # mean over starts
    A, B = m[a].unstack(), m[b].unstack()                       # seeds x plants
    plants = A.columns.intersection(B.columns)
    A, B = A[plants].to_numpy(), B[plants].to_numpy()
    rng = np.random.default_rng(seed)
    sa, sb, pl = A.shape[0], B.shape[0], len(plants)
    out = np.empty(n)
    for k in range(n):
        p = rng.integers(0, pl, pl)
        out[k] = A[rng.integers(0, sa, sa)][:, p].mean() - B[rng.integers(0, sb, sb)][:, p].mean()
    return dict(a=float(A.mean()), b=float(B.mean()), diff=float(A.mean() - B.mean()),
                ci95=[float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))],
                n_seeds=[int(sa), int(sb)], n_plants=int(pl))


def load(t30, t15=None):
    df = pd.read_csv(t30)
    df['success'] = df.success.astype(float)
    if t15:
        d15 = pd.read_csv(t15)
        d15['success'] = d15.success.astype(float)
        key = ['case', 'seed', 'init', 'plant']
        x = df[df.case == 'A2+S'].merge(d15[d15.case == 'A2+S'], on=key, suffixes=('_30', '_15'))
        if x.empty:
            print('WARNING: no overlapping A2+S rows between T30 and T15 files')
        else:
            bad = int((x.success_30 != x.success_15).sum())
            nA = int((df.case == 'A2+S').sum()); nB = int((d15.case == 'A2+S').sum())
            if bad or len(x) != nA or len(x) != nB:
                print(f'WARNING: A2+S differs between files: {bad} success mismatches of {len(x)} matched rows '
                      f'(T30 has {nA}, T15 has {nB})')
            else:
                print(f'A2+S rows agree between T30 and T15 ({len(x)} rows)')
        keep = d15[d15.case.isin(['R+H3', 'R+E3'])].copy()
        keep['case'] = keep.case.map({'R+H3': 'R15+H3', 'R+E3': 'R15+E3'})
        df = pd.concat([df, keep], ignore_index=True)
    return df


def train_table(files):
    out = {}
    for f in files:
        r = json.loads(Path(f).read_text())
        fin = [x['train_success'] for x in r['final']]
        last = r['history'][-1]['mean_candidate_success'] if r.get('history') else None
        out[Path(f).name] = dict(final_train_success=fin, last_mean_candidate_success=last,
                                 gen=(r['history'][-1].get('gen') if r.get('history') else None))
    return out


def main():
    argv = sys.argv[1:]
    train = []
    if '--train' in argv:
        i = argv.index('--train'); train = argv[i + 1:]; argv = argv[:i]
    if not argv:
        sys.exit(__doc__)
    csvs = [a for a in argv if a.endswith('.csv')]
    outp = next((a for a in argv if not a.endswith('.csv')), None)
    df = load(csvs[0], csvs[1] if len(csvs) > 1 else None)
    cases = set(df.case)

    def cmp(a, b, sets):
        if a in cases and b in cases:
            return two_level(df, a, b, sets)
        return dict(skipped=f'missing case {a if a not in cases else b}')

    res = {
        'primary_RH3_minus_A2S_new': cmp('R+H3', 'A2+S', NEW),
        'RH3_minus_A2S_ood': cmp('R+H3', 'A2+S', OOD),
        'RH3_minus_RE3_new': cmp('R+H3', 'R+E3', NEW),
        'RH3_minus_RE3_ood': cmp('R+H3', 'R+E3', OOD),
    }
    if 'R15+H3' in cases:
        res.update({
            'R15H3_minus_A2S_new': cmp('R15+H3', 'A2+S', NEW),
            'R15H3_minus_A2S_ood': cmp('R15+H3', 'A2+S', OOD),
            'R15H3_minus_RH3_new': cmp('R15+H3', 'R+H3', NEW),
            'R15H3_minus_RH3_ood': cmp('R15+H3', 'R+H3', OOD),
        })
    p = res['primary_RH3_minus_A2S_new']
    res['non_inferiority_passed'] = bool(p['ci95'][0] > -0.05) if 'ci95' in p else None

    sr = df.groupby(['case', 'plant_set']).success.mean().unstack().round(4)
    res['by_case_set'] = {c: r.dropna().to_dict() for c, r in sr.iterrows()}
    res['by_case_seed_new'] = {c: {str(k): v for k, v in r.to_dict().items()} for c, r in
                               df[df.plant_set.isin(NEW)].groupby(['case', 'seed']).success.mean().unstack().round(4).iterrows()}
    res['ood_by_plant'] = {c: r.to_dict() for c, r in
                           df[df.plant_set.isin(OOD)].groupby(['case', 'plant']).success.mean().unstack().round(3).iterrows()}
    res['held_out_by_plant'] = {c: r.to_dict() for c, r in
                                df[df.plant_set == 'held_out'].groupby(['case', 'plant']).success.mean().unstack().round(3).iterrows()}
    if train:
        res['training'] = train_table(train)

    txt = json.dumps(res, indent=2)
    print(txt)
    if outp:
        Path(outp).write_text(txt)
    figure(df)


def figure(df):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    order = [c for c in ['A2+S', 'R+H3', 'R+E3', 'R15+H3', 'R15+E3'] if c in set(df.case)]
    order += sorted(set(df.case) - set(order))
    groups = [('New plants (held-out + new)', NEW), ('Out-of-range plants', OOD)]
    fig, ax = plt.subplots(figsize=(max(6, 1.6 * len(order) + 2), 4.5))
    w = 0.38
    rng = np.random.default_rng(0)
    for gi, (lab, sets) in enumerate(groups):
        m = df[df.plant_set.isin(sets)].groupby(['case', 'seed', 'plant']).success.mean()
        per_seed = m.groupby(['case', 'seed']).mean()
        for ci, c in enumerate(order):
            if c not in per_seed.index.get_level_values(0):
                continue
            v = per_seed[c].to_numpy()
            x = ci + (gi - 0.5) * w
            ax.bar(x, v.mean(), w * 0.92, color=['0.35', '0.75'][gi], edgecolor='k', linewidth=0.6,
                   label=lab if ci == 0 else None)
            ax.scatter(x + rng.uniform(-w * 0.25, w * 0.25, len(v)), v, s=14, color='k', zorder=3)
    ax.set_xticks(range(len(order))); ax.set_xticklabels(order, fontsize=11)
    ax.set_ylabel('Success rate (mean over plants and starts)', fontsize=11)
    ax.set_ylim(0, 1.05); ax.tick_params(labelsize=10)
    ax.legend(fontsize=10, loc='lower right', frameon=False)
    ax.set_title('Arm push: success by case (dots = individual seeds)', fontsize=12)
    fig.tight_layout()
    Path('figures').mkdir(exist_ok=True)
    fig.savefig('figures/arm_fu9_bars.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
