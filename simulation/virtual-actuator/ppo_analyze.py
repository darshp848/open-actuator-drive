"""Preregistered comparison for a deployment CSV (rl_deploy.py / ppo_deploy.py output).

New plants = held-out + fresh (30). Two-level bootstrap: resample seeds, then plants, 5000 draws.
Usage: python ppo_analyze.py <csv> [out json]
"""
import json, sys
from pathlib import Path
import numpy as np
import pandas as pd


def two_level(df, a, b, sets, n=5000, seed=0):
    d = df[df.plant_set.isin(sets)]
    # per (case, seed, plant) success rate over the 3 starts
    m = d.groupby(['case', 'seed', 'plant']).success.mean()
    A, B = m[a].unstack(), m[b].unstack()          # seeds x plants
    plants = A.columns.intersection(B.columns)
    A, B = A[plants].to_numpy(), B[plants].to_numpy()
    rng = np.random.default_rng(seed)
    sa, sb, pl = A.shape[0], B.shape[0], len(plants)
    out = np.empty(n)
    for k in range(n):
        p = rng.integers(0, pl, pl)
        out[k] = A[rng.integers(0, sa, sa)][:, p].mean() - B[rng.integers(0, sb, sb)][:, p].mean()
    return dict(a=float(A.mean()), b=float(B.mean()), diff=float(A.mean() - B.mean()),
                ci95=[float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))])


if __name__ == '__main__':
    df = pd.read_csv(sys.argv[1])
    new, ood = ['held_out', 'fresh'], ['out_of_range']
    res = {
        'primary_RH3_minus_A2S_new': two_level(df, 'R+H3', 'A2+S', new),
        'RH3_minus_A2S_ood': two_level(df, 'R+H3', 'A2+S', ood),
        'RH3_minus_RE3_new': two_level(df, 'R+H3', 'R+E3', new),
        'RH3_minus_RE3_ood': two_level(df, 'R+H3', 'R+E3', ood),
        'by_case_set': {c: r.to_dict() for c, r in df.groupby(['case', 'plant_set']).success.mean().unstack().round(4).iterrows()},
        'by_case_seed_new': {c: {str(k): v for k, v in r.to_dict().items()} for c, r in
                             df[df.plant_set.isin(new)].groupby(['case', 'seed']).success.mean().unstack().round(4).iterrows()},
        'ood_by_plant': {c: r.to_dict() for c, r in
                         df[df.plant_set == 'out_of_range'].groupby(['case', 'plant']).success.mean().unstack().round(3).iterrows()},
    }
    lo = res['primary_RH3_minus_A2S_new']['ci95'][0]
    res['non_inferiority_passed'] = bool(lo > -0.05)
    txt = json.dumps(res, indent=2)
    print(txt)
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(txt)
