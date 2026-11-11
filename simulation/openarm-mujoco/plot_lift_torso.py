"""Plot results/lift_torso_sizing/sizing_arrays.npz (system Python with matplotlib)."""
from pathlib import Path
import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
OUT = Path(__file__).resolve().parent / 'results' / 'lift_torso_sizing'
a = np.load(OUT / 'sizing_arrays.npz')
pitch = np.degrees(a['pitches']); j1 = np.degrees(a['j1s'])
plt.rcParams.update({'font.size': 10, 'savefig.dpi': 200})
fig, ax = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
for c, p in enumerate(('0', '3')):
    g = np.abs(a[f'grid_tau_straight_{p}'])
    im = ax[0, c].contourf(j1, pitch, g, levels=np.arange(0, 160, 10), cmap='viridis')
    cs = ax[0, c].contour(j1, pitch, g, levels=[50], colors='w', linewidths=1.5)
    ax[0, c].clabel(cs, fmt='50 Nm (X10-100 rated)')
    ax[0, c].set(xlabel='shoulder angle joint1 (deg, 0 = arms along torso, + = forward)', ylabel='torso bend (deg)',
                 title=f'Torso bend holding torque, elbows straight, {p} kg/hand')
    fig.colorbar(im, ax=ax[0, c], label='Nm')
    for T in ('2', '1', '0.6'):
        ax[1, c].plot(a[f'bend_t_{p}_{T}'], a[f'bend_tau_{p}_{T}'], label=f'{T} s')
    ax[1, c].axhline(50, color='k', ls='--', lw=1); ax[1, c].text(0.02, 53, 'X10-100 rated 50 Nm', fontsize=9)
    ax[1, c].set(xlabel='time (s)', ylabel='torque (Nm)', title=f'Bend 0 to 90 deg, worst arm pose, {p} kg/hand')
    ax[1, c].legend(title='bend time'); ax[1, c].grid(alpha=0.3)
fig.savefig(OUT / 'torso_pitch_sizing.png')
print('saved')
