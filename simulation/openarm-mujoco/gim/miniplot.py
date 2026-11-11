"""Tiny Pillow line plotter (matplotlib is not installed in .venv-openarm)."""
import numpy as np
from PIL import Image, ImageDraw, ImageFont

COLORS = [(31, 119, 180), (214, 39, 40), (44, 160, 44), (255, 127, 14), (148, 103, 189), (140, 86, 75), (0, 0, 0)]


def _font(size):
    try:
        return ImageFont.load_default(size)
    except TypeError:
        return ImageFont.load_default()


def _ticks(lo, hi, n=5):
    if hi - lo < 1e-12:
        hi = lo + 1
    raw = (hi - lo) / n
    mag = 10 ** np.floor(np.log10(raw))
    step = mag * min([1, 2, 5, 10], key=lambda s: abs(s * mag - raw))
    t = np.arange(np.ceil(lo / step) * step, hi + step * 1e-9, step)
    return t, step


def plot(path, panels, ncols=2, cell=(640, 400)):
    """panels: list of dict(title, xlabel, ylabel, series=[(label, x, y)], hline=[values])."""
    f, fs = _font(14), _font(12)
    rows = (len(panels) + ncols - 1) // ncols
    img = Image.new('RGB', (cell[0] * ncols, cell[1] * rows), 'white')
    dr = ImageDraw.Draw(img)
    for k, p in enumerate(panels):
        ox, oy = (k % ncols) * cell[0], (k // ncols) * cell[1]
        L, R, T, B = 70, 15, 35, 50
        x0, x1, y0, y1 = ox + L, ox + cell[0] - R, oy + T, oy + cell[1] - B
        xs = np.concatenate([np.asarray(s[1], float) for s in p['series']])
        ys = np.concatenate([np.asarray(s[2], float) for s in p['series']] + [np.asarray(p.get('hline', []), float)])
        ok = np.isfinite(xs) & np.isfinite(ys[:len(xs)]) if len(ys) == len(xs) else np.isfinite(xs)
        xl, xh = np.nanmin(xs), np.nanmax(xs)
        yl, yh = np.nanmin(ys), np.nanmax(ys)
        pad = 0.05 * (yh - yl if yh > yl else 1)
        yl, yh = yl - pad, yh + pad
        sx = lambda v: x0 + (v - xl) / ((xh - xl) or 1) * (x1 - x0)
        sy = lambda v: y1 - (v - yl) / ((yh - yl) or 1) * (y1 - y0)
        dr.rectangle([x0, y0, x1, y1], outline='black')
        for t in _ticks(xl, xh)[0]:
            dr.line([sx(t), y1, sx(t), y1 + 4], fill='black')
            dr.text((sx(t), y1 + 6), f'{t:.3g}', fill='black', font=fs, anchor='ma')
        for t in _ticks(yl, yh)[0]:
            dr.line([x0 - 4, sy(t), x0, sy(t)], fill='black')
            dr.line([x0, sy(t), x1, sy(t)], fill=(225, 225, 225))
            dr.text((x0 - 6, sy(t)), f'{t:.3g}', fill='black', font=fs, anchor='rm')
        for v in p.get('hline', []):
            dr.line([x0, sy(v), x1, sy(v)], fill=(150, 150, 150))
        dr.text(((x0 + x1) / 2, oy + 8), p.get('title', ''), fill='black', font=f, anchor='ma')
        dr.text(((x0 + x1) / 2, oy + cell[1] - 22), p.get('xlabel', ''), fill='black', font=fs, anchor='ma')
        dr.text((ox + 6, (y0 + y1) / 2), p.get('ylabel', ''), fill='black', font=fs, anchor='lm')
        for i, (lab, x, y) in enumerate(p['series']):
            c = COLORS[i % len(COLORS)]
            pts = [(sx(a), sy(b)) for a, b in zip(x, y) if np.isfinite(a) and np.isfinite(b)]
            if len(pts) > 1:
                dr.line(pts, fill=c, width=2)
            elif pts:
                dr.ellipse([pts[0][0] - 3, pts[0][1] - 3, pts[0][0] + 3, pts[0][1] + 3], fill=c)
            dr.text((x0 + 8, y0 + 4 + 14 * i), lab, fill=c, font=fs)
    img.save(path)
