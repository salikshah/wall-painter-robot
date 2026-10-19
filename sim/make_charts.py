"""PIL-only charts: force trace, coverage maps, summary bars (no matplotlib)."""
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
REP = ROOT / 'sim/reports'
DOCS = ROOT / 'docs'
DOCS.mkdir(exist_ok=True)
TARGET = 5.0

try:
    F_BIG = ImageFont.truetype('arial.ttf', 26)
    F_MED = ImageFont.truetype('arial.ttf', 20)
    F_SM = ImageFont.truetype('arial.ttf', 16)
except OSError:
    F_BIG = F_MED = F_SM = ImageFont.load_default()


def load(tag):
    d = np.load(REP / f'{tag}.npz')
    return d


def force_trace():
    ctl, base = load('ctl5'), load('base5')
    W, H = 1500, 1150
    img = Image.new('RGB', (W, H), 'white')
    dr = ImageDraw.Draw(img)
    dr.text((30, 18), 'Pad-sand force across the wall — controlled vs position-only',
            fill=(20, 20, 20), font=F_BIG)
    x0, xw = 90, 1360
    ph, pw = 120, 1580
    ph3 = 300
    for k in range(3):
        t0, t1 = 13.21 * k, 13.21 * (k + 1)
        top, bot = 120 + k * ph3, 120 + k * ph3 + 236
        fmax = 36.0
        dr.rectangle([x0, top, x0 + xw, bot], outline=(120, 120, 120))
        for fgrid in range(0, 37, 5):
            gy = bot - (fgrid / fmax) * (bot - top)
            dr.line([x0, gy, x0 + xw, gy], fill=(225, 225, 225))
            dr.text((x0 - 55, gy - 9), f'{fgrid}', fill=(90, 90, 90), font=F_SM)
        sx0 = x0 + (-0.035 + 0.14) / 0.28 * xw
        sx1 = x0 + (0.035 + 0.14) / 0.28 * xw
        dr.rectangle([sx0, top, sx1, bot], fill=(255, 246, 214))
        bx0 = x0 + (-0.075 - 0.030 + 0.14) / 0.28 * xw
        bx1 = x0 + (-0.075 + 0.030 + 0.14) / 0.28 * xw
        dr.rectangle([bx0, top, bx1, bot], fill=(226, 232, 240))
        ty = bot - (TARGET / fmax) * (bot - top)
        for gx in range(int(x0), int(x0 + xw), 14):
            dr.line([gx, ty, gx + 7, ty], fill=(130, 130, 130))
        for tag, col in [('ctl5', (30, 110, 175)), ('base5', (200, 70, 30))]:
            d = load(tag)
            m = (d['phase'] == 2) & (d['t'] >= t0) & (d['t'] < t1)
            xs, fs = d['cmd'][m, 0], d['force'][m]
            pts = [(x0 + (xv + 0.14) / 0.28 * xw, bot - min(fv, fmax) / fmax * (bot - top))
                   for xv, fv in zip(xs, fs)]
            dr.line(pts, fill=col, width=3)
        dr.text((x0 + 8, top + 6), f'pass {k + 1}   z = {0.34 + 0.03 * k:.2f} m',
                fill=(40, 40, 40), font=F_MED)
    dr.text((x0, 84), 'force [N]', fill=(60, 60, 60), font=F_MED)
    dr.line([1320, 60, 1360, 60], fill=(30, 110, 175), width=5)
    dr.text((1370, 48), 'controlled', fill=(30, 110, 175), font=F_MED)
    dr.line([1320, 88, 1360, 88], fill=(200, 70, 30), width=5)
    dr.text((1370, 76), 'baseline', fill=(200, 70, 30), font=F_MED)
    dr.text((x0, H - 40), 'x along wall [m]:   -0.14 … +0.14 (seam at 0 shaded; blob x=-0.075)',
            fill=(90, 90, 90), font=F_MED)
    img.save(DOCS / 'force_trace.png')
    print('saved docs/force_trace.png')


def coverage():
    W, H = 1900, 620
    img = Image.new('RGB', (W, H), 'white')
    dr = ImageDraw.Draw(img)
    dr.text((30, 18), 'Contact coverage of the 28 cm × 20 cm target strip',
            fill=(20, 20, 20), font=F_BIG)
    for n, (tag, label) in enumerate([('ctl5', 'controlled'), ('base5', 'baseline')]):
        d = load(tag)
        ct = d['cov_time']
        nz, nx = ct.shape
        frac = 100.0 * float((ct >= 0.02).mean())
        cell = 30
        ox = 110 + n * 800
        oy = 110
        dr.text((ox, oy - 34), f'{label} — {frac:.0f} % of cells contacted', fill=(40, 40, 40), font=F_MED)
        for iz in range(nz):
            for ix in range(nx):
                v = ct[nz - 1 - iz, ix]
                col = (228, 231, 235) if v < 0.02 else (30, 110, 175)
                px = ox + ix * cell
                py = oy + iz * cell
                dr.rectangle([px, py, px + cell - 2, py + cell - 2], fill=col)
        dr.rectangle([ox, oy, ox + nx * cell, oy + nz * cell], outline=(90, 90, 90))
        dr.text((ox, oy + nz * cell + 10), 'x: -0.14 … +0.14 m', fill=(90, 90, 90), font=F_SM)
    dr.text((50, 250), 'z', fill=(90, 90, 90), font=F_SM)
    img.save(DOCS / 'coverage.png')
    print('saved docs/coverage.png')


def summary():
    ctl, base = load('ctl5'), load('base5')
    ctl_f, base_f = ctl['force'][ctl['phase'] == 2], base['force'][base['phase'] == 2]
    ctl_e = np.abs(ctl_f - TARGET)
    base_e = np.abs(base_f - TARGET)
    ctl_cov = 100.0 * float((ctl['cov_time'] >= 0.02).mean())
    base_cov = 100.0 * float((base['cov_time'] >= 0.02).mean())
    rows = [
        ('contact duty [%]', 100.0 * float((ctl_f >= 1.5).mean()),
         100.0 * float((base_f >= 1.5).mean()), 100),
        ('coverage [%]', ctl_cov, base_cov, 100),
        ('mean |F-5N| [N]', ctl_e.mean(), base_e.mean(), 10),
        ('p95 |F-5N| [N]', np.percentile(ctl_e, 95), np.percentile(base_e, 95), 16),
    ]
    W, H = 1300, 640
    img = Image.new('RGB', (W, H), 'white')
    dr = ImageDraw.Draw(img)
    dr.text((30, 18), 'Controlled (force loop) vs baseline (fixed depth)', fill=(20, 20, 20), font=F_BIG)
    xbar, bw, gap = 300, 100, 30
    for r, (label, vc, vb, vmax) in enumerate(rows):
        oy = 100 + r * 125
        dr.text((30, oy + 34), label, fill=(40, 40, 40), font=F_MED)
        for j, (v, col) in enumerate([(vc, (30, 110, 175)), (vb, (200, 70, 30))]):
            h = max(4, int(v / vmax * 90))
            px = xbar + j * (bw + gap)
            dr.rectangle([px, oy + 100 - h, px + bw, oy + 100], fill=col)
            dr.text((px + 2, oy + 108), f'{v:.1f}', fill=col, font=F_MED)
    dr.line([xbar - 20, 90, xbar - 20, 100 + 3 * 125 + 100], fill=(120, 120, 120))
    img.save(DOCS / 'summary.png')
    print('saved docs/summary.png')


if __name__ == '__main__':
    force_trace()
    coverage()
    summary()
