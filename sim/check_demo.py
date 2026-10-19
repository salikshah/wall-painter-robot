"""Acceptance gates for the wall-sanding demo runs (controlled + baseline).

Run after:
  python run_demo.py --mode controlled --passes 3 --save --tag ctl5
  python run_demo.py --mode baseline  --passes 3 --save --tag base5
"""
import hashlib
from pathlib import Path

import numpy as np

import wall_geometry as wg

ROOT = Path(__file__).resolve().parent.parent
REP = ROOT / 'sim/reports'
TARGET = wg.F_DES


def sweep_stats(tag):
    d = np.load(REP / f'{tag}.npz')
    sw = d['phase'] == 2
    x, f = d['cmd'][sw, 0], d['force'][sw]
    err = np.abs(f - TARGET)
    seam = np.abs(x) < 0.055
    blob = np.abs(x + 0.075) < 0.065
    steady = ~(seam | blob)
    return {
        'duty': float((f >= wg.F_CONTACT_MIN).mean()),
        'err_mean': float(err.mean()),
        'err_p95': float(np.percentile(err, 95)),
        'fmax': float(f.max()),
        'steady_mean': float(err[steady].mean()),
        'steady_p95': float(np.percentile(err[steady], 95)),
        'coverage': float((d['cov_time'] >= 0.02).mean()),
        'pe_max': float(d['pe'][sw].max()) * 1000,
    }


def hash_matches(tag):
    d = np.load(REP / f'{tag}.npz')
    h = hashlib.sha256()
    for k in ['t', 'force', 'pad', 'cmd']:
        h.update(np.round(d[k], 6).tobytes())
    want = (REP / f'{tag}.sha256').read_text().strip()
    return h.hexdigest() == want, h.hexdigest()[:16]


def main():
    ctl = sweep_stats('ctl5')
    base = sweep_stats('base5')
    checks = [
        ('controlled  ik pe max <= 1.0 mm', ctl['pe_max'] <= 1.0, f"{ctl['pe_max']:.2f} mm"),
        ('controlled  contact duty >= 0.90', ctl['duty'] >= 0.90, f"{ctl['duty']:.3f}"),
        ('controlled  coverage >= 0.98', ctl['coverage'] >= 0.98, f"{ctl['coverage']:.3f}"),
        ('controlled  steady mean|e| <= 1.5 N', ctl['steady_mean'] <= 1.5,
         f"{ctl['steady_mean']:.2f} N"),
        ('controlled  steady p95|e| <= 4.5 N', ctl['steady_p95'] <= 4.5,
         f"{ctl['steady_p95']:.2f} N"),
        ('controlled  peak force <= 35 N', ctl['fmax'] <= 35.0, f"{ctl['fmax']:.1f} N"),
        ('comparison  duty gain >= 0.25', ctl['duty'] - base['duty'] >= 0.25,
         f"{ctl['duty'] - base['duty']:+.3f}"),
        ('comparison  err_mean <= 0.5 x baseline', ctl['err_mean'] <= 0.5 * base['err_mean'],
         f"{ctl['err_mean']:.2f} vs {base['err_mean']:.2f} N"),
        ('comparison  coverage gain >= 0.20', ctl['coverage'] - base['coverage'] >= 0.20,
         f"{ctl['coverage'] - base['coverage']:+.3f}"),
    ]
    ok_all = True
    for name, ok, val in checks:
        print(f'{"PASS" if ok else "FAIL"}  {name}   [{val}]')
        ok_all = ok_all and ok
    for tag in ['ctl5', 'base5']:
        ok, h = hash_matches(tag)
        print(f'{"PASS" if ok else "FAIL"}  hash {tag}.npz matches recorded sha256 [{h}]')
        ok_all = ok_all and ok
    print()
    print('ALL GATES PASS' if ok_all else 'GATE FAILURES PRESENT')
    return 0 if ok_all else 1


if __name__ == '__main__':
    raise SystemExit(main())
