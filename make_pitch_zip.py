"""Build the shareable pitch package: ../wall-painter-robot-pitch.zip.

The zip is an explicit allowlist, not an exclude-list over the tree. An
allowlist cannot accidentally ship a scratch file, a frame dump, a log, or a
`.venv`, and it fails loudly if something the package needs has moved.

The package carries the source, the recorded runs, the figures, the editable
cards and the published film. The local 50-second demo masters and the
earlier unbranded film assembly are not included.

Run this after the film and docs are final. Entry count and total size are
printed so the release notes can be kept current.
"""
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT.parent / 'wall-painter-robot-pitch.zip'
PREFIX = 'wall-painter-robot/'

ENTRIES = [
    # top level
    '.gitignore', 'CITATION.cff', 'CHANGELOG.md', 'HANDOFF.md', 'LICENSE',
    'NOTICE', 'README.md', 'ROADMAP.md', 'llms.txt',
    # assets (the URDF carries the link inertials, so it must ship)
    'assets/floor_tex.png', 'assets/grit_tex.png',
    'assets/licenses/SO101-Apache-2.0.txt',
    'assets/so101.urdf', 'assets/wall_tex.png',
    # docs and deliverables
    'docs/DOCS-INDEX.md', 'docs/guide.md', 'docs/pitch.md',
    'docs/writing.md',
    'docs/wallfin_final_film.mp4', 'docs/wallfin_main_cover.png',
    'docs/coverage.png', 'docs/force_trace.png',
    'docs/summary.png',
    # editable cards (demo + film + CTA)
    'docs/cards/cta.png',
    'docs/cards/end_baseline.png', 'docs/cards/end_controlled.png',
    'docs/cards/intro_baseline.png', 'docs/cards/intro_controlled.png',
    'docs/cards/launch_01_title.png', 'docs/cards/launch_02_what.png',
    'docs/cards/launch_03_how.png', 'docs/cards/launch_04_controlled.png',
    'docs/cards/launch_05_baseline.png', 'docs/cards/launch_06_cta.png',
    # real-arm stack (runs the same control law on real servos)
    'hw/README.md', 'hw/check_kinematics.py', 'hw/feetech.py',
    'hw/force_est.py', 'hw/host.py', 'hw/kinematics.py',
    # sim source
    'sim/build_model.py', 'sim/check_demo.py', 'sim/control.py',
    'sim/edit.py', 'sim/edit_launch.py',
    'sim/make_cards.py', 'sim/make_cards_launch.py',
    'sim/make_cards_white.py', 'sim/make_charts.py',
    'sim/make_textures.py', 'sim/probe_servo.py', 'sim/probe_stuck.py',
    'sim/robot.py', 'sim/run_demo.py',
    'sim/seed_bank.npz', 'sim/seed_boost.py', 'sim/wall_geometry.py',
    # recorded runs + acceptance fingerprints + generated model
    'sim/reports/base5.npz', 'sim/reports/base5.sha256',
    'sim/reports/ctl5.npz', 'sim/reports/ctl5.sha256',
    'sim/reports/model_wallfin.xml',
    'sim/reports/shots_v2base.json', 'sim/reports/shots_v2ctl.json',
    # cards consumed by the assemblers
    'sim/reports/card_end_baseline.png', 'sim/reports/card_end_controlled.png',
    'sim/reports/card_intro_baseline.png',
    'sim/reports/card_intro_controlled.png',
    'sim/reports/card_launch_01_title.png',
    'sim/reports/card_launch_02_what.png',
    'sim/reports/card_launch_03_how.png',
    'sim/reports/card_launch_04_controlled.png',
    'sim/reports/card_launch_05_baseline.png',
    'sim/reports/card_launch_06_cta.png',
]

# Deliberately NOT shipped: the 50-second demo masters, the earlier unbranded
# film, frame dumps, edit_build, __pycache__, .venv, hw/logs, hw/robot.json,
# probe_hits.npz, probe scripts other than the two above, *.bak,
# docs/reference/, and other private or regenerable material.


def main():
    missing = [e for e in ENTRIES if not (ROOT / e).is_file()]
    if missing:
        print('MISSING:', *missing, sep='\n  ')
        return 1
    if OUT.exists():
        OUT.unlink()
    total = 0
    with zipfile.ZipFile(OUT, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for e in ENTRIES:
            data = (ROOT / e).read_bytes()
            total += len(data)
            z.writestr(PREFIX + e, data)
    mb = OUT.stat().st_size / 1e6
    print(f'wrote {OUT}  {len(ENTRIES)} entries  {mb:.1f} MB  ({total / 1e6:.1f} MB raw)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
