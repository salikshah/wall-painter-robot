# WallFin — documentation index

**What this project is.** WallFin is a physics simulation of a small robot arm
that sands drywall. A force loop holds a steady 5 N push while the arm sweeps
the wall, and the wall visibly changes as coverage is achieved. The same run
is repeated with a fixed-depth controller so the two can be compared side by
side. Everything runs on a laptop with free software (MuJoCo + Python). A
parallel `hw/` stack is written for real Feetech STS3215 servos and runs the
same control law in dry rehearsal.

**What this file is.** A map of every document, deliverable, script and
recorded run, plus the commands that regenerate them. Start here after
`HANDOFF.md` and the README.

---

## 1. Read in this order

| Order | File | Why |
|---|---|---|
| 0 | `HANDOFF.md` | Restart point: live URLs, open work and the first commands. |
| 1 | `README.md` | The concept, results, quick start and honest limits. |
| 2 | `docs/guide.md` | A 14-section builder's guide, from zero to a real wall. |
| 3 | `docs/pitch.md` | The one-page version. |
| 4 | `ROADMAP.md` | Simulation, prototype, product — stage by stage. |
| 5 | `hw/README.md` | Wiring, calibration and the first hour with a real arm. |
| 6 | `sim/control.py` | The force loop. It is short and it is the heart. |
| 7 | `CHANGELOG.md` | Public release history. |

---

## 2. Deliverables

| What | Where | Notes |
|---|---|---|
| Launch film | `docs/wallfin_final_film.mp4` | 1:48 (108.46 s), 1920×1080, 50 fps, 5422 frames, 31.4 MiB, AAC stereo. Branded intro, both demos, CTA. |
| Film poster | `docs/wallfin_main_cover.png` | 1088×608 cover used as the README poster. |
| Figures | `docs/force_trace.png`, `docs/coverage.png`, `docs/summary.png` | Built from `ctl5.npz` and `base5.npz` only. |
| Editable cards | `docs/cards/` | 4 demo cards + 6 film cards + `cta.png` (11 PNGs). Source scripts in §3. |
| One-page summary | `docs/pitch.md` | Investor / collaborator friendly. |
| Builder's guide | `docs/guide.md` | Long-form tutorial with exercises. |
| Writing rules | `docs/writing.md` | Voice, grammar and claim discipline. |
| Restart notes | `HANDOFF.md` | Live URLs, open tasks and YouTube upload copy. |
| Release history | `CHANGELOG.md` | Public changes and verified release state. |

The film is the published cut and carries both comparisons. The 50-second
demo masters are local working files (git-ignored); §6 rebuilds them.

The published film is uniform `50/1` constant frame rate, H.264 video and AAC
audio. The earlier mixed-fps title-card defect was found and fixed.

---

## 3. Simulation source — `sim/`

### Core pipeline

| File | Role |
|---|---|
| `wall_geometry.py` | Scene constants and the analytic wall surface (seam, lean, screw head). |
| `robot.py` | Arm wrapper, seeded IK, force readout. |
| `control.py` | Force loop, phase logic and coverage grid. |
| `build_model.py` | MJCF builder — arm, wall, toolhead, paint overlay, studio lighting and materials. |
| `run_demo.py` | Phase runner (move → acquire → sweep → retract), HUD, camera director, renderer. |
| `check_demo.py` | Acceptance gates and fingerprint verification. |
| `make_textures.py` | Procedural wall, grit and floor textures. |
| `make_charts.py` | Builds the three figures in `docs/`. |
| `make_cards_white.py` | White intro/end cards for the two demos. Writes `docs/cards/` and `sim/reports/`. |
| `make_cards_launch.py` | Six film cards plus the standalone CTA. Writes `docs/cards/` and `sim/reports/`. |
| `make_cards.py` | Older dark-card generator, kept for reference. The published cuts use `make_cards_white.py`. |
| `edit.py` | Demo assembly: grading, captions, strip/graph overlay, cards, concat, web cuts. |
| `edit_launch.py` | Development assembly for the earlier unbranded film. The published 1:48 film is the branded cut. |
| `seed_bank.npz`, `seed_boost.py` | Saved IK poses known to reach the wall. |
| `probe_stuck.py`, `probe_servo.py` | Two published diagnostic probes. Other `probe_*.py` files are local scratch and are git-ignored. |

### Recorded runs — `sim/reports/`

| File | What |
|---|---|
| `ctl5.npz` / `ctl5.sha256` | Controlled run, 3 passes. Fingerprint `e208219d5ea24829`. |
| `base5.npz` / `base5.sha256` | Fixed-depth run, 3 passes. Fingerprint `804c7deb8551ad7d`. |
| `model_wallfin.xml` | The generated MJCF actually loaded by MuJoCo. Commit it; it makes model diffs readable. |
| `card_intro_{controlled,baseline}.png`, `card_end_{controlled,baseline}.png` | Demo cards consumed by `edit.py`. |
| `card_launch_01…06_*.png` | Film cards, duplicated from `docs/cards/` for the assembler. |
| `shots_v2ctl.json`, `shots_v2base.json` | Camera shot manifests for the two 1080p renders. |
| `frames_v2ctl/`, `frames_v2base/` | Rendered PNGs, 2425 and 2423 frames after card removal. Git-ignored; deleted after encoding to save space. |
| `edit_build/` | Encoded segments, strips and panels. Git-ignored, about 35 MiB. |

Frames are written to `sim/reports/frames_<tag>/` by `run_demo.py`. A full
1080p render takes roughly **2 hours per mode** on the current laptop. Use a
450 or 720 preview first; the frame count and shot plan are printed at the
start. Re-render before running `edit.py` or `edit_launch.py`, because the
frames are not kept in the repository.

---

## 4. Real-arm code — `hw/`

Runs without MuJoCo. Same force law as the sim.

| File | Role |
|---|---|
| `kinematics.py` | Plain-numpy arm math. |
| `check_kinematics.py` | Checks the hw math against the sim. |
| `feetech.py` | STS3215 serial driver plus a fake bus for testing. |
| `force_est.py` | Pad force from servo load and gravity correction. |
| `host.py` | Phase runner, safety stops, logging. |
| `README.md` | Wiring, calibration and the first hour. |
| `logs/` | Recorded dry-run sessions. Git-ignored. |

Dry rehearsal (`python hw/host.py --dry --passes 3`): 100 % coverage, mean
force 4.7 N against the 5 N goal, 85 % time on the wall, one seam catch, no
IK failures. Repeatable.

---

## 5. Assets — `assets/`

| File | What |
|---|---|
| `so101.urdf` | SO-101 robot description — **authoritative joint origins, axes and limits.** |
| `licenses/SO101-Apache-2.0.txt` | SO-101 / SO-ARM100 attribution (TheRobotStudio). |
| `wall_tex.png`, `grit_tex.png`, `floor_tex.png` | Procedural studio textures. Required by the model. |
| `floor_marker.png` | UV-mapping debug quadrants. Not used in the published renders. |

---

## 6. Commands

Use a virtual environment. The commands below assume it is active; on Windows,
`.venv\Scripts\activate`; on macOS or Linux, `source .venv/bin/activate`.

```bash
python -m venv .venv
pip install mujoco numpy pillow
```

The demo videos also need `ffmpeg` on `PATH` for encoding and concatenation.

### Reproduce the verified runs

```bash
cd sim
python run_demo.py --mode controlled --passes 3 --save --tag ctl5
python run_demo.py --mode baseline  --passes 3 --save --tag base5
python check_demo.py
```

You should see `ALL GATES PASS` and both fingerprints. The physics-only runs
take a few minutes each.

### Hardware checks (no hardware needed)

```bash
python hw/check_kinematics.py
python hw/host.py --dry --passes 3
```

### Figures, cards and video

```bash
cd sim
python make_textures.py
python make_charts.py
python make_cards_white.py
python make_cards_launch.py

# masters (hours; preview first)
python run_demo.py --mode controlled --passes 3 --render --res 1080 --shots full --tag v2ctl
python run_demo.py --mode baseline  --passes 3 --render --res 1080 --shots full --tag v2base

# assembly
python edit.py          # both demo videos, ~25 min per mode
python edit_launch.py   # combined film, a few minutes
```

`edit.py` and `edit_launch.py` read the white cards from `sim/reports/`, so
run the card scripts before the assemblers. If you edit card text, rerun both
assemblers. These scripts produce the local demo masters and the earlier
unbranded assembly; the published branded film is `docs/wallfin_final_film.mp4`.

### Shareable package

```bash
# from the repository root
python make_pitch_zip.py
```

The script uses an explicit allowlist and writes
`../wall-painter-robot-pitch.zip`. It fails loudly if a required file is
missing. Frame dumps, logs, caches and private material are never included.
The package carries the published film and poster, not the local demo masters.

---

## 7. Ground rules when changing the model

1. **Acceptance fingerprints must not drift.** `ctl5 = e208219d5ea24829`,
   `base5 = 804c7deb8551ad7d`. These gate every release.
2. **Visual geoms must stay physics-neutral.** Any geom used purely for looks
   needs `density="0"` and `contype="0" conaffinity="0"`. Link inertia comes
   from the URDF `<inertial>` blocks, so changing geometry that carries
   `density="0"` cannot move the fingerprints. Changing a mass-bearing geom
   will break them.
3. **Verify after every model edit**, before committing to a long render.
   Re-run the controlled save to a scratch tag, hash `t` / `force` / `pad` /
   `cmd` rounded to 6 dp, and confirm it still equals `e208219d5ea24829`.
   Then run `check_demo.py`.
4. **Preview before rendering.** A full 1080p cycle is about two hours per
   mode.
5. **Keep docs portable.** No absolute paths, no machine names, no private
   scratch directories. Paths are relative to the repository root.

---

## 8. Design notes

The published videos are final for this model.

**Arm proportions.** Joint positions are parsed directly from
`assets/so101.urdf`. The visual housings were once too wide: about 57 mm
against a real SO-101 servo of about 21 mm. Since joint-to-joint spacing is
only 64.8 mm at `shoulder_lift → elbow_flex`, consecutive housings left just
7.8 mm of visible bare link, and the arm read as one continuous mass. The fix
in `build_model.py` uses 27.6 mm pods, 23 mm link shells and 26.4 mm dark
collars, giving 36–38 mm of visible bone at the tight junctions. The pad stays
at its realistic 100 mm diameter; the toolhead was slimmed to match.

**Arm posture.** The task is exactly determined (5 joints, 6 constraints), so
the fold is the unique solution; a strong joint-centring term collapses pad
axis alignment instead. A weak centring term (0.004) is kept because it helps
the solver at the railed-back geometry (IK position error 0.80 mm vs 1.00 mm;
steady force p95 3.0 N vs 5.0 N). The real issue was working distance. The
rail was too close to the pad; `ARM_BASE` is now set back to `y = 0.15`,
giving 0.40 m of working distance. The arm works extended (shoulder 67 %,
elbow 75 %) at 0.00 mm / 0.96° accuracy. Moving the rail back softened the
contact: peak force 33.5 N → 15.0 N and contact duty 94.4 % → 97.8 %.
`y = 0.08` is a dead zone (duty 0.87, peak 57 N). Do not use it.

**Test wall.** 610 mm wide (two boards, 300 mm and 310 mm), 4 mm seam, 1.5°
lean, 2.5 mm screw head. The sweep covers a 280 mm strip over three passes at
z = 340, 370 and 400 mm.

---

## 9. Generated and ignored files

| Location | Contents |
|---|---|
| `sim/reports/frames_*/` | Rendered PNG frames. Rebuild with `run_demo.py`. |
| `sim/reports/edit_build/` | Video assembly intermediates. |
| `sim/reports/shots_prev*.json` | Old shot manifests. |
| `hw/logs/`, `hw/robot.json` | Local run logs and per-arm calibration. |
| `sim/probe_hits.npz`, `sim/cam_*.png` | Local probe output. |
| `docs/_check_*.png`, `docs/reference/` | Local previews and private reference material. |
| `docs/controlled.mp4`, `docs/baseline.mp4`, `docs/*_720p.mp4`, `docs/wallfin_film.mp4`, `docs/hero.gif` | Local demo masters and the earlier unbranded film. |
| `*.bak`, `__pycache__/`, `.venv/` | Backups, bytecode, virtual environment. |

None of these are published. The repository keeps the source, recorded run
data, figures, cards and final videos.

---

## 10. Quick orientation for a new contributor

1. Read `README.md` for the concept and the honest comparison against fixed
   depth.
2. Read `docs/guide.md` if you want the ideas from zero.
3. Read `sim/control.py` — the force loop is the heart and it is short.
4. Read `assets/so101.urdf` before touching `build_model.py`.
5. Check §7 here before changing anything visual, and keep the fingerprints.
6. Returning after a pause? Start with `HANDOFF.md`, then run its restart
   checklist.
