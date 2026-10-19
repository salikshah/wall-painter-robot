# WallFin - handoff and restart notes

**Stopped:** 2026-10-19

**State:** public v1.0.0 proof of concept. Simulation, real-arm software and
launch film are complete. No physical hardware.

## Live

- Repository: https://github.com/salikshah/wall-painter-robot
- Film: https://salikshah.github.io/wall-painter-robot/wallfin_final_film.mp4
- Pages root: https://salikshah.github.io/wall-painter-robot/
- Contact: https://www.linkedin.com/in/salikshah/ · salik.shah@gmail.com

## Complete

- MuJoCo demo and fixed-depth baseline; nine acceptance gates pass.
- Real-arm `hw/` stack; kinematics cross-check against MuJoCo passes.
- Branded 1:48 film, cover, cards and figures.
- Public README, guide, pitch, roadmap, docs index and citation metadata.
- GitHub Pages serves the film as `video/mp4` with range requests.
- Repository is public; only the final film is published as video.

## Open

- Upload the 1:48 film to YouTube. Description and tags are below.
- Optional LinkedIn launch post; the repository URL is live.
- Hardware path: buy the BOM, bench bring-up, first wall contact, then a full
  sweep. Budget about 3 months after parts arrive, followed by dust extraction,
  calibration, safety and pilots. See `ROADMAP.md` P0.5.
- P1 skills: tool changer, mural mode, realism pass, corner head, ROS2 export.
- Re-run `hw/host.py --dry` on the target machine. A release-time run with
  other checks running in parallel logged 519 overruns in 1989 control ticks;
  the kinematics-only check passes, so this is a timing/load issue to confirm.
- Local demo masters, frame dumps and old previews remain git-ignored; do not
  add them to the public repository.

## Restart checklist

```bash
git pull
# activate the virtual environment first
cd sim
python check_demo.py
cd ..
python hw/check_kinematics.py
python hw/host.py --dry --passes 3
```

Then read `README.md`, `ROADMAP.md` and `docs/DOCS-INDEX.md` section 7 before
changing the model. Canonical fingerprints: `ctl5 = e208219d5ea24829`,
`base5 = 804c7deb8551ad7d`.

## YouTube upload copy

**Title**

WallFin - Open-Source Wall Sanding Robot (Proof of Concept)

**Description**

WallFin is an open-source wall sanding and finishing robot - a simulation-first proof of concept.

This film compares a controlled 5 N sanding pass with a baseline. In simulation, the controlled pass holds contact 97.8% of the time and covers 100% of the wall, versus 61.6% and 71.9% for the baseline. The same force loop is implemented for a real SO-101 arm with Feetech servos.

Code, build guide and results: https://github.com/salikshah/wall-painter-robot

A physical prototype is a ~3-month project with a small team, followed by dust extraction, calibration, safety and pilots. If you build one, I'd like to hear how it goes.

Salik Shah
LinkedIn: https://www.linkedin.com/in/salikshah/
GitHub: https://github.com/salikshah
Email: salik.shah@gmail.com

#robotics #opensource #constructiontech #drywall #automation

**Tags (498/500 characters)**

```text
WallFin, wall sanding robot, wall finishing robot, drywall sander, drywall sanding, drywall finishing, drywall robot, sanding robot, wall sanding, wall finishing, construction robot, construction robotics, construction automation, painting robot, plastering robot, wall putty, robot arm, SO-101, Feetech, MuJoCo, robot simulation, force control, admittance control, open source, DIY robot, physical AI, embodied AI, automation, contech, Salik Shah, drywall repair, dust extraction, proof of concept
```
