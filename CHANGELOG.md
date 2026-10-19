# Changelog

All notable public changes to WallFin are recorded here. Version numbers follow
`CITATION.cff`.

## [1.0.0] - 2026-10-19

First public release: a simulation-first proof of concept for a wall sanding
and finishing robot.

### Added

- MuJoCo drywall-sanding demo: a 610 mm test wall with a 4 mm seam, 1.5° lean
  and 2.5 mm screw head; three sweep passes and a visible coverage map.
- 50 Hz force/admittance loop with a fixed-depth baseline for comparison.
- Seeded, repeatable runs and nine acceptance gates in `sim/check_demo.py`.
- `hw/` real-arm stack for Feetech STS3215 servos: plain-numpy kinematics,
  load-based force estimate, serial driver and fake bus, safety stops,
  dry-run host, calibration and wiring guide.
- Branded 1:48 launch film (`docs/wallfin_final_film.mp4`) and cover.
- GitHub Pages film playback; no other video masters are published.
- Public documentation: README, builder's guide, pitch, roadmap, docs index,
  writing guide and `llms.txt`.
- Apache-2.0 license, NOTICE, CITATION.cff and LinkedIn contact details.

### Verified

- `python sim/check_demo.py`: all nine gates pass; `ctl5` and `base5` hashes
  match the recorded fingerprints.
- `python hw/check_kinematics.py`: all kinematics checks pass (FK, gravity,
  Jacobian, warm-started IK and sweep grid).
- The published film endpoint serves `video/mp4` with byte-range support.
- The README cover and film link render on the public repository page.

### Known limits

- No physical hardware yet; the real-arm stack is dry-rehearsed only.
- Hardware force is inferred from servo load, not a dedicated wrist sensor.
- Dust extraction, calibration, safety and pilots are prototype-stage work.
- Full 1080p renders take hours; frame dumps are not stored in the repository.
- A dry host run during release verification logged loop overruns under
  concurrent CPU load. Re-run timing on the target machine before trusting it.
