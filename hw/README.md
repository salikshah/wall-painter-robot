# hw — the real-arm side of WallFin

This folder runs the same sanding loop on a real Feetech STS3215 arm when one
is connected. No hardware yet? No problem. A dry mode stands in for the arm
and the wall, so everything here works today. When your arm arrives, the same
steps welcome it in.

What we check against the sim (`python check_kinematics.py`):

| check | result |
|---|---|
| pad pose vs MuJoCo | 0.001 µm max |
| gravity torque vs MuJoCo | 0.04 N·m max (hidden sim-only masses) |
| jacobian vs small step test | 2e-10 |
| IK along the sweep | ≤ 0.8 mm, tick p50 3.5 / p95 6 ms (20 ms budget) |

A full dry run (`python host.py --dry --passes 3`) gives: 100 % coverage,
force mean 4.7 N (goal 5 N), 85 % time on the wall, one seam catch, no IK
failures. Same numbers every time — pleasant for comparing changes.

## Files

| file | purpose |
|---|---|
| `kinematics.py` | arm math in plain numpy (no MuJoCo) |
| `check_kinematics.py` | checks the arm math against the sim (run after changes) |
| `feetech.py` | STS3215 serial driver + fake bus; `python feetech.py COM3` scans |
| `force_est.py` | pad force from servo loads + gravity correction |
| `host.py` | phase runner (move → acquire → sweep → retract) + safety + logs |
| `robot.json` | per-arm calibration (made by `--calib`; optional) |
| `logs/` | CSV trace + JSON numbers per run |

All control settings (force loop, sweep speed, standoff, pretilt) come from
the sim. One source of truth keeps the sim and the robot honest with each
other.

## Wiring (when your arm arrives)

- 5× STS3215 servos. IDs 1–5 are shoulder_pan, shoulder_lift, elbow_flex,
  wrist_flex, wrist_roll. Wired differently? Edit `robot.json` or `SERVOS`.
- 12 V / 5 A power supply. A half-duplex USB adapter (for example the
  Waveshare bus-servo adapter). Speed: 1 Mbaud.
- The base rail is not built yet (the plan lives in ROADMAP). For bench
  tests, lock the carriage and add `--no-base`.

## Your first hour with the arm

1. **Meet the servos.** `python feetech.py COM3` → all 5 IDs answer. Check
   12 V and normal temps.
2. **Set signs and offsets.** Hold the arm in a comfortable pose by hand.
   Read the ticks. Put `sign` and `offset_rad` for each joint into
   `robot.json`.
3. **Feel the force.** `python host.py --port COM3 --enable --hold`. The
   resting arm should read `f_est ≈ 0 ± 0.5 N`. Push the pad gently **toward
   the wall**. `f_est` must **go up**. If it goes down, flip `load_sign` for
   joints 2–4 in `robot.json`.
4. **Learn the bias.** `python host.py --port COM3 --enable --calib`. Keep
   the arm still and off the wall. This writes `tau_bias` into `robot.json`.
5. **First move — slow and gentle.** Put the arm near the start pose, then run
   `python host.py --port COM3 --enable --no-base --passes 1 --servo-speed 400 --max-force 8`.
   Keep a hand on the power switch. Ctrl+C always turns the motors off.
6. **Rehearse dry first.** `python host.py --dry --passes 3` before every
   real run. Touch the wall with the rail when the rail exists.

## Safety, the friendly kind

Robots deserve the same care we give a power drill.

- Real moves need `--enable`. Without it, host.py refuses to move.
- Ctrl+C or a crash turns torque off on all servos.
- `--estop FILE` checks a file each tick (you can hook up a foot switch later).
- Auto stop: servo temp > 70 °C, supply < 9.5 V, force > `--max-force`
  (default 20 N; use about 15 N for the first wall tests).
- First contact tests: go slow, set a low force limit, hand on the power.

## What we're still learning

- The force estimate has no inertia correction yet. Steady force is good
  (bias ≤ 0.4 N in dry runs). Quick hits at the seam can show 15–20 N spikes
  against the 5 N goal. The sim's own pad peaks at 15 N on the same feature,
  so part of the spike is the contact itself; the rest is the load estimate.
  Plans: a wrist force sensor later.
- `PRESENT_LOAD` depends on each servo's torque constant and friction. The
  per-joint `load_sign` and `max_torque_nm` are the knobs that tune accuracy.
- Windows timing: about 2 % of ticks run late on a busy laptop. The production
  target is a Raspberry Pi or another Linux board at 50 Hz.
- The rail (stepper) driver is not written yet. Use `--no-base` until then.
