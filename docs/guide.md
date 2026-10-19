# WallFin — a builder's guide

This guide explains WallFin from zero. It is written for a student, a curious engineer, or a founder who wants to understand what the robot does, why it is built this way, and what it would take to make it real.

You do not need a robotics background. You do need a little Python, and the willingness to run a few commands. Every idea is tied to a file you can open.

**How to read this guide**

- In a hurry? Read [Force control](#7-force-control) and [The real arm](#10-the-real-arm).
- Want the full picture? Read in order. Each section builds on the last.
- Want to run it first? Follow the quick start in the [README](../README.md), then come back.

**Contents**

1. [Start here](#1-start-here)
2. [The job](#2-the-job)
3. [The machine](#3-the-machine)
4. [Simulation 101](#4-simulation-101)
5. [The wall](#5-the-wall)
6. [Kinematics and IK](#6-kinematics-and-ik)
7. [Force control](#7-force-control)
8. [Coverage and metrics](#8-coverage-and-metrics)
9. [Reproducibility](#9-reproducibility)
10. [The real arm](#10-the-real-arm)
11. [From demo to product](#11-from-demo-to-product)
12. [The landscape](#12-the-landscape)
13. [Exercises](#13-exercises)
14. [Glossary and next steps](#14-glossary-and-next-steps)

## 1. Start here

WallFin is a robot that sands drywall. It presses a pad against the wall with a steady **5 N** push and sweeps the surface without gaps. The wall is not flat; the robot has to feel it and adjust.

The project has three parts:

| Part | What it is | Where |
|---|---|---|
| Simulation | A MuJoCo model of the arm, wall and tool, with a force loop | `sim/` |
| Real-arm stack | The same control law for Feetech STS3215 servos, no MuJoCo | `hw/` |
| Story | Videos, figures, a one-page pitch, this guide | `docs/` |

The simulation works today and repeats exactly. The real-arm code is written and dry-rehearsed. The rest of this guide explains what exists, what it proves, and where the engineering work remains.

A useful mental model: the simulation is a **digital twin**. It is close enough to test control decisions, but it is not the wall in your house. Section 10 is about the gap.

## 2. The job

Drywall is the gypsum board that makes most interior walls. Finishing it happens in stages:

1. Hang the boards.
2. Tape the seams and cover them with joint compound ("mud").
3. Sand the dried compound smooth.
4. Prime and paint.

Sanding is dusty, repetitive work. The dust can contain silica, which is a health hazard. The work also asks for steady pressure and a lot of overhead reaching. It is one of the last big construction jobs still done mostly by hand.

Robots already do parts of this. Canvas builds a drywall machine that sprays and sands to a high finish. Okibo builds an autonomous painter and sander. PaintJet paints building exteriors from a lift. They are large, capital-scale machines aimed at large commercial projects.

Small renovation and repainting jobs are different. The rooms are small, the walls are irregular, and the budget is tight. A big machine does not fit the job.

That is the opening WallFin studies: a small arm, a rail, one tool at a time, and force control good enough to touch a wall carefully. The first tool is a sander because sanding is the dusty, skill-dependent step — and because it is the best test of contact control.

## 3. The machine

The robot in the simulation is a stylized **SO-101**: a small, 3D-printable arm designed by TheRobotStudio. WallFin uses five joints:

| Joint | What it does |
|---|---|
| `shoulder_pan` | turns the arm left and right |
| `shoulder_lift` | raises the upper arm |
| `elbow_flex` | bends the elbow |
| `wrist_flex` | tilts the tool up and down |
| `wrist_roll` | rolls the tool around its axis |

The tool is a sanding head: a motor housing, a soft pad, and a 100 mm disc with a wide dome on its face. The dome matters. Think of a beach ball against a wall: tilt it, and it still touches at one point. A flat pad would rock on its edge; a domed pad keeps the contact predictable.

The arm sits on a **vertical rail**. The rail carries the arm up and down between passes. In the real product it will also carry the arm along the wall. The rail is not just transport; it sets the working distance. Set it too close and the arm folds against its joint limits. Set it back and the arm works extended, with room to move. This is why the base sits at `y = 0.15` in `sim/wall_geometry.py`, about 0.40 m from the wall. At `y = 0.08` the arm cannot reach comfortably — a dead zone.

The test cell is deliberately small: a 610 mm wall, three horizontal passes, and a pad that overlaps each pass. The point is not to paint a room. The point is to make the contact problem visible and measurable.

## 4. Simulation 101

MuJoCo is a physics engine built for contact-rich robotics. It takes a model written in **MJCF** (an XML format), steps it forward in time, and reports positions, velocities and contact forces.

The model is built in `sim/build_model.py`. It contains:

- **bodies** — the base, links, wrist and tool;
- **joints** — the five arm joints, plus the rail carriage;
- **geoms** — the shapes used for collision and visuals;
- **actuators** — the motors that hold joint positions;
- **materials and lights** — the studio look for the renders.

The simulation steps at **500 Hz** (`DT = 0.002` s). The controller runs every tenth step, at **50 Hz** (`CTRL_DECIM = 10`). That split is normal: physics needs small steps for stable contact; the control loop can run slower.

Contact is soft on purpose. The pad's collision settings (`solref 0.04 1`, `solimp 0.7 0.9 0.01`) let the wall push back gradually instead of acting like a brick. Soft contact forgives small mistakes and keeps the force signal smooth. In the working range, the wall pushes back about 6–12 N per millimetre of depth.

Why simulate at all?

- It is safe. A bad controller cannot hurt anyone or break a wall.
- It is repeatable. The same seeds give the same numbers, so two runs can be compared.
- It is fast enough to iterate. You can test a control idea before touching hardware.

What the simulation does **not** model well: servo friction and backlash, cable forces, dust, the real texture of joint compound, and the fact that every wall is a little different. Section 10 is about those gaps.

## 5. The wall

Real walls are never perfectly flat. The test wall in `sim/wall_geometry.py` carries three common surprises:

- a **4 mm sunken seam** where two boards meet;
- a **1.5° lean** on the right board, so it moves away as the arm travels right;
- a **2.5 mm screw head** standing proud of the left board.

The wall is two panels. The left panel is flat. The right panel is recessed at the seam and tilts away. A small cylinder smooths the seam corner so the pad does not catch on a sharp edge.

One function returns the wall surface:

```python
wall_surface_y(x, z)
```

Give it a position along the wall and a height, and it returns the surface depth. The controller uses this as a map. In the simulation the map is exact. On a real site it would come from a scan, a touch probe, or a registration step. The map does not remove the need for force control; it gives the controller a good starting depth.

The pad still has to follow the surface. That is the job of the force loop.

## 6. Kinematics and IK

A robot arm is a chain of links and joints. Two questions show up everywhere:

- **Forward kinematics (FK):** given joint angles, where is the tool?
- **Inverse kinematics (IK):** given a tool pose, what joint angles put it there?

FK is arithmetic. IK is a puzzle. Many poses can reach the same point, and most of them are awkward: elbows near a limit, wrists twisted, the arm folded against itself. The solver has to pick one.

WallFin's task is demanding. The pad must be at a position **and** aimed along the wall normal. That is six constraints (three position, three orientation) with five joints. The task is exactly determined: in the tight geometry, there is often one solution, not a family of them. A joint-centring term cannot fix a bad working distance; the fold is the only answer.

The practical fix was mechanical: set the rail back so the arm works extended. At 0.40 m, the shoulder and elbow sit at comfortable ranges (about 67 % and 75 % of their travel) instead of 90–100 %. The pad is aimed with a small upward tilt (2 %) to balance servo sag.

The solver starts from a bank of saved poses (`sim/seed_bank.npz`) that are known to reach the wall, then refines. Across the whole sweep, the worst position error is **0.79 mm**. That is well inside the 1 mm acceptance gate.

Position accuracy is necessary but not sufficient. A perfectly positioned pad can still press too hard or lose contact. That is why the next layer is force.

## 7. Force control

This is the heart of the project. If you read only one section, read this one.

Imagine pushing a sanding block against a wall in the dark. You cannot see the seam. You feel the pressure in your hand and adjust. If the wall drops away, you lean in a little. If you hit a screw head, you ease off. You are controlling **force**, not position.

A fixed-depth robot does the opposite. It moves to one depth and holds it. On a flat board that might press 8–25 N. Over a recessed board it floats, touching nothing. The wall changed, but the robot did not.

WallFin uses **admittance control**. The controller measures the force at the pad and adjusts its target depth:

```text
p += −k_a · (f − f_des) · dt + integral
```

- `p` is the commanded depth past the local wall surface.
- `f` is the measured force; `f_des` is 5 N.
- `k_a` converts a force error into a depth change.
- The integral term removes steady offset.
- A 50 ms filter calms the force signal.
- Rate limits and a clamp stop the loop from running away.

The loop has two moods:

- **In the air:** the pad creeps toward the wall at 15 mm/s. Soft, like a hand in the dark.
- **On contact:** the loop holds 5 N and adjusts as the wall changes.

A guard pauses the loop when the depth is out of reach. That prevents "wind-up": the controller pushing harder and harder while nothing happens.

One tick, in plain words:

1. Read the force.
2. Filter it.
3. Compare it with 5 N.
4. Nudge the depth.
5. Convert the depth to a target pose.
6. Solve IK.
7. Command the joints.
8. Repeat 50 times a second.

The result is a pad that stays on the wall through a seam and over a screw head. When the pad edge meets the 4 mm step, the force rises for a moment; the loop eases off and settles again in about half a second.

The baseline in the videos is the same arm, wall and path with this loop switched off. The comparison is the point: it shows what the force feedback buys.

## 8. Coverage and metrics

A demo can look good and still miss half the wall. Coverage keeps the project honest.

The target strip is divided into a grid of **10 mm cells**. The pad's contact is approximated as a circle with a **45 mm** effective radius. A cell counts as covered when it has received at least **20 ms** of contact. Coverage is the share of cells that pass.

Four numbers tell most of the story:

| Metric | What it means | Why it matters |
|---|---|---|
| Contact duty | share of sweep samples with force ≥ 1.5 N | is the pad actually working? |
| Coverage | share of cells with ≥ 20 ms contact | did the tool touch the whole strip? |
| Force error | mean and p95 of `|f − 5 N|` | how steady is the push? |
| Peak force | largest force in the run | could this damage the wall? |

"Steady" error leaves out the bands around the seam and the screw. The pad is wide, so it feels those features early. Including them would mix "the wall is bumpy" with "the controller is sloppy".

The controlled run reaches **97.8 %** duty and **100 %** coverage. The fixed-depth run reaches **61.6 %** and **71.9 %**. Average force error is **1.05 N** versus **2.79 N**.

Numbers can be gamed, so the project uses several at once and checks them in `sim/check_demo.py`. A run that covers the wall by pressing too hard will show up in the peak force and error columns.

## 9. Reproducibility

Science needs a way to check a claim. WallFin does three things:

1. **Fixed seeds.** The runs use fixed random seeds, so the same code produces the same result.
2. **Recorded data.** `--save` writes the time, force, pad position and command to `sim/reports/<tag>.npz`.
3. **Fingerprints.** A SHA-256 of the rounded arrays is stored next to the data as `<tag>.sha256`.

`sim/check_demo.py` checks nine acceptance gates and both fingerprints. The canonical runs are `ctl5` and `base5`:

```text
ctl5 = e208219d5ea24829
base5 = 804c7deb8551ad7d
```

If a change alters the physics, the fingerprint changes and the gate fails. That is the point: a visual tweak should not silently change the results.

The rule for model edits: geometry used only for looks must stay physics-neutral (`density="0"`, `contype="0"`, `conaffinity="0"`). Mass and inertia come from the URDF `<inertial>` blocks. Change a mass-bearing shape and the fingerprints will move. Details are in [DOCS-INDEX.md](DOCS-INDEX.md).

## 10. The real arm

The `hw/` folder carries the same control law to a real arm. It does not import MuJoCo.

The target is a Feetech STS3215 arm: five serial bus servos, a 12 V / 5 A supply, and a half-duplex USB adapter. The code includes a driver, a fake bus for testing, force estimation, the phase runner, and safety checks.

Force on a real arm comes from the servos, not a force sensor. Each STS3215 reports its load. With the arm's pose, the code estimates gravity torque and subtracts it:

```text
force ≈ (measured load − gravity) → pad force
```

That estimate is good enough to start. It has no inertia correction yet, so a quick hit at the seam can show 15–20 N spikes. A wrist force sensor is the next upgrade.

Bring-up is a sequence, not a leap:

1. Scan the bus and confirm all five IDs answer.
2. Set joint signs and offsets by hand.
3. Check the load sign: push the pad toward the wall; the force estimate must go up.
4. Learn the gravity bias with the arm still and off the wall.
5. Move slowly, with a hand on the power switch.
6. Rehearse dry before every real run.

Safety is part of the code: torque off on exit, temperature and voltage stops, a force limit, an E-stop file, and an `--enable` gate that refuses to move without it.

`python hw/host.py --dry --passes 3` runs the full loop with a fake bus and a virtual wall. It reports 100 % coverage and 4.7 N mean force. The dry rehearsal shows **85 %** time on the wall — lower than the sim's 97.8 %, and a useful reminder that the real stack has its own timing and force estimate.

What is missing: a physical build, the rail stepper driver, dust extraction, and a tool changer. The software is ready for a careful first contact, not for a site.

## 11. From demo to a prototype

This is an estimated path, not a commitment; each step can fail cheaply.

| Stage | Work | Time |
|---|---|---|
| 0 | Simulation, force loop, tests, real-arm code, dry rehearsal | done |
| 1 | Buy parts: 5–7 servos, printed SO-101, bus adapter, 12 V supply, rail + stepper, sanding head, camera (~₹50–80k) | 1–2 weeks |
| 2 | Bench bring-up: calibration, load sign, gravity check, slow moves | 1–3 weeks |
| 3 | First wall contact; hold 5 N | 1–2 weeks |
| 4 | Full sweep on a real wall, matching the sim's gates; dust and cables | 4–8 weeks |
| 5 | Product: rail driver, tool changer, dust extraction, calibration routine, safety, pilots, certification | months |

A physical prototype is estimated to take **about 3 months after the parts arrive**. Dust extraction, calibration, safety and pilots come after the first working sweep.

Product work is not just more code:

- **Dust** is a health problem and a machine problem. A sander needs extraction, seals, and a filter that does not clog.
- **Registration** is how the robot knows where it is on the wall. The simulation uses an exact surface map; a real machine needs a scan or a probe.
- **Tool change** turns one arm into a finisher: sander, roller, sprayer, corner tool. Each tool changes the contact model.
- **Safety** means force limits, watchdogs, an E-stop, and a risk assessment. Certification comes later.
- **Pilots** are how you learn what crews actually need. A demo that impresses an investor may not survive a dusty room with bad lighting.

The design target is a prototype roughly **10× cheaper than imported commercial machines**. That is a target to test, not a promise.

## 12. The landscape

The companies below are real and worth studying. The numbers are vendor claims, not independent tests.

| Company | Machine | What it shows |
|---|---|---|
| [Canvas](https://canvas.build) (acquired by JLG, 2026) | 1200CX: 12 ft reach, 1,200 lb, L4/L5 spray and sand, 99.9 % dust capture | drywall finishing can be a commercial product |
| [Okibo](https://okibo.com) | EG7/EG7+: 800 lb, 27 in wide, up to 24 ft, 1,000–1,500 sq ft/h, CE | autonomous painting and sanding at commercial scale |
| [PaintJet](https://paintjet.com) | Bravo: lift-based exterior painting, 14× faster claim, 25 % less paint | robotics can win on large, repetitive exterior work |

What they prove:

- The market is real.
- Dust and labor are the pain.
- Customers will pay for throughput and safety.

What they leave open:

- Small rooms and renovation jobs.
- Low-cost tools that a small crew can own.
- Open control software that others can study and improve.

WallFin is not trying to out-reach Canvas or out-automate Okibo. It is testing a smaller, cheaper, open path to careful contact. If that works, it can serve the jobs the big machines skip.

## 13. Exercises

Each exercise teaches one idea. Use a scratch tag (for example `--tag test1`) so you do not overwrite the canonical runs.

1. **Feel the loop.** Run the controlled demo, then the baseline. Watch the force strip at the bottom. Where does the baseline lose contact? Where does the controlled run recover? *Easy.*
2. **Change the target force.** Set `F_DES = 3.0` in `sim/wall_geometry.py`. Run both modes. What happens to duty and peak force? Restore 5 N afterward. *Easy.*
3. **Make the wall harder.** Increase `TILT` to 3°. Run the controlled mode to a scratch tag. Which metric moves first: duty, coverage, or peak force? *Medium.*
4. **Add a second screw.** Copy the blob geometry in `build_model.py` and move it. Does the loop recover? *Medium.*
5. **Measure by hand.** Load `sim/reports/ctl5.npz` in Python and compute contact duty yourself: the share of sweep samples with force ≥ 1.5 N. Compare with the table. *Medium.*
6. **Rehearse the arm.** Run `python hw/host.py --dry --passes 3`. Read the JSON log. Which phase has the lowest duty? *Easy.*
7. **Design the rail.** Sketch the missing stepper axis: motor, belt or screw, carriage, limits, cable path. What would you test first? *Hard, no code.*

The exercises are also a good way to learn the codebase. Start with `sim/control.py`; it is short and it is the heart.

## 14. Glossary and next steps

**Glossary**

- **Admittance control** — adjusting target position in response to measured force.
- **Contact duty** — share of the sweep with the pad touching (force ≥ 1.5 N).
- **Coverage** — share of grid cells that received enough contact time.
- **Digital twin** — a simulation close enough to test real decisions.
- **DOF** — degrees of freedom; independent joint motions.
- **End effector** — the tool at the end of the arm.
- **FK / IK** — joint angles to pose; pose to joint angles.
- **MJCF** — MuJoCo's XML model format.
- **MuJoCo** — a physics engine for contact-rich robotics.
- **SO-101** — a small, 3D-printable arm design by TheRobotStudio.
- **STS3215** — a serial bus servo from Feetech.
- **Sim-to-real** — moving a controller from simulation to hardware.

**Where to go next**

- Read `sim/control.py` — the force loop, about 40 lines.
- Read `sim/wall_geometry.py` — the scene, in constants.
- Read `sim/robot.py` — IK and the pose bank.
- Read `hw/host.py` — the same idea on hardware, with safety.
- Run `python sim/check_demo.py` and change one number.
- Read [ROADMAP.md](../ROADMAP.md) for the estimated path.
- Read [docs/pitch.md](pitch.md) for the one-page version.
- Read [docs/writing.md](writing.md) for how we write.

Further reading:

- [MuJoCo documentation](https://mujoco.readthedocs.io)
- [SO-ARM100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100)
- [MOSS × Jev](https://github.com/metrox-eth/moss-jev) — the IK build pattern this project adapted.

If you build something with this, or find a wall that breaks the loop, tell me. That is how the next version gets better.

— Salik Shah · [@salikshah](https://github.com/salikshah) · [salik.shah@gmail.com](mailto:salik.shah@gmail.com)
