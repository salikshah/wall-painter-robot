# WallFin — one-page summary

**What it is.** A robot that helps finish walls. It sands and paints with
real tools. It presses a pad or a roller on the wall and keeps a steady push
(5 newtons — a water bottle's weight) even over seams, screw heads and
leaning boards. A rail carries the arm side to side.

**Why it matters.** Wall finishing is one of the last dusty, tiring jobs
left to human hands. Commercial systems (Canvas, Okibo) are large,
capital-scale machines built for big sites. Small repainting and renovation
work — the kind that fills Indian towns and cities — has little built for
it. Inkjet wall printers add color; the dusty prep stage still needs people.
We think a gentle robot should share that work.

**What works today (simulation, repeatable).**

| metric | WallFin force loop | fixed depth |
|---|---|---|
| time on the wall | **97.8 %** | 61.6 % |
| wall covered | **100 %** | 71.9 % |
| average force error | **±1.1 N** (steady ±1.1 N, p95 ±3.0 N) | ±2.8 N |
| biggest push (worst spot) | **15.0 N** | 13.8 N |

- Every number comes from a fixed MuJoCo sim with an acceptance test suite
  (`sim/check_demo.py`). Videos and charts are in `docs/`.
- The test wall carries the surprises real rooms do: 4 mm seam, 1.5° lean,
  2.5 mm screw head.

**Software status: ready for a real arm.** The control brain is more than a
sim toy:

- Real-robot code (`hw/`): math checked against the sim to **0.001 µm**,
  gravity correction within 0.04 N·m, 50 Hz control loop, STS3215 servo
  driver, force from motor current, and safety that behaves like a careful
  lab partner (torque off, temp / voltage / force stops, E-stop hook).
- A full dry rehearsal passes with no hardware. 100 % coverage, 5 N loop.

**Prototype estimate.**

1. A small prototype: 6-7 servos, 3D-printed SO-101 parts, rail +
   stepper, sanding head, USB camera — **≈ ₹50–80k total**.
2. Bench bring-up: 1–3 weeks (calibrate, first moves).
3. First wall contact, 5 N loop holding: +1–2 weeks.
4. Full sweep passing the sim's gates on a real wall: +4–8 weeks.

A physical prototype is estimated to take **about 3 months after the parts
arrive**. The hard control loop already exists; what remains is assembly and
tuning, then dust extraction, calibration, safety and pilots.

**The edge.** One arm, many tools: sander → roller → spray → corner tools.
Force control is the difference; it is what lets a robot do careful physical
work. The roadmap adds a tool changer and a mural mode (paint a picture with
a real roller). Build target: **10× under imported machines**.

**Assets in this repo.**

- `docs/wallfin_final_film.mp4` — the branded 1:48 launch film (1080p50,
  with sound). `docs/wallfin_main_cover.png` is the poster.
- `docs/guide.md` — a 14-section builder's guide, from zero to the wall.
- `sim/` — simulation, control law, acceptance tests.
- `hw/` — real-robot code + setup guide.
- `ROADMAP.md` — the estimated path, step by step.

**What we're still learning.** A sim gets about 70 % of the way. Real servo
friction, load-sign setup and dust each ask for patience — the estimate
above leaves room for them. Force comes from motor current (v1); a wrist
force sensor is the next upgrade. If any of this sparks a thought, we would
love to hear it.
