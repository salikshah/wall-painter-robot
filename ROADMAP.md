# WallFin — Roadmap

> **Paused 19 October 2026.** Public v1.0.0 is live. Restart from
> [HANDOFF.md](HANDOFF.md).

What this is: a proof-of-concept robot that helps finish walls. It exists in
simulation; the software is ready for a real arm.

Today: the robot sands at 5 N in MuJoCo. The test wall is 610 mm wide,
with two boards, a seam and a screw bump. The arm sweeps a 280 mm strip.
The force loop keeps the pad on the wall
far longer than fixed-depth control can (`sim/check_demo.py` measures it).
Runs repeat exactly. Videos and charts live in `docs/`.

The arm looks like a real SO-101. Bone-white shells. Gunmetal joints. Studio
light with real shadows. A force bar on screen. Paint shows up as it sands.

The real-arm code (`hw/`) is written and rehearsed with no hardware. The
math matches the sim (0.001 µm pose, 0.04 N·m gravity). The loop runs at
50 Hz. There is a driver, force from servo loads, safety guards, and a
setup guide.

## P0 — the story so far (done)

- [x] Robot looks right. Links, lights, backdrop, on-screen force bar.
- [x] Wall paints itself as it gets sanded. Easy to see in the video.
- [x] White title and end cards on both demo videos, plus a branded 1:48
      launch film. 1080p50 master.
- [x] Branded film cover and CTA.
- [x] README and a 14-section builder's guide, written for a first-time
      reader.

## P0.5 — from sim to a real arm (software done, hardware path)

Software (`hw/`, free, no hardware needed):

- [x] Arm math in plain numpy. Checked against MuJoCo to 0.001 µm.
- [x] STS3215 serial driver + a fake bus. Scan: `python feetech.py COM3`.
- [x] Force from servo loads + gravity correction (0.04 N·m).
- [x] Control loop: same phases and same force loop as the sim. 50 Hz.
      Safety: torque off on exit, temp/volt/force stops, E-stop file,
      --enable gate.
- [x] Dry run passes: 100 % coverage, 4.7 N vs 5 N. Same numbers every time.
- [x] Setup guide + calibration. Tools: `--hold`, `--calib`, `robot.json`.

Hardware (prototype build — about ₹50–80k):

- [ ] Buy: 5-7 servos, printed SO-101 parts, bus adapter, 12 V power supply,
      rail + stepper, sanding motor, current sensor, webcam.
- [ ] Bench bring-up (1–3 weeks). Meet the servos → set signs →
      check the load sign → gravity check → first slow moves.
- [ ] First wall contact. Hold 5 N (+1–2 weeks).
- [ ] Full sweep passing the sim's gates on a real wall (+4–8 weeks with
      assembly and shipping). Dust and cables sorted.
- [ ] Stepper rail driver. Replaces `--no-base`.

A physical prototype is estimated to take about **3 months after the parts
arrive**. The parts that ask for patience: load-sign setup, cable routing and
dust. Dust extraction, calibration, safety and pilots come after the first
working sweep.

## P1 — more skills for the demo (2-3 weeks)

- [ ] Tool changer: sander → roller → spray gun. New visuals + new pad
      physics.
- [ ] Mural mode: load a picture, turn it into strokes, paint it with the
      roller. Real touching, real paths.
- [ ] Realism pass: sensor noise, uneven wall friction, servo backlash.
- [ ] Second head: corner and edge sander.
- [ ] Export notes to move to ROS2 later (URDF/ONNX).

## P2 — product, brand, launch

- [ ] Name + logo + colors. Keep the bone-white and gunmetal, with the amber
      accent.
- [ ] Landing page + cut-down launch film (the branded 1:48 film exists).
- [ ] Cost sheet for an India 1.5 m rail build (SO-101 clone, V-slot, power
      supply, tool heads). Goal: 10× under imported wall machines.
- [ ] Possible early users: drywall crews (dust is a health question),
      homestay and cafe murals, repaint jobs.

## Why this is different

Wall printers add ink. They do not touch the wall, so prep and sand stay
manual. WallFin touches the wall: sand, prime, paint, roller murals. One
arm, swappable tools. Force control is the edge. The sim holds the hard
control loop; the hardware path is assembly and tuning, not new science.
