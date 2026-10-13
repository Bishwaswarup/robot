# Gait-Aware Command Timing for Decentralized Quadruped Swarms

**Unitree Go1 · MuJoCo · ORCA · Bezier trot**

[![Preprint DOI](https://img.shields.io/badge/preprint-10.21203%2Frs.3.rs--11332226%2Fv1-blue)](https://doi.org/10.21203/rs.3.rs-11332226/v1)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue)
![MuJoCo 3.2+](https://img.shields.io/badge/MuJoCo-3.2%2B-orange)

<p align="center">
  <img src="docs/media/swarm_swap16.gif" width="49%" alt="16 Go1 robots swapping positions through the centre">
  <img src="docs/media/swarm_gate.gif" width="49%" alt="Six-robot formation passing a narrow gate">
</p>

> **Paper:** Bishwaswarup Nayak, *Gait-Aware Command Timing for Decentralized Quadruped Swarms*.
> Preprint, Research Square, 2026, under review.
> [doi:10.21203/rs.3.rs-11332226/v1](https://doi.org/10.21203/rs.3.rs-11332226/v1) ·
> LaTeX source in [`paper/`](paper/) · [Supplementary video](docs/media/online_resource_1.mp4) ·
> [How to cite](#citation)

Multi-robot collision avoidance usually treats each robot as a point that can change its velocity at any
moment. A trotting quadruped cannot: how well it absorbs a new velocity command depends on **where in the
gait cycle** the change arrives. This repository measures that dependence for a Bezier-curve trot
controller on a simulated Unitree Go1 and builds a decentralized swarm layer around it:

- the swarm layer solves **ORCA one gait period ahead** (lookahead 0.36 s);
- a **commander holds** each non-urgent velocity change for at most half a gait period (0.18 s) and
  releases it at the **gait phase** a measured capability map predicts is least destabilizing;
- a **gait-aware slew limiter** shapes the change;
- urgent conflicts (time to collision < 0.8 s, or < 1.5 s to a static obstacle) bypass the hold.

Every robot runs the same local rules; there is no central planner. All results are in simulation.

## Results at a glance

Paired by seed, exact sign-flip permutation tests, Holm-corrected within each comparison; 1,110 runs in the
final configuration.

**A. Full system vs. reactive baseline** (21 primary tests, 15 significant, 13 in favour of the full system)

| | |
|---|---|
| Urgent avoidance events, crowded or obstructed scenarios (6, 9, 16, 32 robots) | **−22 to −39 %** |
| 99th-percentile body tilt, same scenarios | **−19 to −24 %** |
| Closest robot–robot distance in formations | **+4 to +8 %** (2–5 cm) |
| Falls | **0** in 230 full-system runs (baseline: 2 in 170) |
| Robot–robot / robot–obstacle contact onsets | 16 / 1 (baseline: 28 / 4) — fewer, **not eliminated** |
| 53 m obstacle course, team of 6 | p99 tilt 7.95° → 6.68°, max tilt 18.3° → 13.7° |

**B and C. What does the work.** Holding a change and planning ahead, not the choice of phase, produces the
reduction in urgent events: a control that holds for the same time but releases after a **random wait**
reduces urgent events nearly as much as the full system (−27 % vs −30 % in the pillar field, −30 % vs −34 %
at the gate). Choosing the
phase the map predicts is safest adds a **smaller, consistent reduction in tilt** (−5.9 % p99 tilt against
the random wait, pooled over 80 seed pairs; best phase < random wait < worst phase in every scenario).

**Map in context.** The capability map predicts isolated command steps well (held-out R² 0.98) but not the
rapid command sequences of a swarm (R² −0.99 on 488 replayed swarm events). Its phase ranking transfers only
in part: its choice saves 0.21° of tilt against a random release on average, up to ~1° when the previous
command change was at least a gait period earlier.

**Where it does not help.** With four robots the method increases urgent events (+109 %); in the 8-robot swap
it does not change them significantly. Collision avoidance is validated empirically, not guaranteed.

Full tables: [`results/paper/paper_tables.md`](results/paper/paper_tables.md).

## Gallery

| Single robot: Bezier trot | Obstacle course, team of 6 |
|:---:|:---:|
| <img src="docs/media/go1_trot.gif" width="100%" alt="Go1 trotting: forward, arc, sidestep"> | <img src="docs/media/course_team6.gif" width="100%" alt="Six robots on the obstacle course"> |

**Swarm paths** (full system, seed 0; one colour per robot, dots trace the trunk):

| 16-robot antipodal swap | 6-robot formation through a 1.9 m gate |
|:---:|:---:|
| <img src="docs/media/swarm_swap16_paths.png" width="100%" alt="Paths of 16 robots swapping"> | <img src="docs/media/swarm_gate_paths.png" width="100%" alt="Paths of 6 robots through a gate"> |

<p align="center">
  <img src="docs/media/course_team6_paths.png" width="100%" alt="Paths of the team of 6 over the whole obstacle course"><br>
  <em>Obstacle course (53 m): pole slalom, gate, chicane, two timed moving obstacles, pillar field.</em>
</p>

<p align="center">
  <img src="docs/media/go1_closeup.png" width="60%" alt="Unitree Go1 model in MuJoCo">
</p>

GIFs and stills are rendered offscreen by [`scripts/make_media.py`](scripts/make_media.py) (about 15 min on a laptop).
To add your own (e.g. hardware photos or videos), drop the file in [`docs/media/`](docs/media/) and link it here:
`<img src="docs/media/your_file.gif" width="100%">`.

## Supplementary video

[**Online Resource 1**](docs/media/online_resource_1.mp4) (MP4, 13 MB): reactive baseline and full system side
by side in the 16-robot swap, the 9-robot formation through pillars, the 6-robot formation through the gate
and the 6-robot team on the obstacle course (seed 0). Made by [`scripts/make_video.py`](scripts/make_video.py).

## Paper figures

<details open>
<summary><b>Fig. 1 – The gait phase matters</b></summary>

<img src="paper/figures/fig2_phase.png" alt="Tilt response vs gait phase">

Walking at 0.4 m/s: (a) peak body tilt after a 0.6 m/s command step, by direction of the change and the gait
phase at which it is released; (b) largest survivable push, relative to the best phase for that direction.
</details>

<details>
<summary><b>Fig. 2 – Control stack</b></summary>

Drawn in TikZ inside the paper ([`paper/main.tex`](paper/main.tex)): predictive ORCA → commander (hold ≤ 0.18 s,
release at the best phase) → gait-aware slew limiter → Bezier trot controller, with the urgent bypass.
</details>

<details>
<summary><b>Fig. 3 – Primary results (full system vs. baseline)</b></summary>

<img src="paper/figures/fig3_primary.png" alt="Primary results">
</details>

<details>
<summary><b>Fig. 4 – Ablation: which component does the work</b></summary>

<img src="paper/figures/fig4_ablation.png" alt="Ablation">

Prediction alone does not help; adding the commander (hold and phase choice) produces most of the gain.
Fig. 7 separates the hold from the phase choice.
</details>

<details>
<summary><b>Fig. 5 – Lookahead</b></summary>

<img src="paper/figures/fig5_lookahead.png" alt="Lookahead sweep">

Solving ORCA one gait period ahead lowers the share of urgent commands from 56 % to 22–43 %.
</details>

<details>
<summary><b>Fig. 6 – Obstacle course paths</b></summary>

<img src="paper/figures/fig6_course.png" alt="Course paths, baseline vs full">

Pole slalom, gate, chicane, two timed moving obstacles and a pillar field (53 m), team of 6.
</details>

<details>
<summary><b>Fig. 7 – Timing controls: the phase or the wait?</b></summary>

<img src="paper/figures/fig7_controls.png" alt="Worst phase vs random wait vs best phase">

All three are the full system with the same hold; they differ only in when the held change is released.
Urgent events barely depend on it; p99 tilt is lowest at the map's best phase in every scenario.
</details>

<details>
<summary><b>Fig. 8 – The capability map inside the swarm</b></summary>

<img src="paper/figures/fig8_map_context.png" alt="Map predictions vs measurement in context">

(a) Absolute predictions miss in context (bias −1.8°). (b) The map's phase choice still helps, most where
events resemble its calibration.
</details>

## Setup

```bash
git clone https://github.com/Bishwaswarup/robot.git && cd robot
git clone https://github.com/google-deepmind/mujoco_menagerie.git   # Go1 model (not included)
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
pytest                                                              # unit + physics tests
```

The model is looked up at `./mujoco_menagerie`; override with `MENAGERIE_PATH=/path/to/mujoco_menagerie`.
On macOS the interactive viewer needs `mjpython` (installed with `mujoco`); elsewhere use `python`.

## Quick start

```bash
mjpython examples/walk_demo.py                               # one robot, scripted walk
mjpython examples/swarm_demo.py --scenario formation6_gate   # swarm scenario in the viewer
mjpython examples/course_demo.py --team 6                    # obstacle course
python scripts/make_media.py                                 # README GIFs/PNGs (headless Linux: MUJOCO_GL=osmesa)
```

```python
import mujoco
from go1_bezier import load_team, Go1BezierController

model, data = load_team([(0, 0, 0), (1.5, 0, 0)])            # (x, y, yaw) per robot
robots = [Go1BezierController(model, data, prefix=f"go1_{i}/") for i in range(2)]
robots[0].set_command(0.4, 0.0, 0.3)    # vx, vy (m/s), wz (rad/s), body frame
for _ in range(1000):
    for r in robots:
        r.step()
    mujoco.mj_step(model, data)
print(robots[0].phase)                  # gait phase in [0, 1), used by the swarm layer
```

## Reproduce the paper

| Step | Command | Output |
|---|---|---|
| 1. Phase dependence (pilot) | `python scripts/pilot_phase.py` | `results/pilot*` |
| 2. Capability map | `python scripts/capability_map.py` | `data/capability_map_bezier.npz` |
| 3. Push envelope | `python scripts/push_envelope.py` | `data/push_envelope_bezier.npz` |
| 4. Gait stability | `python scripts/floquet.py` | Floquet multipliers |
| 5. Swarm experiments | `python scripts/paper_runs.py --robust` | `results/paper/{main,lookahead,scaling,obstacles}_robust/` |
| 6. Obstacle course | `python scripts/course_eval.py --seeds 20 --speed 0.45 --robust` | `results/course/` |
| 7. Tables | `python scripts/paper_tables.py` | `results/paper/paper_tables.md`, `primary_tests.csv` |
| 8. Figures | `python scripts/paper_figures.py` | `paper/figures/*.pdf, *.png` |
| 9. Timing controls (random / worst phase) | `python scripts/paper_runs.py --suites timing --robust` then `python scripts/timing_controls.py` | `results/paper/timing_robust/` |
| 10. Map validation in context | `python scripts/map_validation.py` | `results/map_validation/` |
| 11. Media and video | `python scripts/make_media.py`, `python scripts/make_video.py` | `docs/media/` |

The maps in `data/` and the raw results in `results/` are included, so steps 7 and 8 run without re-simulating.
Step 5 takes several hours on one machine (`--quick` ≈ 15 min). Swarm configurations are written
`layer:strategy[:q]`; the full system is `predictive:phase+gaslc_equal:q`, the baseline `reactive:limiter`.

<details>
<summary>All scripts</summary>

| Command | What it does |
|---|---|
| `python examples/record_video.py` | Render the walk to MP4 without a window |
| `python scripts/test_walk.py` | Command vs. measured velocity table |
| `python scripts/stress_test.py` | Random abrupt commands, counts falls |
| `python scripts/height_range.py` | Min/max stand height and walkable heights |
| `python scripts/floquet.py` | Gait stability: Poincaré map, Floquet multipliers |
| `python scripts/pilot_phase.py` | Does the disturbance response depend on gait phase? (`--quick` ≈ 1 min) |
| `python scripts/capability_map.py` | Build and validate the capability map; phase-aware timing analysis |
| `python scripts/push_envelope.py` | Push-capability envelope J_max(direction, phase, speed) by bisection |
| `python scripts/commander_eval.py` | Closed-loop test of phase-aware timing vs. immediate release / slew limiter |
| `python scripts/swarm_eval.py` | Swarm scenarios × configurations (`--robust` for the final safety stack) |
| `python scripts/swarm_stats.py <csv>` | Paired statistics (bootstrap CI, permutation p) vs. baseline |
| `python scripts/course_eval.py` | Obstacle course, team of 1 and 6, section times |
| `python scripts/paper_runs.py` | All paper swarm experiments (`--suites`, `--quick`, `--robust`) |
| `python scripts/paper_tables.py` | Holm-corrected tables from the final runs |
| `python scripts/paper_figures.py` | Paper figures |
| `python scripts/timing_controls.py` | Full system vs random-wait and worst-phase controls |
| `python scripts/map_validation.py` | Capability map scored on replayed swarm events (`--quick` ≈ 5 min) |
| `python scripts/make_media.py` | README GIFs and stills (`--only trot,swap16,gate,course`) |
| `python scripts/make_video.py` | Supplementary video (`--only gate` for one segment) |
| `mjpython examples/trick_demo.py` | Height range, lie down/stand up, sit and wave |

</details>

## Repository layout

```
go1_bezier/        controller and swarm package
  kinematics.py      Go1 leg IK/FK
  gait.py            phase-clocked Bezier trot
  controller.py      velocity interface: slew limit, PI velocity loop, Raibert foot placement
  capability.py      CapabilityMap (tilt/fall vs. phase) and PushEnvelope
  commander.py       PhaseAwareCommander: holds large changes, releases at the best phase
                     (random-wait and worst-phase rules for the controls)
  gasl.py            gait-aware slew limiter + strategy names
  orca.py            ORCA (port of RVO2's linear programs) with static disc obstacles
  swarm.py           SwarmAgent: predictive ORCA, keep-right, turn gate, local-minimum escape,
                     FormationPlan, queueing responsibility
  scenarios.py       swap4/8/16/32, formation9/16 through pillars, formation6 through a gate
  course.py          53 m obstacle course, PathPlan, timed moving obstacles
  poses.py, tricks.py  static body poses, sit/wave routine
data/              capability map and push envelope (.npz)
examples/          viewer demos, video recording
scripts/           experiments, statistics, tables, figures, media
results/           raw CSVs of the paper runs
paper/             LaTeX source (Springer Nature sn-jnl: main.tex), figures, compiled PDF
docs/              research notes, docs/media (README GIFs and images)
tests/             pytest
```

## Low-level controller

- Tracks within ~0.02 of the command: forward 0.3/0.6 m/s, backward −0.3 m/s, lateral 0.25 m/s, yaw 0.8 rad/s.
- Random abrupt commands (vx −0.4…0.8, vy ±0.3, wz ±1.0, changed every 0.5–2 s): 0 falls in 4 min.
- Trunk height 0.12–0.38 m standing; walks at hip-to-foot 0.16–0.34 m.
- Speed on one CPU core: 1 robot ≈ 7× real time, 16 ≈ 1×, 64 ≈ 0.25×.

Defaults: period 0.36 s, duty 0.5, clearance 0.07 m, stand 0.27 m, foot friction 0.9, k_i 1.5, k_raibert 0.1,
slew limit (1.5 m/s², 1.0 m/s², 3 rad/s²).

## Limitations

- Simulation only (MuJoCo, flat ground, perfect state estimation, no sensing noise or delay).
- One gait (trot) and one controller; the capability map must be re-measured for another controller.
- The map is calibrated on isolated steps; inside the swarm its phase ranking holds only in part.
- Collision avoidance is validated empirically, not guaranteed: contacts are reduced, not eliminated.
- In sparse traffic the hold costs time and urgent events. Hardware validation is future work.

## Citation

If you use this code, please cite the preprint (GitHub's **"Cite this repository"** button reads
[`CITATION.cff`](CITATION.cff)):

```bibtex
@misc{nayak2026gaitaware,
  author    = {Nayak, Bishwaswarup},
  title     = {Gait-Aware Command Timing for Decentralized Quadruped Swarms},
  year      = {2026},
  publisher = {Research Square},
  note      = {Preprint},
  doi       = {10.21203/rs.3.rs-11332226/v1},
  url       = {https://doi.org/10.21203/rs.3.rs-11332226/v1}
}
```

## License and credits

MIT, see [LICENSE](LICENSE). Third-party components ([THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)):
Bezier swing curve from spot_mini_mini (MIT); Go1 model from MuJoCo Menagerie (BSD-3, Unitree Robotics).
