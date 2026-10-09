# Gait-aware quadruped swarm (Unitree Go1, MuJoCo)

Research code for decentralized, **gait-aware** navigation of quadruped teams.
Current state: the low-level layer, a Bezier-trot velocity controller for the
Unitree Go1 with an explicit gait phase, plus pose control and test routines.
The swarm layer is next (see [`docs/research_notes.md`](docs/research_notes.md)).

## Setup
```bash
git clone <this repo> robot && cd robot
git clone https://github.com/google-deepmind/mujoco_menagerie.git   # robot model (not in this repo)
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```
The model is looked up at `./mujoco_menagerie`; override with `MENAGERIE_PATH=/path/to/mujoco_menagerie`.

## Run
| Command | What it does |
|---|---|
| `mjpython examples/walk_demo.py` | Viewer: scripted walking (fwd, arc, sidestep, turn, backward). `--n 9` for a 3×3 team |
| `mjpython examples/swarm_demo.py --scenario formation6_gate` | Viewer: swarm scenario (ORCA + formation + commander) |
| `mjpython examples/course_demo.py --team 6` | Viewer: obstacle course (pole slalom, gate, chicane, two timed moving obstacles, pillars) for 1 robot or a team |
| `mjpython examples/trick_demo.py` | Viewer: height range, lie down/stand up, sit + wave each paw, crouch/tall walking |
| `python examples/record_video.py` | Render the walk to MP4 without a window |
| `python scripts/test_walk.py` | Command vs. measured velocity table |
| `python scripts/stress_test.py` | Random abrupt commands, counts falls |
| `python scripts/height_range.py` | Min/max stand height and walkable heights |
| `python scripts/floquet.py` | Gait stability: Poincaré map, Floquet multipliers |
| `python scripts/pilot_phase.py` | Pilot: does disturbance response depend on gait phase? (`--quick` for a 1-min test) |
| `python scripts/capability_map.py` | Build + validate the capability map from the pilot CSV; phase-aware timing analysis |
| `python scripts/push_envelope.py` | Measure the push-capability envelope J_max(direction, phase, speed) by bisection (`--quick` ≈ 2 min) |
| `python scripts/swarm_eval.py` | Swarm test: 4 scenarios × configs `layer:strategy[:q]` (reactive/predictive, commander strategy, bottleneck queueing) |
| `python scripts/course_eval.py --seeds 20 --speed 0.45` | Obstacle course: team 1 and 6, reactive vs. predictive, section times (`--quick`) |
| `python scripts/paper_runs.py` | All paper swarm experiments: main ablation (20 seeds), lookahead sweep, scaling to 16/32 robots + paired stats (`--quick` ≈ 15 min); `--suites obstacles` re-runs the obstacle scenarios with static-obstacle safety |
| `python scripts/swarm_stats.py <csv>` | Paired statistics (bootstrap CI, permutation p) vs. baseline |
| `python scripts/commander_eval.py` | Closed-loop test of phase-aware command timing vs. immediate release / slew limiter (`--quick` ≈ 1 min) |
| `pytest` | Unit + physics tests |

On macOS the viewer needs `mjpython` (installed with `mujoco`); on Linux/Windows use `python`.

## Using the controller
```python
from go1_bezier import load_team, Go1BezierController
model, data = load_team([(0, 0, 0), (1.5, 0, 0)])           # (x, y, yaw) per robot
robots = [Go1BezierController(model, data, prefix=f"go1_{i}/") for i in range(2)]
robots[0].set_command(0.4, 0.0, 0.3)   # vx, vy (m/s), wz (rad/s), body frame
robots[0].set_height(0.22)             # hip-to-foot, walkable 0.16-0.34 m
# each physics step:
for r in robots: r.step()
mujoco.mj_step(model, data)
robots[0].phase, robots[0].contact_schedule   # gait state for the swarm layer
```

## Layout
```
go1_bezier/      controller package
  kinematics.py    Go1 leg IK/FK
  gait.py          phase-clocked Bezier trot
  controller.py    velocity interface: slew limit, PI velocity loop, Raibert foot placement
  poses.py         static body-pose control (feet planted, trunk moved, legs lifted/waved)
  tricks.py        the sit/wave/height routine
  sim_utils.py     load one robot or a team (shared meshes), foot friction
  capability.py    CapabilityMap (tilt/fall/recovery/slip vs. phase) and PushEnvelope (J_max)
  commander.py     PhaseAwareCommander: swarm-layer -> robot, releases big changes at the best phase
  gasl.py          gait-aware slew limiter (phase/direction-shaped acceleration limit) + strategy names
  orca.py          ORCA (port of RVO2's linear programs), static disc obstacles
  swarm.py         SwarmAgent (ORCA + keep-right + heading control, predictive lookahead), FormationPlan,
                   elastic_scale, queue_responsibility (bottleneck priority)
  scenarios.py     swap4/8/16/32 (antipodal), formation9/16 through pillars, formation6 through a gate
  course.py        obstacle course + PathPlan (curvilinear formation along waypoints, single-file
                   collapse where narrow, path-following preference) + moving obstacles
data/            capability_map_bezier.npz, push_envelope_bezier.npz (built from experiments)
examples/        viewer demos, video recording
scripts/         headless evaluation scripts
tests/           pytest
docs/            research notes
```

## Low-level controller: current numbers
- Tracking within ~0.02 of command: forward 0.3/0.6 m/s, backward −0.3, lateral 0.25, yaw 0.8 rad/s.
- Random abrupt commands (vx −0.4…0.8, vy ±0.3, wz ±1.0, changed every 0.5–2 s): 0 falls in 4 min.
- Trunk height 0.12–0.38 m standing; walks at hip-to-foot 0.16–0.34 m.
- 1 robot ≈ 7× real time, 16 ≈ 1×, 64 ≈ 0.25× (single CPU core).

Defaults: period 0.36 s, duty 0.5, clearance 0.07 m, stand 0.27 m, foot friction 0.9,
k_i 1.5, k_raibert 0.1, slew limit (1.5 m/s², 1.0 m/s², 3 rad/s²).

## Credits
See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md): Bezier swing curve from
spot_mini_mini (MIT); Go1 model from MuJoCo Menagerie (BSD-3, Unitree Robotics).
