# Research notes: gait-aware decentralized navigation for quadruped teams

Living document. Last updated 2026-10-08.

## Working research question
Can a decentralized reactive controller that knows each robot's locomotion state
(gait phase φ, contact schedule, speed and turn rate) keep a team of quadrupeds safe
and in formation as team size grows, without global communication or centralized
optimization?

## Proposed controller (draft)
Per robot, every control tick:
1. **Preferred velocity** `v_pref` = goal attraction + formation springs (with elastic
   spacing `d_ij* = d0 + Δd(d_obs, TTC, ρ)`) + APF obstacle repulsion.
2. **Safety + feasibility projection**:
   `v* = argmin ||v − v_pref||²  s.t.  v ∈ ∩_j H_ij (ORCA half-planes)  ∩  V_i(q_i)`
   where `V_i(q_i)` is the *locomotion-feasible* velocity set (identified from the
   low-level controller, see pilot). Small LP/QP per robot; inherits ORCA's collision
   avoidance guarantee whenever it is feasible.
3. **Infeasible / stalled** (`||v|| < v_min` far from goal for `T_dead`) → deadlock
   recovery by perturbing formation spacing / weights.
4. `v*` → `Go1BezierController.set_command()`.

Note: blending `α v_APF + (1−α) v_VO + v_form` *after* safety can put the result inside a
velocity obstacle; safety must be the last step (the projection), not one term of a sum.

## Positioning vs. prior work (status of each claim)
| Work | Relevance | Verified |
|---|---|---|
| Role-adaptive collaborative formation planning for quadruped teams (arXiv 2602.18260; Mechatronics 2026) | Closest. Go1 teams of 3–4, centralized/global info | yes (arXiv) |
| Acceleration-Velocity Obstacles, AVO2 (github.com/snape/AVO2) | Kinodynamic VO: must be a baseline | yes |
| Robot navigation with acceleration obstacles (arXiv 2504.13637); NLVO→NAO (arXiv 2506.06255) | Recent kinodynamic VO: baselines / related | yes |
| NH-ORCA | Non-holonomic ORCA: related | known, not re-checked |
| Distributed MPC + gait sync for multiple quadrupeds (ICRA 2023, IEEE 10161260) | Real competitor (distributed, not centralized) | **to verify** |
| Safe distributed learning-enhanced predictive control for multiple quadrupeds (arXiv 2503.05836) | Competitor (DMPC + CBF + learning) | **to verify** |
| DEFORM (adaptive formation, DMPC + PF) | Related | **to verify** |

Gap claim (weak evidence only, needs a proper search): no prior work found that puts a
gait-phase / locomotion-state-dependent feasible set inside reciprocal velocity
obstacles for *teams* of legged robots. Single-robot legged collision avoidance exists
(Agile But Safe, arXiv 2401.17583; Omni-Perception, arXiv 2505.19214).

## Risks a reviewer will raise
1. "V_i(φ, q) is just acceleration limits" → must show dependence AVO-style limits can't capture.
2. Gait phase may wash out at swarm rate (10–20 Hz) vs. stride ~0.36 s → **pilot first**.
3. "Feasible set is an artifact of a weak Bezier gait" → repeat key results with a learned
   Go1 policy (MuJoCo Playground) as a second low-level controller.
4. Too many components → core = locomotion-aware feasible set in reciprocal VO + scaling
   study; elastic formation and deadlock recovery are supporting (with ablations).
5. Baselines: ORCA, AVO, APF, APF+VO, distributed MPC (not only centralized MPC).

## Pilot experiment (next)
From steady walking, apply a step change Δv (direction × magnitude grid) at 10–20 gait
phases φ. Measure peak tilt, tracking error/settling time, foot slip, falls.
Repeat with and without the command slew limiter.
Decision: strong φ-dependence → φ is a headline variable; otherwise keep the
locomotion state q only ("locomotion capability map").

## Gait stability (scripts/floquet.py)
Stroboscopic Poincare map (sample once per gait period, x/y/yaw removed), Jacobian by
central differences, Floquet multipliers = eigenvalues. max|lambda| < 1 -> locally stable.

| command | open loop | Raibert only | PI + Raibert (default) |
|---|---|---|---|
| stand-in-place | 0.78 | 0.24 | 0.47 |
| fwd 0.3 | 0.74 | 0.26 | 0.42 |
| fwd 0.6 | 0.73 | 0.35 | 0.51 |
| back -0.3 | 0.87* | 0.35 | 0.59 |
| lateral 0.25 | 0.78 | 0.41 | 0.49 |

All locally stable. The open-loop gait is stable but slow to reject perturbations
(15-30 strides for 99% decay) and has a speed bias; Raibert foot placement speeds
recovery ~3x; the PI loop removes the bias but adds a slower integrator mode.
(*) open-loop backward orbit not fully converged (drift 1e-2): treat as approximate.
Local multipliers say nothing about large disturbances -> complement with basin /
push-recovery tests and fall statistics (stress test, pilot experiment).

## Low-level controller status (go1_bezier)
- Velocity tracking within ~0.02 of command (fwd 0.3/0.6, back −0.3, lateral 0.25, yaw 0.8).
- Random abrupt commands, 4 min: 0 falls with slew limiter (max tilt 9–22°);
  without limiter, tilt up to ~35° and occasional falls.
- Stand height (trunk) 0.12–0.38 m statically; walks reliably at hip-foot 0.16–0.34 m
  (0.14: slow speeds only; ≥0.36: fails).
- Per-leg check: sit on hind legs and wave FR/FL, sit on front legs and wave RL/RR;
  waving foot lifts > 0.2 m while the other three stay planted.
- Scaling: 1 robot ≈ 7× real time, 16 ≈ 1×, 64 ≈ 0.25× (Linux container; Python-bound).
