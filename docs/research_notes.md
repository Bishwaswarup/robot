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

Implemented: `scripts/pilot_phase.py` (10,752 trials: 2 base speeds × 16 phases ×
[10 command-step directions × 4 sizes × limiter on/off + 8 push directions × 4 sizes]
× 3 repeats with small state noise). Metrics: fell, peak tilt, recovery time
(tilt < 5°, stride-averaged velocity error < 0.1 m/s), excess foot slip (loaded-contact
slip beyond undisturbed walking). Decide on effect size (phase swing, phase-gated
falls), not on p-values: in a near-deterministic simulator any phase effect is
"significant".
### Pilot result (full run, 10,752 trials, 2026-10-08), results/pilot_summary.txt
Median phase swing (worst-phase mean − best-phase mean, % of mean):

| group | falls | phase-gated | tilt swing | recovery swing | excess-slip swing |
|---|---|---|---|---|---|
| cmd step, v0 0.0, no limiter | 0% | 0/40 | 3.3° (85%) | 0.45 s (51%) | 58 mm (26%) |
| cmd step, v0 0.0, limiter | 0% | 0/40 | 2.3° (39%) | 0.32 s (22%) | 22 mm (14%) |
| cmd step, v0 0.4, no limiter | 0% | 0/40 | 3.6° (57%) | 0.42 s (57%) | 61 mm (56%) |
| cmd step, v0 0.4, limiter | 0% | 0/40 | 2.1° (37%) | 0.34 s (30%) | 30 mm (34%) |
| push, v0 0.0 | 24.5% | 12/32 | 10.8° (78%) | 0.48 s (40%) | 122 mm (63%) |
| push, v0 0.4 | 23.4% | 15/32 | 9.2° (70%) | 0.55 s (42%) | 180 mm (99%) |

Reading:
- Phase effect is large by the pre-set rule (swings > 30%, 27/64 push conditions
  phase-gated: 0% falls at one phase, 100% at another). Run-to-run noise is small
  (eta2 0.75–0.98).
- Mirror check passes: best phase for a push at direction d matches direction −d
  shifted by half a cycle (e.g. 45°/240 N best phi 0.125 ↔ 315°/240 N best 0.625;
  135° 0.688 ↔ 225° 0.188), as expected from the trot's diagonal-pair symmetry →
  physical effect, not noise. (The "worst phi = 0.000" entries are argmax ties.)
- Falls are phase-gated only for big pushes (180–240 N × 0.1 s ≈ 1.5–2 m/s Δv);
  realistic robot-robot bumps are far smaller. Command steps never caused falls.
- The slew limiter roughly halves the command-step phase effect (to ~2° / 0.3 s),
  i.e. uniform limiting already buys most of the safety, at the cost of latency.

Decision / revised claim: keep gait phase in the swarm layer, framed as
**phase-aware command timing**: instead of uniformly rate-limiting, release or shape
an aggressive velocity change according to phase → same tilt/fall safety with less
reaction latency (test as a Pareto front: reaction time vs. peak tilt/falls,
raw vs. uniform limiter vs. phase-aware).

Robustness checks still needed before claiming:
1. second low-level controller (MuJoCo Playground Go1 policy) shows the same effect;
2. other gait periods (0.30 / 0.42 s) and finer phase grid (32);
3. ~~capability map~~ done, see below.

### Capability map (scripts/capability_map.py, go1_bezier/capability.py)
Grid (v0 × size × direction × 16 phases) of mean tilt / fall prob. / recovery / slip,
pooled with the trot mirror symmetry, linear interpolation (periodic in phase and
direction). Saved as data/capability_map_bezier.npz.
- Mirror symmetry: r = 0.993 (mean |diff| 0.55°) vs. r = 0.853 (3.7°) for a plain
  half-cycle shift → strong physical structure.
- Held-out repeat: phase-aware vs phase-blind prediction of peak tilt
  R² 0.98 vs 0.87 (limiter), 0.94 vs 0.70 (no limiter), 0.97 vs 0.71 (push);
  push falls Brier 0.012 vs 0.073 (6× better). 8-phase map predicts the 8
  in-between phases with R² 0.88–0.98 → 16 phases is sufficient.
- Phase-aware timing (cross-validated; large command changes 0.6–0.8 m/s):

| strategy | tilt v0=0 | settle v0=0 | tilt v0=0.4 | settle v0=0.4 |
|---|---|---|---|---|
| no limiter, release now | 8.8° | 1.21 s | 8.9° | 1.13 s |
| uniform slew limiter | 6.5° | 1.21 s | 6.3° | 1.14 s |
| no limiter, wait ≤ ½ cycle (≈ 85 ms mean) | 5.8° | 1.13 s | 5.9° | 1.02 s |
| limiter + wait ≤ ½ cycle | 5.3° | 1.20 s | 5.3° | 1.17 s |

  Waiting on average ~85 ms for a good phase beats the uniform limiter on BOTH tilt
  (−0.4 to −0.7°) and settle time (−0.08 to −0.12 s). This is the core evidence for
  phase-aware command timing.
- Caveats: "settle" uses our recovery criterion (tilt < 5°, stride-averaged velocity
  error < 0.1 m/s); for collision avoidance the relevant number may be the time to
  e.g. 50% of the velocity change → measure it in the next experiment. Pushes cannot
  be timed; their map is for risk estimation (e.g. inflating safety margins at bad phases).
  All results are for the Bezier controller only.

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
