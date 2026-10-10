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

### Push capability envelope J_max (scripts/push_envelope.py) — design
Why: the pilot tested only 60/120/180/240 N and the fall transition lies between 120
and 180 N, so any fitted J_max there would be interpolation. A parametric
Fourier/logistic fit (support-diagonal axis 45/135°, folded mod 180°) was checked on the
held-out repeat: fall Brier 0.042–0.072 vs 0.012 for the lookup table, and it predicted
up to 9–19% falls at 120 N where none occurred; the true stance-diagonal axes are ±34°
(FR+RL: −34°, FL+RR: +34°), and forward vs backward pushes differ (27% vs 60% falls at
240 N), so folding mod 180° is not valid. → measure J_max directly.
Design: 2 speeds × 16 directions × 16 phases × 3 repeats, bisection 60–320 N
(6 steps, ~4 N), criteria fall and safe (no fall and tilt ≤ 20°), same state noise
for all probes of one bisection; monotonicity spot check at 0.85·J / 1.15·J; agreement
with pilot falls; maps in absolute direction and relative to the stance diagonal
(both half-cycles pooled by mirror symmetry).

### Push envelope result (full run, 3072 bisections, 2026-10-08)
- Clean thresholds: monotonicity check 99% pass at 0.85·J, 98% fail at 1.15·J;
  repeat-to-repeat std 1.9 N; 40/1536 fall-cells censored above 320 N.
- J_fall median ~215 N (125–≥320 N); J_safe (no fall, tilt ≤ 20°) median 167 N (101–317 N).
- Phase swing per direction: median 106–121 N (fall), 84–109 N (safe).
- Knowing the phase: mean allowable push / phase-blind worst case = 1.33× (fall),
  1.38–1.44× (safe).
- Mirror symmetry r = 0.991 (fall), 0.984 (safe).
- The envelope explains the pilot: "falls iff F > J_fall" matches 98.6% of 3072
  pilot push trials.
- **Effective support axis (physical model).** The direction the robot tolerates best
  rotates during stance from ≈ +25° (just after touchdown) to ≈ +65° (mid-stance), and
  +68° is the axis of the *next* diagonal. So the robot best resists pushes along the
  diagonal that will catch it. Model J = J0(ψ) + A(ψ)·cos 2(θr − α(ψ)): held-out
  R² 0.82–0.84 (fall), 0.68–0.82 (safe); 4 harmonics 0.89–0.95; full 16×16 table
  0.96–0.97. → interpretable model for the paper, table for control.
- Relevance: a walking-speed bump between two Go1s (~3 N·s ≈ 30 N for 0.1 s) is far
  below J_max; the envelope matters for hard impacts and as a phase-dependent margin.

Note: thresholds (tilt 20°) still arbitrary; a walking-speed bump between two Go1s is
~3 N·s (≈ 30 N for 0.1 s), far below J_max → J_max matters for hard impacts.

### Closed-loop phase-aware commander (go1_bezier/commander.py, scripts/commander_eval.py)
Small changes (< 0.15 m/s) pass through; a large change is held ≤ ½ gait period and
released at the phase minimising predicted tilt + 50°·P(fall) (capability map);
`urgent=True` bypasses (imminent collision). Strategies compared on identical
conditions/timings/noise (paired): raw, limiter (old default), phase, phase+limiter.
Metrics from the REQUEST: t50/t90 of the velocity change (half-period smoothing),
peak tilt, falls, settle; plus a 60 s 10 Hz random-command stream test.
Smoke test (v0 0.4, 4 directions, 0.6 m/s, 16 paired trials): phase vs limiter: tilt
equal (+0.0°, CI ±1°), t50 −0.09 s, t90 −0.13 s (CI excludes 0); phase+limiter vs
limiter: tilt −1.1° but t50 +0.09 s. → combined with the limiter, phase timing gives
lower tilt; without it, the same tilt as the limiter with a faster response. Full run pending. (Offline prediction was tilt −0.4…−0.7° at
faster settle; live, the tilt benefit is smaller.)

### Commander result (full run, 2026-10-08): results/commander_summary.txt
STEP test (960 paired trials per strategy; events inside the map's domain):
| v0 | strategy | tilt | t50 | t90 | settle |
|---|---|---|---|---|---|
| 0.0 | raw / limiter / **phase** / phase+lim | 8.28 / 6.09 / **5.60** / 4.94° | 0.21 / 0.36 / **0.29** / 0.43 s | 0.30 / 0.47 / **0.36** / 0.55 s | 1.14 / 1.14 / **1.05** / 1.13 s |
| 0.4 | raw / limiter / **phase** / phase+lim | 8.10 / 6.04 / **5.76** / 4.99° | 0.20 / 0.33 / **0.27** / 0.41 s | 0.28 / 0.45 / **0.35** / 0.52 s | 1.09 / 1.11 / **1.01** / 1.16 s |
Paired phase − limiter: tilt −0.49° / −0.29° (CI excl. 0), t50 −0.070 / −0.053 s,
t90 −0.115 / −0.097 s, settle −0.091 / −0.097 s → phase timing DOMINATES the uniform
limiter (less tilt AND faster) for in-domain events. phase+limiter: −1.1° tilt, +0.07–0.08 s t50.

STREAM test (6 × 60 s random commands at 10 Hz): phase 0.33 falls/min (2 falls), p99
tilt 13.7° ≈ raw (14.8°) vs limiter 7.8°, 0 falls. Diagnosis: the stream's big events
are combined planar+yaw jumps (up to ~1.1 m/s and 1.2 rad/s) from turning/sideways,
unsettled walking — outside the map (straight walking at 0/0.4 m/s, single-axis change
≤ 0.8). Picking the "best" phase from a model outside its domain can pick a bad one.
Fix: strategy phase_safe = phase timing in-domain, uniform limiter otherwise (domain
check: settled, straight base, single change ≤ 0.85): stream 0 falls, mean tilt 2.11°
(limiter 2.08°), p99 9.1° (7.8°), max 28.6° (12.8°), tracking = limiter. In this stream
~97% of big events were out-of-domain, so phase_safe ≈ limiter there; identical to phase
in the step test. phase_soft (phase + 2× softer limiter): p99 13.2°, no gain.
→ To generalise, extend the map to a capability MODEL over arbitrary base velocity
(vx, vy, wz) and combined changes (random-event dataset + learned regressor), and
evaluate with the real swarm layer (ORCA commands are smoother than random jumps).
(Seeds now use crc32, so runs are reproducible across processes.)

### Swarm layer v0 (go1_bezier/orca.py, swarm.py, scenarios.py; scripts/swarm_eval.py)
Per robot, 10 Hz, local info only (neighbours ≤ 4 m, measured velocities): goal or
formation slot (virtual reference with shared clock, elastic lateral offsets from local
obstacle sensing) → ORCA (RVO2 LP port, τ = 2 s, r = 0.38 m, keep-right 15° bias to break
symmetric deadlocks — without it 8-agent swaps deadlock) → heading control → commander
(urgent if TTC with current command < 0.8 s) → Bezier controller.

First run (1 seed per scenario; Linux):
| scenario | raw | limiter | phase | phase_safe |
|---|---|---|---|---|
| swap4 | ok, p99 tilt 6.3° | ok, 5.7° | ok, 4.6° | = limiter |
| swap8 | ok, 12.8°, min d 0.61 | ok, 7.2°, 0.57 | ok, 9.8°, 0.69 | = limiter |
| formation9 pillars | 5 falls | ok, 8.2° | 6 falls, fail | ok, 8.6° |
| formation6 gate | 4 falls, fail | ok, 7.0°, 47 s | 1 fall, fail | ok, 8.2°, 47 s |

Key findings:
1. Only 0–2% of the swarm's big command changes are inside the capability map's domain
   (settled, straight, single change). Swarm changes are frequent (20–100 per robot-minute),
   mostly turning (median |Δwz| ≈ 0.48 rad/s with |Δv| ≈ 0.15–0.17 m/s) from unsettled walking,
   and ~50% are urgent (TTC < 0.8 s) → waiting for a phase is rarely possible or sensible.
2. Without a slew limiter (raw, phase) the robots fall in the formation scenarios; the
   uniform limiter and phase_safe: 0 falls, 100% success. phase_safe ≈ limiter.
3. Min centre distance drops below 2r (0.57–0.62 vs 0.76 m) because of tracking lag;
   faster response (phase in swap8: 0.69 m) improves separation at the cost of tilt.
→ Phase-aware TIMING (waiting) does not transfer to dense swarm operation.
→ Proposed pivot back to the original idea: a gait-aware FEASIBLE SET / slew limit —
   allowed velocity change per tick as a function of (phase, direction, state), from the
   capability data, applied every tick without waiting (also usable as the ORCA LP's
   velocity bound). Expected benefit: faster reaction (larger min distance, fewer urgent
   events) at the same tilt/falls as the uniform limiter. Needs a capability model over
   arbitrary state + combined changes (random-event data).

### Swarm full run (3 seeds, user's Mac) — confirms the single-seed picture
raw/phase: falls in swap8 (3/2), pillars (12/12), gate (6/2), success 33% in formation
scenarios; limiter & phase_safe: 0 falls, 100% success. In-domain fraction 0–2%.
Viewer observation (Bishwa): robots "get confused" at the gate entrance and need a few
tries before passing → ORCA jostling at a bottleneck (urgent events peak there). Future:
bottleneck ordering / queueing (deadlock-recovery item of the original plan).

### Gait-aware slew limiter (go1_bezier/gasl.py) — first results
Idea: no waiting; the per-step acceleration limit is scaled by the robot's tolerance at
the current phase and acceleration direction. Two tolerance sources:
  push   : push envelope J_safe(θ, φ)  (d'Alembert analogy: acceleration ≈ push)
  cmd    : 1 / (tilt per m/s of an instantaneous step at φ)  from the capability map —
           under linear superposition, ramp tilt ≈ ∫ a(t)·K(θ, φ(t)) dt, so put the
           acceleration where K is small.
Modes: "equal" (same mean limit as uniform, redistributed), "x k" (mean k×uniform).
Fair comparison = uniform limiter family limiter*k (k = 0.75, 1, 1.35, 1.7).
Step test (v0 0.4, size 0.6, 60 paired trials, Linux):
| strategy | tilt | t50 |
|---|---|---|
| limiter*0.75 / 1 / 1.35 / 1.7 | 5.54 / 6.43 / 7.00 / 7.42° | 0.365 / 0.332 / 0.305 / 0.288 s |
| gasl_equal (push) | 6.54° | 0.336 s |
| gaslc_equal (cmd) | 6.23° | 0.328 s |
| gaslc_x1.35 | 6.93° | 0.302 s |
- push-shaped: no gain (+0.1°, n.s.) → the d'Alembert analogy does not hold here.
- cmd-shaped: −0.20° at equal speed (CI [−0.35, −0.03]), gains concentrated in backward
  diagonals (135°/225°: −0.8°); vs limiter*1.35: −0.07° (n.s.). Phase lead 0.125/0.25: no help.
- Interpretation: a limiter ramp spans ~1–1.5 strides, so shaping WITHIN the ramp averages
  most of the phase effect out; the uniform acceleration magnitude dominates.
Swarm (1 seed): gaslc_x1.35 vs limiter*1.35: swap8 min dist 0.70 vs 0.64 m, pillars p99
tilt 9.6 vs 13.3°, max 20.5 vs 43.9°; gaslc_equal vs limiter mixed. Needs the 5-seed run.

Status of the "gait-aware" claim so far:
  strong   for isolated events: phase gates falls (pilot), J_max 1.33–1.44× (envelope),
           timing dominates uniform limiting in the step test;
  weak     for continuous reactive swarm commands: waiting rarely possible (urgent ~50%,
           in-domain ~1%), ramp shaping ≈ 3% tilt reduction.
Options if the multi-seed swarm run confirms "weak":
  (D) predictive swarm layer (plan manoeuvres ≥ 1 stride ahead → turn urgent reactions into
      scheduled ones, where phase timing works);
  (C) regimes where phase matters more (higher speed, payload, slopes, other gaits/controllers);
  (B) write it up as a characterisation: when gait phase matters for multi-robot control and
      when it does not.

### GASL full results (user's Mac, 2026-10-08)
STEP test (480 paired trials per v0 and strategy). Distance to the uniform-limiter curve
(limiter ×0.75/1/1.35/1.7), interpolated at equal response time t50:
| strategy | v0 = 0 | v0 = 0.4 |
|---|---|---|
| gasl_equal (push-shaped) | −0.13° (−2%) | +0.09° (+2%) |
| gaslc_equal (step-sensitivity-shaped) | −0.50° (−8%), ≙ 24 ms faster at equal tilt | −0.22° (−3.5%), 9 ms |
| gaslc_x1.35 | −0.45° (−6.5%), 17 ms | −0.21° (−3%), 9 ms |
| gaslc_x1.7 | −0.37° (−5%), 19 ms | −0.26° (−4%), 12 ms |
Paired CIs exclude 0 for all gaslc rows. → small, consistent benefit of step-sensitivity
shaping; push-envelope shaping ≈ none.
STREAM (6×60 s): gaslc_equal p99 tilt 7.39° vs limiter 7.82°; limiter*1.35 and gaslc_x1.35
each 1 fall; gaslc_x1.7 no gain over limiter*1.7.
SWARM (5 seeds × 4 scenarios): falls — limiter 0, gaslc_equal 1, limiter*1.35 2,
gaslc_x1.35 0, raw 37. At the 1.35× level gait-aware shaping looks more robust
(pillars: 0 vs 2 falls, max tilt 25 vs 59°; gate: max 22 vs 43°, min dist 0.65 vs 0.52 m);
at 1× it is mixed (swap8 p99 6.7 vs 7.3° but min dist 0.52 vs 0.57 m; pillars 1 fall vs 0).
Gate: limiter and gaslc_equal each timed out once (bottleneck jam, not falls).
Verdict: real but small (3–8% tilt at equal speed). Phase TIMING had the larger effect
(−0.3…−0.5° AND 50–70 ms faster) but only applies to planned/non-urgent manoeuvres →
next: predictive swarm layer (option D) + bottleneck ordering at gates.

### Predictive swarm layer (option D) — implemented, full run pending
- SwarmAgent.predictive(): ORCA solved for the situation one lookahead (0.36 s = 1 gait
  period) ahead — own position extrapolated with the CURRENT command, neighbours with
  measured velocities. The resulting velocity is requested with deadline = lookahead, so
  timing strategies (phase+limiter, phase+gaslc_equal) may pick the gait phase without
  being late. If the current command is unsafe (TTC < 0.8 s) → reactive + urgent.
- Commander: request(..., deadline=s) caps the wait at min(max_wait, deadline);
  "phase+gaslc_equal" = phase timing + step-sensitivity-shaped limiter.
- Bottleneck queueing (":q"): near obstacles (elastic scale < 0.6) the robot further back
  takes ORCA responsibility 0.9, the one in front 0.1 (instead of 0.5/0.5).
- Configs compared: reactive:limiter (baseline), reactive:limiter:q, predictive:limiter:q,
  predictive:phase+limiter:q, predictive:phase+gaslc_equal:q.
- Single run (swap8, seed 0): predictive:phase+limiter:q vs reactive:limiter —
  min dist 0.66 vs 0.57 m, p99 tilt 6.6 vs 7.2°, max 10.9 vs 13.6°, urgent 40% vs 55%.

### Predictive layer — 5-seed results (results/predictive, user's Mac)
| scenario | metric | reactive:limiter | predictive:limiter:q | predictive:phase+limiter:q | predictive:phase+gaslc_equal:q |
|---|---|---|---|---|---|
| swap8 | urgent / min dist / contacts / p99 | 54% / 0.57 / 0.6 / 7.3° | 53% / 0.51 / 0.8 / 8.0° | 37% / 0.62 / 0 / 6.9° | 39% / 0.63 / 0 / 7.0° |
| pillars | urgent / contacts / p99 / max | 55% / 0.4 / 8.5° / 25° | 46% / 0 / 7.5° / 19° | 38% / 0 / 7.0° / 20° | 41% / 0 / 7.0° / 21° |
| gate | success / urgent / p99 / max | 80% / 53% / 6.3° / 16° | 80% / 54% / 5.6° / 15° | 100% / 45% / 5.3° / 15° | 100% / 43% / 4.8° / 12° |
- No falls in any predictive config. Prediction ALONE is mixed (swap8 worse); queueing alone ≈ no
  effect; prediction + phase timing improves all scenarios → the gait-phase effect matters in the
  swarm once the planner gives the commander slack (core claim, revised).
- Costs: swaps +0.5–0.6 s (4–7%) from the ~85 ms mean wait; max tilt in swap8 higher (17.8–19.7°
  vs 13.8°) although p99 lower.
- Viewer (Bishwa): at the gate the predictive version kept formation, did not lag, no crowding.
- "time" for formation scenarios is floored by the reference arrival (46.7 s gate) → added
  form_err / lag / arrive metrics.
- Statistics: with 5 seeds the sign-flip permutation test cannot go below p = 0.0625 → 20 seeds.

### Paper experiment plan (scripts/paper_runs.py)
main: 4 scenarios × {reactive:limiter, reactive:limiter:q, predictive:limiter,
predictive:phase+limiter, predictive:phase+gaslc_equal, predictive:phase+gaslc_equal:q} × 20 seeds
(ablation: queueing / prediction / timing / shaping). lookahead: 0.18–0.72 s on swap8 + gate,
10 seeds. scaling: swap16, swap32, formation16_pillars × {baseline, predictive:limiter:q, full},
10 seeds. Paired stats (scripts/swarm_stats.py): bootstrap CI + sign-flip permutation p;
pre-registered primary metrics: urgent/min, min_dist, p99_tilt, success (Holm-corrected).

### Obstacle course (go1_bezier/course.py, scripts/course_eval.py, examples/course_demo.py)
~46 m path, side walls: slalom poles (two on the centre line) → 1.9 m gate → chicane (two staggered
walls, path swings to y = ±2.4) → crossing obstacle (0.4 m/s, robots take full responsibility,
+0.15 m margin) → 7-pillar field → finish straight. Team follows the waypoint path (0.3 m/s).
Design lessons from building it:
- Rigid formations rotating with the path swing slots through walls → CURVILINEAR slots (dx along
  the path, dy across it), the team bends like a train.
- Lateral width from the known course geometry: lam(s) = largest fraction of the offsets keeping
  slots ≥ 0.75 m from static obstacles, min over [tail, +3 m] → collapses to single file
  (queue order, 0.95 m gaps) before narrow parts and stays collapsed until the tail is through.
- Straight-line attraction to a slot behind a wall = ORCA local minimum (a robot got stuck behind
  the chicane wall) → path-following preference: carrot 1 m ahead of the robot's own tracked
  path progress (windowed, so it can't jump to a neighbouring leg).
- Static obstacles: +0.1 m ORCA margin, τ_obs = 2 s, and "urgent" also when the current command
  reaches an (inflated) static obstacle within 1.5 s — the predictive layer otherwise passed
  walls/poles with less clearance than reactive (deadline + slew-limited ramp react later).
First runs (seed 0): team 1: both configs succeed, 0 contacts, equal time (reference-limited
154 s); team 6: both succeed, 0 falls, 0 robot-robot/mover contacts; predictive:phase+gaslc_equal:q
vs reactive:limiter: urgent 18.7 vs 28.2 /robot-min, p99 tilt 6.9 vs 8.2°, max 21 vs 29°, lag 0.17
vs 0.23 m — but more light static-obstacle contacts (11 vs 3) before the static-urgency fix; full
5-seed run pending (run after the paper runs).

## PAPER RESULTS (results/paper, 2026-10-08) — full system = predictive:phase+gaslc_equal:q vs reactive:limiter
Paired by seed; p = sign-flip permutation (20 seeds main, 10 seeds lookahead/scaling).
| scenario (N) | urgent / robot-min | p99 tilt | max tilt | min dist | success | falls | time |
|---|---|---|---|---|---|---|---|
| swap4 (4) | +6.0** (worse) | −0.55° (−9%)* | −1.0°** | −0.016 m** | 20/20 = | 0 = | +0.57 s** (+7%) |
| swap8 (8) | −1.9 (n.s.); urgent share −17 pp** | −0.62° (−9%)* | −0.7 n.s. | +0.014 n.s. | 20/20 = | 0 = | +0.42 s** (+3%) |
| pillars (9) | −18.0** (−34%) | −1.81° (−21%)** | −1.4 n.s. | +0.046 (p .06) | 20/20 = | 0 = | +0.7 n.s. |
| gate (6) | −6.8** (−29%) | −1.60° (−24%)** | −4.05°** | +0.064 m** | 19/20 = | 0 = | 0 |
| swap16 (16) | −13.8** (−26%) | −1.60° (−19%)** | −2.1 n.s. | +0.052 m* | 10/10 = | 0 = | −0.6 n.s. |
| swap32 (32) | −11.8** (−18%) | −1.48° (−17%)** | −1.8 n.s. | +0.036 (p .1) | 10/10 = | 0 = | −0.4 n.s. |
| formation16 pillars (16) | −21.7** (−40%) | −2.03° (−23%)** | −4.4 (p .08) | +0.063 m* | 10/10 = | 0 = | +2.5 n.s. |
Ablation (main): prediction alone (predictive:limiter) — worse in swaps (p99 +0.6–1.0°, slower),
3 falls / 19/20 success in pillars; queueing alone — 2 falls in pillars, gate success 17/20.
Timing is the active ingredient: predictive:phase+limiter already gives most of the gain; the shaped
limiter adds a little (pillars urgent −18 vs −10/min, p99 −1.8 vs −1.4°). Only configs WITH phase
timing had zero falls across all 480 main runs.
Lookahead (10 seeds): urgent share falls monotonically with lookahead (−14 pp @0.18 s → −34 pp
@0.72 s) but command changes per minute rise (53 → 114 big/robot-min) and the gate degrades at
≥0.54 s (success 8/10, form err +0.1 m); swap8 slower at 0.72 s (+0.54 s*). Sweet spot 0.18–0.36 s
(½–1 gait period); 0.18 s was best on gate (p99 −2.05°**, max −4.8°**, min dist +0.053 m**, 10/10).
Scaling: benefits hold from 4 to 32 robots (largest relative gains in the densest/most obstructed
scenarios); prediction alone gives none (swap32 robot contacts +1.6, formation16 obstacle contacts
+2.7*). Zero falls in all scaling runs.
Weak spots: (1) sparse swap4 — more urgent events and +7% time (little to gain, waiting costs);
(2) robot-obstacle contacts up with prediction (formation16 +2.0 n.s., pillars/gate small) → port
the static-obstacle urgency (TTC to inflated obstacles < 1.5 s → react now) from course_eval into
swarm_eval and re-run the obstacle scenarios; (3) primary-metric p-values are mostly ≤ 0.002 and
survive Holm correction; secondary metrics should be reported as exploratory.

## OBSTACLE COURSE (results/course, speed 0.45 m/s, 5 seeds, paired vs reactive:limiter)
All 30 runs finished. Full system (predictive:phase+gaslc_equal:q):
- Team 6: urgent/robot-min 45.3 -> 26.4 (-42%, lower in 5/5 seeds), p99 tilt 8.4 -> 6.9 deg (5/5),
  static-obstacle contacts 1.8 -> 0 per run, obstacle clearance 0.22 -> 0.32 m, robot min dist 0.59 -> 0.63 m,
  0 falls, time 114.6 -> 111.7 s. Seed 3 had 2 robot-robot touches (max tilt 24 deg) — the only blemish.
- Team 1: time 104.8 -> 102.7 s, p99 6.1 -> 5.2 deg, clearance 0.28 -> 0.38 m, lag 0.85 -> 0.53 m.
- Prediction without phase (predictive:limiter:q): faster, but 1 fall (max tilt 59 deg) and 1 robot-robot
  touch in team 6, p99 unchanged — same pattern as the paper runs.
- Where the time goes: chicane is the slowest section (35 s solo, 47 s team); pillars is where prediction
  saves the most (-3 to -5 s).
Caveats: with 5 seeds the smallest possible two-sided p is 0.0625, so nothing can reach p<0.05 -> rerun with
10-20 seeds. The moving obstacle never came close to the solo robot (clearance 1.83 m) and stayed >=0.42 m
from the team -> the crossing currently isn't a test; retime the mover to cross the robot's path. The
slalom path runs nearly straight between the pillar rows (y ~ -0.5) -> make it actually weave.

### Course v2 + obstacle-safety suite (2026-10-09)
- Old course results kept in results/course_v1.
- Slalom: 3 poles on the centre line (x=4,6,8), path weaves +-1.3 m (pole clearance >= 0.79 m centre);
  teams fall into single file (start moved to x=-3 to give room).
- Crossing: two movers (x=22.5 up, x=25 down, 0.4 m/s) timed from the reference speed to pass y=0 when
  the lead row / last row is ~1 m short. Smoke test (speed 0.45, 1 seed): mover clearance 1.83 m -> 0.5 m
  (robots now have to avoid), team-6 baseline fell once, full system 0 falls, p99 9.0 -> 7.1 deg.
- swarm_eval: opt-in static safety (margin 0.1, tau_obs 2, static TTC 1.5); defaults reproduce the
  paper runs exactly (checked gate/full/seed 0). paper_runs suite "obstacles": pillars9, gate6,
  pillars16 x {base, pred:limiter:q, pred:phase+limiter:q, full} x 15 seeds. Smoke: gate full seed 0
  obstacle touch 1 -> 0.

### Course v1 at 20 seeds (speed 0.45; old course - the new course.py had been overwritten on the Mac)
Results kept in results/course_v1_20seeds. Paired sign-flip p (20 seeds), full system vs reactive:limiter:
- Team 6: p99 tilt 8.44 -> 6.90 deg (better in 20/20, p<.0001); urgent/robot-min 43.9 -> 28.5 (-35%, p=.0001);
  robot min dist 0.571 -> 0.618 m (p=.0003); max tilt 18.8 -> 16.5 (p=.04); obstacle-contact onsets
  27 -> 1 in total (runs with a contact 5 -> 1; p=.06, contacts clumped in a few runs); time equal (112.6 vs 112.8 s).
- Team 1: time 104.6 -> 102.7 s (20/20), p99 6.1 -> 4.9 deg (20/20), max 12.0 -> 9.1 deg (19/20),
  obstacle clearance 0.33 -> 0.41 m (p=.0003), urgent -19% (p=.005), lag 0.84 -> 0.59 m.
- Prediction without phase (pred:limiter:q), team 6: urgent -22% and p99 -0.5 deg, but max tilt unchanged
  and the only fall in 120 runs (59 deg); full system is better on every safety metric.
- User-added fixes kept: n/a for single-robot metrics, makedirs before writing, tests/test_course_eval.py.

### Course v2 (20 seeds, speed 0.45) + obstacle-safety suite (15 seeds) - 2026-10-09
Course v2 team 6: full system p99 7.8 vs 9.0 deg and urgent 32 vs 39/min, BUT 3 falls (seeds 4, 12, 13)
and 1 failure (seed 12) vs 0 for baseline / prediction-only (Fisher p ~ 0.23; still bad optics).
Diagnosis (fall_where logging added to both eval scripts; seed 12 reproduces on Linux):
- seed 12: no contact. Robot 0 at the first slalom corner (4.0, -1.1) faced -90 deg from its travel
  direction; ORCA direction flipped -> heading target flipped -> commands saturated at
  vx 0.5 + vy 0.27-0.35 + wz 1.0-1.2 simultaneously (outside the stress-tested box |vy|<=0.3, |wz|<=1.0);
  roll 15 -> 60 deg in 0.8 s. Single-robot sweep: such combos alone give up to 16 deg tilt (no fall),
  so it is the combination with fast re-targeting.
- Swarm obstacle suite falls (all configs, incl. baseline): every one after a pillar contact at the same
  pillars ((8.0,0.3) in pillars9, (6.25,0.10) in pillars16), robot centre ~0.55 m from pillar centre
  (ORCA wants 0.78): squeezed by neighbours + command tracking lag. Shared weakness, not timing-specific.
Fix (opt-in, --turn-gate): SwarmAgent body-command envelope |wz|<=1.0, |vy|<=0.3, and translation
scaled 1.0 -> 0.3 as heading error goes 30 -> 90 deg (turn first, then walk). Cloud check seed 12:
full system fall -> 0 falls (max 14 deg); baseline 132.8 -> 125.9 s, urgent 46 -> 33/min with the gate.
Obstacle suite (static safety on, 15 seeds), full vs base: pillars9 urgent -15.7/min**, p99 -1.40**,
min dist +0.069**; gate p99 -1.95**, max -4.1**, min dist +0.057*; pillars16 p99 -1.99**, min dist
+0.068**, urgent -21.1** but 13/15 success (1 fall, 1 stuck) vs 15/15. Obstacle touches with safety:
full 0.4 / 0 / 0.5 per run (pillars9 / gate / pillars16) vs 1.05 / 0.45 / 2.5 without.

### Turn-gate runs (course 20 seeds, obstacle suite 15 seeds) + local-minimum escape - 2026-10-09
Turn gate removed most falls: course team 6 falls base/pred/full = 1/0/0 (were 0/0/3); obstacle suite
falls 1/0/0/1 (base, pred, phase+lim, full) vs 1/2/2/1 before; obstacle touches near 0 everywhere.
Full system still best on p99 (course 6.7 vs 7.9 deg; pillars9 5.7 vs 7.2; gate 4.5 vs 5.9; pillars16 5.5
vs 7.1) and min dist (+5-9 cm). NEW problem: 13 stuck runs, ALL in predictive configs (success 80-93 %),
course full 18/20. Diagnosis (gate seed 0, reproduced): robot 0 pressed against the gate wall at
(6.34,-1.35) with its preferred velocity pointing through the wall to its slot -> ORCA returns ~0 for
45 s (local minimum; swarm scenarios have no path planner). The gate's slow turning made robots end up
there more often. Fix: SwarmAgent(escape=True): if ORCA output < 0.05 m/s while the preferred speed
> 0.15 for > 1 s, turn the preferred velocity 75 deg towards a hint (formation reference point /
path 1.5 m ahead) for 2 s, flip side if still stuck. Cloud check: gate seeds 0, 5 (phase+lim),
pillars9 seed 3 (full), course seeds 1, 12 (full) -> all succeed, 0 falls, 0 contacts.
--robust = static safety + turn gate + escape (swarm_eval, paper_runs -> results/paper/<suite>_robust,
course_eval). Plan: run all paper suites + course with --robust and use those as the paper numbers.
