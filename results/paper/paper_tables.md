# Paper tables (full system vs baseline)

Full = `predictive:phase+gaslc_equal:q`, baseline = `reactive:limiter`; paired by seed; exact two-sided sign-flip permutation p; Holm over all 21 primary tests.

| scenario | robots | seeds | metric | baseline | full | change | p (Holm) |
|---|---|---|---|---|---|---|---|
| swap4 | 4 | 20 | urgent events / robot-min | 5.21 | 10.9 | +109% (worse) | 0.0070** |
| swap4 | 4 | 20 | p99 tilt (deg) | 5.9 | 5.3 | -10%  | 0.1055 |
| swap4 | 4 | 20 | closest robot distance (m) | 0.744 | 0.727 | -2% (worse) | 0.0060** |
| swap8 | 8 | 20 | urgent events / robot-min | 29.6 | 28 | -5%  | 0.8789 |
| swap8 | 8 | 20 | p99 tilt (deg) | 7.37 | 6.54 | -11%  | 0.0100** |
| swap8 | 8 | 20 | closest robot distance (m) | 0.626 | 0.628 | +0%  | 0.8951 |
| swap16 | 16 | 10 | urgent events / robot-min | 54.1 | 39.2 | -28%  | 0.0234* |
| swap16 | 16 | 10 | p99 tilt (deg) | 8.51 | 6.46 | -24%  | 0.0234* |
| swap16 | 16 | 10 | closest robot distance (m) | 0.6 | 0.637 | +6%  | 0.1055 |
| swap32 | 32 | 10 | urgent events / robot-min | 64.5 | 50.6 | -22%  | 0.0234* |
| swap32 | 32 | 10 | p99 tilt (deg) | 8.39 | 6.76 | -19%  | 0.0234* |
| swap32 | 32 | 10 | closest robot distance (m) | 0.566 | 0.578 | +2%  | 0.8789 |
| formation9_pillars | 9 | 20 | urgent events / robot-min | 40.4 | 28.4 | -30%  | 0.0000** |
| formation9_pillars | 9 | 20 | p99 tilt (deg) | 7.14 | 5.76 | -19%  | 0.0000** |
| formation9_pillars | 9 | 20 | closest robot distance (m) | 0.604 | 0.651 | +8%  | 0.0054** |
| formation6_gate | 6 | 20 | urgent events / robot-min | 24.4 | 16.2 | -34%  | 0.0012** |
| formation6_gate | 6 | 20 | p99 tilt (deg) | 5.84 | 4.62 | -21%  | 0.0001** |
| formation6_gate | 6 | 20 | closest robot distance (m) | 0.661 | 0.707 | +7%  | 0.0001** |
| formation16_pillars | 16 | 10 | urgent events / robot-min | 42 | 25.7 | -39%  | 0.0234* |
| formation16_pillars | 16 | 10 | p99 tilt (deg) | 7.22 | 5.62 | -22%  | 0.0234* |
| formation16_pillars | 16 | 10 | closest robot distance (m) | 0.593 | 0.617 | +4%  | 0.2266 |

Significant after Holm (p<0.05): 15 of 21 (13 in favour of the full system).

## Safety totals (all configurations)

| suite | scenario | config | runs | falls | not finished |
|---|---|---|---|---|---|
| main | formation9_pillars | predictive:phase+limiter | 20 | 0 | 0 |
| main | formation9_pillars | reactive:limiter:q | 20 | 0 | 1 |
| main | formation9_pillars | predictive:phase+gaslc_equal:q | 20 | 0 | 0 |
| main | formation9_pillars | reactive:limiter | 20 | 0 | 0 |
| main | formation9_pillars | predictive:limiter | 20 | 0 | 0 |
| main | formation9_pillars | predictive:phase+gaslc_equal | 20 | 0 | 0 |
| main | swap8 | reactive:limiter | 20 | 0 | 0 |
| main | swap8 | reactive:limiter:q | 20 | 0 | 0 |
| main | swap8 | predictive:limiter | 20 | 0 | 0 |
| main | swap8 | predictive:phase+limiter | 20 | 0 | 0 |
| main | swap8 | predictive:phase+gaslc_equal | 20 | 0 | 0 |
| main | swap8 | predictive:phase+gaslc_equal:q | 20 | 0 | 0 |
| main | formation6_gate | reactive:limiter | 20 | 0 | 0 |
| main | formation6_gate | reactive:limiter:q | 20 | 0 | 0 |
| main | formation6_gate | predictive:limiter | 20 | 0 | 0 |
| main | formation6_gate | predictive:phase+limiter | 20 | 0 | 0 |
| main | formation6_gate | predictive:phase+gaslc_equal | 20 | 0 | 0 |
| main | formation6_gate | predictive:phase+gaslc_equal:q | 20 | 0 | 0 |
| main | swap4 | reactive:limiter | 20 | 0 | 0 |
| main | swap4 | reactive:limiter:q | 20 | 0 | 0 |
| main | swap4 | predictive:limiter | 20 | 0 | 0 |
| main | swap4 | predictive:phase+limiter | 20 | 0 | 0 |
| main | swap4 | predictive:phase+gaslc_equal | 20 | 0 | 0 |
| main | swap4 | predictive:phase+gaslc_equal:q | 20 | 0 | 0 |
| scaling | swap32 | reactive:limiter | 10 | 0 | 0 |
| scaling | swap32 | predictive:limiter:q | 10 | 0 | 0 |
| scaling | swap32 | predictive:phase+gaslc_equal:q | 10 | 0 | 0 |
| scaling | swap16 | reactive:limiter | 10 | 0 | 0 |
| scaling | swap16 | predictive:limiter:q | 10 | 0 | 0 |
| scaling | swap16 | predictive:phase+gaslc_equal:q | 10 | 0 | 0 |
| scaling | formation16_pillars | reactive:limiter | 10 | 1 | 0 |
| scaling | formation16_pillars | predictive:phase+gaslc_equal:q | 10 | 0 | 0 |
| scaling | formation16_pillars | predictive:limiter:q | 10 | 0 | 2 |

## Lookahead (mean over seeds)

| scenario | config | p99 tilt | urgent/min | min dist | time | falls | not finished |
|---|---|---|---|---|---|---|---|
| formation6_gate | predictive@0.18:phase+gaslc_equal:q | 4.81 | 17.3 | 0.695 | 46.7 | 0 | 0 |
| formation6_gate | predictive@0.36:phase+gaslc_equal:q | 4.54 | 16.4 | 0.705 | 46.9 | 0 | 0 |
| formation6_gate | predictive@0.54:phase+gaslc_equal:q | 4.44 | 15.0 | 0.699 | 46.7 | 0 | 0 |
| formation6_gate | predictive@0.72:phase+gaslc_equal:q | 4.51 | 16.4 | 0.706 | 46.7 | 0 | 0 |
| formation6_gate | reactive:limiter | 5.95 | 26.3 | 0.658 | 46.9 | 0 | 0 |
| swap8 | predictive@0.18:phase+gaslc_equal:q | 6.27 | 24.2 | 0.647 | 15.3 | 0 | 0 |
| swap8 | predictive@0.36:phase+gaslc_equal:q | 6.63 | 29.9 | 0.604 | 15.5 | 0 | 0 |
| swap8 | predictive@0.54:phase+gaslc_equal:q | 6.14 | 23.2 | 0.669 | 15.3 | 0 | 0 |
| swap8 | predictive@0.72:phase+gaslc_equal:q | 6.31 | 25.1 | 0.635 | 15.8 | 0 | 0 |
| swap8 | reactive:limiter | 7.47 | 30.2 | 0.610 | 15.0 | 0 | 0 |

## Obstacle course (mean over seeds)

| team | config | runs | falls | not finished | time | p99 tilt | max tilt | obstacle clearance | urgent/min |
|---|---|---|---|---|---|---|---|---|---|
| 6 | reactive:limiter | 20 | 1 | 0 | 128.7 | 7.95 | 59.8 | 0.23 | 33.9 |
| 6 | predictive:limiter:q | 20 | 0 | 0 | 129.1 | 7.44 | 19.3 | 0.15 | 30.7 |
| 6 | predictive:phase+gaslc_equal:q | 20 | 0 | 1 | 129.3 | 6.68 | 18.8 | 0.18 | 29.2 |
| 1 | reactive:limiter | 20 | 0 | 0 | 117.8 | 4.78 | 12.4 | 0.27 | 7.4 |
| 1 | predictive:limiter:q | 20 | 0 | 0 | 117.9 | 4.92 | 12.1 | 0.20 | 11.6 |
| 1 | predictive:phase+gaslc_equal:q | 20 | 0 | 0 | 117.8 | 4.67 | 11.7 | 0.19 | 10.4 |
