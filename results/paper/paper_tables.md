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

## Family B: prediction + timing vs prediction alone (main suite)

Holm over the 12 tests of this family.

| scenario | pair | metric | seeds | x | y | change | p (Holm) |
|---|---|---|---|---|---|---|---|
| swap4 | pred+timing - pred | urgent events / robot-min | 20 | 10.8 | 13.6 | -21% | 0.4184 |
| swap4 | pred+timing - pred | p99 tilt (deg) | 20 | 5.45 | 7.09 | -23% | 0.0002** |
| swap4 | pred+timing - pred | closest robot distance (m) | 20 | 0.727 | 0.723 | +1% | 1.0000 |
| swap8 | pred+timing - pred | urgent events / robot-min | 20 | 25.2 | 31.4 | -20% | 0.0001** |
| swap8 | pred+timing - pred | p99 tilt (deg) | 20 | 6.52 | 7.54 | -14% | 0.0102* |
| swap8 | pred+timing - pred | closest robot distance (m) | 20 | 0.643 | 0.614 | +5% | 0.4184 |
| formation9_pillars | pred+timing - pred | urgent events / robot-min | 20 | 34 | 41.5 | -18% | 0.0025** |
| formation9_pillars | pred+timing - pred | p99 tilt (deg) | 20 | 6.03 | 6.97 | -13% | 0.0000** |
| formation9_pillars | pred+timing - pred | closest robot distance (m) | 20 | 0.637 | 0.627 | +2% | 1.0000 |
| formation6_gate | pred+timing - pred | urgent events / robot-min | 20 | 20.9 | 21.6 | -3% | 1.0000 |
| formation6_gate | pred+timing - pred | p99 tilt (deg) | 20 | 5.14 | 5.82 | -12% | 0.0034** |
| formation6_gate | pred+timing - pred | closest robot distance (m) | 20 | 0.675 | 0.668 | +1% | 1.0000 |

## Family C: full system vs random wait (sham) and worst phase (timing suite)

Holm over the 24 tests of this family.

| scenario | pair | metric | seeds | x | y | change | p (Holm) |
|---|---|---|---|---|---|---|---|
| swap4 | full - sham | urgent events / robot-min | 20 | 10.9 | 8.87 | +23% | 1.0000 |
| swap4 | full - sham | p99 tilt (deg) | 20 | 5.3 | 5.98 | -11% | 0.0114* |
| swap4 | full - sham | closest robot distance (m) | 20 | 0.727 | 0.734 | -1% | 1.0000 |
| swap4 | full - worst | urgent events / robot-min | 20 | 10.9 | 12.8 | -15% | 1.0000 |
| swap4 | full - worst | p99 tilt (deg) | 20 | 5.3 | 6.69 | -21% | 0.0005** |
| swap4 | full - worst | closest robot distance (m) | 20 | 0.727 | 0.73 | -0% | 1.0000 |
| swap8 | full - sham | urgent events / robot-min | 20 | 28 | 25.7 | +9% | 1.0000 |
| swap8 | full - sham | p99 tilt (deg) | 20 | 6.54 | 6.63 | -1% | 1.0000 |
| swap8 | full - sham | closest robot distance (m) | 20 | 0.628 | 0.652 | -4% | 1.0000 |
| swap8 | full - worst | urgent events / robot-min | 20 | 28 | 25.5 | +10% | 1.0000 |
| swap8 | full - worst | p99 tilt (deg) | 20 | 6.54 | 6.88 | -5% | 0.7305 |
| swap8 | full - worst | closest robot distance (m) | 20 | 0.628 | 0.629 | -0% | 1.0000 |
| formation9_pillars | full - sham | urgent events / robot-min | 20 | 28.4 | 29.4 | -3% | 1.0000 |
| formation9_pillars | full - sham | p99 tilt (deg) | 20 | 5.76 | 6.14 | -6% | 0.0649 |
| formation9_pillars | full - sham | closest robot distance (m) | 20 | 0.651 | 0.64 | +2% | 1.0000 |
| formation9_pillars | full - worst | urgent events / robot-min | 20 | 28.4 | 29 | -2% | 1.0000 |
| formation9_pillars | full - worst | p99 tilt (deg) | 20 | 5.76 | 6.23 | -8% | 0.0025** |
| formation9_pillars | full - worst | closest robot distance (m) | 20 | 0.651 | 0.644 | +1% | 1.0000 |
| formation6_gate | full - sham | urgent events / robot-min | 20 | 16.2 | 17 | -5% | 1.0000 |
| formation6_gate | full - sham | p99 tilt (deg) | 20 | 4.62 | 4.83 | -5% | 1.0000 |
| formation6_gate | full - sham | closest robot distance (m) | 20 | 0.707 | 0.691 | +2% | 1.0000 |
| formation6_gate | full - worst | urgent events / robot-min | 20 | 16.2 | 16.3 | -1% | 1.0000 |
| formation6_gate | full - worst | p99 tilt (deg) | 20 | 4.62 | 4.94 | -7% | 0.5375 |
| formation6_gate | full - worst | closest robot distance (m) | 20 | 0.707 | 0.694 | +2% | 1.0000 |

Exploratory, pooled over seeds and scenarios (relative per-seed differences; not part of a corrected family):

| pair | scenarios | metric | change | p | n |
|---|---|---|---|---|---|
| full - sham | all 4 | urgent events / robot-min | +5.9% | 0.3218 | 80 |
| full - sham | all 4 | p99 tilt (deg) | -5.9% | 0.0000 | 80 |
| full - sham | all 4 | closest robot distance (m) | -0.2% | 0.8445 | 80 |
| full - sham | crowded 3 | urgent events / robot-min | +0.2% | 0.9519 | 60 |
| full - sham | crowded 3 | p99 tilt (deg) | -4.0% | 0.0083 | 60 |
| full - sham | crowded 3 | closest robot distance (m) | +0.1% | 0.9578 | 60 |
| full - worst | all 4 | urgent events / robot-min | -2.0% | 0.5672 | 80 |
| full - worst | all 4 | p99 tilt (deg) | -9.9% | 0.0000 | 80 |
| full - worst | all 4 | closest robot distance (m) | +0.6% | 0.4619 | 80 |
| full - worst | crowded 3 | urgent events / robot-min | +2.3% | 0.4750 | 60 |
| full - worst | crowded 3 | p99 tilt (deg) | -6.4% | 0.0000 | 60 |
| full - worst | crowded 3 | closest robot distance (m) | +0.9% | 0.3941 | 60 |
| sham - worst | all 4 | urgent events / robot-min | -6.1% | 0.1340 | 80 |
| sham - worst | all 4 | p99 tilt (deg) | -4.4% | 0.0005 | 80 |
| sham - worst | all 4 | closest robot distance (m) | +0.8% | 0.4188 | 80 |
| sham - worst | crowded 3 | urgent events / robot-min | +2.1% | 0.5102 | 60 |
| sham - worst | crowded 3 | p99 tilt (deg) | -2.4% | 0.0525 | 60 |
| sham - worst | crowded 3 | closest robot distance (m) | +0.9% | 0.5034 | 60 |
| sham - base | all 4 | urgent events / robot-min | -0.1% | 0.9910 | 80 |
| sham - base | all 4 | p99 tilt (deg) | -10.0% | 0.0000 | 80 |
| sham - base | all 4 | closest robot distance (m) | +3.4% | 0.0003 | 80 |
| sham - base | crowded 3 | urgent events / robot-min | -23.5% | 0.0000 | 60 |
| sham - base | crowded 3 | p99 tilt (deg) | -13.8% | 0.0000 | 60 |
| sham - base | crowded 3 | closest robot distance (m) | +5.0% | 0.0000 | 60 |

| scenario | config | mean hold (s) |
|---|---|---|
| formation9_pillars | predictive:phase+gaslc_equal:q | 0.092 |
| formation9_pillars | predictive:worst+gaslc_equal:q | 0.086 |
| formation9_pillars | predictive:sham+gaslc_equal:q | 0.090 |
| formation9_pillars | reactive:limiter | 0.000 |
| swap8 | reactive:limiter | 0.000 |
| swap8 | predictive:phase+gaslc_equal:q | 0.087 |
| swap8 | predictive:sham+gaslc_equal:q | 0.090 |
| swap8 | predictive:worst+gaslc_equal:q | 0.083 |
| formation6_gate | reactive:limiter | 0.000 |
| formation6_gate | predictive:phase+gaslc_equal:q | 0.092 |
| formation6_gate | predictive:worst+gaslc_equal:q | 0.086 |
| formation6_gate | predictive:sham+gaslc_equal:q | 0.091 |
| swap4 | reactive:limiter | 0.000 |
| swap4 | predictive:phase+gaslc_equal:q | 0.090 |
| swap4 | predictive:sham+gaslc_equal:q | 0.093 |
| swap4 | predictive:worst+gaslc_equal:q | 0.078 |

## Contacts and falls, full system vs baseline (all final suites + course)

| config | runs | falls | robot-robot contact onsets | robot-obstacle contact onsets |
|---|---|---|---|---|
| full | 230 | 0 | 16 | 1 |
| baseline | 170 | 2 | 28 | 4 |
