# Timing controls: map-best phase vs random wait (sham) vs map-worst phase

Paired by seed; exact two-sided sign-flip p; Holm over every test in the table.

## Means

| scenario | config | runs | urgent/min | p99 tilt | closest dist | mean wait (s) | falls | not finished |
|---|---|---|---|---|---|---|---|---|
| formation9_pillars | base | 20 | 40.4 | 7.14 | 0.604 | 0.000 | 0 | 0 |
| formation9_pillars | full | 20 | 28.4 | 5.76 | 0.651 | 0.092 | 0 | 0 |
| formation9_pillars | sham | 20 | 29.4 | 6.14 | 0.640 | 0.090 | 0 | 0 |
| formation9_pillars | worst | 20 | 29.0 | 6.23 | 0.644 | 0.086 | 0 | 1 |
| swap8 | base | 20 | 29.6 | 7.37 | 0.626 | 0.000 | 0 | 0 |
| swap8 | full | 20 | 28.0 | 6.54 | 0.628 | 0.087 | 0 | 0 |
| swap8 | sham | 20 | 25.7 | 6.63 | 0.652 | 0.090 | 0 | 0 |
| swap8 | worst | 20 | 25.5 | 6.88 | 0.629 | 0.083 | 0 | 0 |
| formation6_gate | base | 20 | 24.4 | 5.84 | 0.661 | 0.000 | 0 | 0 |
| formation6_gate | full | 20 | 16.2 | 4.62 | 0.707 | 0.092 | 0 | 0 |
| formation6_gate | sham | 20 | 17.0 | 4.83 | 0.691 | 0.091 | 0 | 0 |
| formation6_gate | worst | 20 | 16.3 | 4.94 | 0.694 | 0.086 | 0 | 0 |
| swap4 | base | 20 | 5.2 | 5.90 | 0.744 | 0.000 | 0 | 0 |
| swap4 | full | 20 | 10.9 | 5.30 | 0.727 | 0.090 | 0 | 0 |
| swap4 | sham | 20 | 8.9 | 5.98 | 0.734 | 0.093 | 0 | 0 |
| swap4 | worst | 20 | 12.8 | 6.69 | 0.730 | 0.078 | 0 | 0 |

## Paired differences

| scenario | pair | metric | seeds | difference | relative | p (Holm) |
|---|---|---|---|---|---|---|
| formation9_pillars | full - sham | urgent events / robot-min | 20 | -0.971 | -3% | 1.0000 |
| formation9_pillars | full - sham | p99 tilt (deg) | 20 | -0.382 | -6% | 0.1206 |
| formation9_pillars | full - sham | closest distance (m) | 20 | +0.0105 | +2% | 1.0000 |
| formation9_pillars | full - worst | urgent events / robot-min | 20 | -0.607 | -2% | 1.0000 |
| formation9_pillars | full - worst | p99 tilt (deg) | 20 | -0.472 | -8% | 0.0046** |
| formation9_pillars | full - worst | closest distance (m) | 20 | +0.00695 | +1% | 1.0000 |
| formation9_pillars | sham - worst | urgent events / robot-min | 20 | +0.364 | +1% | 1.0000 |
| formation9_pillars | sham - worst | p99 tilt (deg) | 20 | -0.0896 | -1% | 1.0000 |
| formation9_pillars | sham - worst | closest distance (m) | 20 | -0.00355 | -1% | 1.0000 |
| formation9_pillars | sham - base | urgent events / robot-min | 20 | -11.1 | -27% | 0.0001** |
| formation9_pillars | sham - base | p99 tilt (deg) | 20 | -1 | -14% | 0.0001** |
| formation9_pillars | sham - base | closest distance (m) | 20 | +0.0367 | +6% | 0.1680 |
| swap8 | full - sham | urgent events / robot-min | 20 | +2.28 | +9% | 1.0000 |
| swap8 | full - sham | p99 tilt (deg) | 20 | -0.09 | -1% | 1.0000 |
| swap8 | full - sham | closest distance (m) | 20 | -0.0248 | -4% | 1.0000 |
| swap8 | full - worst | urgent events / robot-min | 20 | +2.48 | +10% | 1.0000 |
| swap8 | full - worst | p99 tilt (deg) | 20 | -0.345 | -5% | 1.0000 |
| swap8 | full - worst | closest distance (m) | 20 | -0.0013 | -0% | 1.0000 |
| swap8 | sham - worst | urgent events / robot-min | 20 | +0.204 | +1% | 1.0000 |
| swap8 | sham - worst | p99 tilt (deg) | 20 | -0.255 | -4% | 1.0000 |
| swap8 | sham - worst | closest distance (m) | 20 | +0.0235 | +4% | 1.0000 |
| swap8 | sham - base | urgent events / robot-min | 20 | -3.85 | -13% | 0.2171 |
| swap8 | sham - base | p99 tilt (deg) | 20 | -0.744 | -10% | 0.1196 |
| swap8 | sham - base | closest distance (m) | 20 | +0.0268 | +4% | 1.0000 |
| formation6_gate | full - sham | urgent events / robot-min | 20 | -0.84 | -5% | 1.0000 |
| formation6_gate | full - sham | p99 tilt (deg) | 20 | -0.219 | -5% | 1.0000 |
| formation6_gate | full - sham | closest distance (m) | 20 | +0.0161 | +2% | 1.0000 |
| formation6_gate | full - worst | urgent events / robot-min | 20 | -0.139 | -1% | 1.0000 |
| formation6_gate | full - worst | p99 tilt (deg) | 20 | -0.321 | -7% | 0.8868 |
| formation6_gate | full - worst | closest distance (m) | 20 | +0.0123 | +2% | 1.0000 |
| formation6_gate | sham - worst | urgent events / robot-min | 20 | +0.701 | +4% | 1.0000 |
| formation6_gate | sham - worst | p99 tilt (deg) | 20 | -0.102 | -2% | 1.0000 |
| formation6_gate | sham - worst | closest distance (m) | 20 | -0.0038 | -1% | 1.0000 |
| formation6_gate | sham - base | urgent events / robot-min | 20 | -7.36 | -30% | 0.0029** |
| formation6_gate | sham - base | p99 tilt (deg) | 20 | -1.01 | -17% | 0.0005** |
| formation6_gate | sham - base | closest distance (m) | 20 | +0.0296 | +4% | 0.1403 |
| swap4 | full - sham | urgent events / robot-min | 20 | +2.02 | +23% | 1.0000 |
| swap4 | full - sham | p99 tilt (deg) | 20 | -0.679 | -11% | 0.0218* |
| swap4 | full - sham | closest distance (m) | 20 | -0.0066 | -1% | 1.0000 |
| swap4 | full - worst | urgent events / robot-min | 20 | -1.91 | -15% | 1.0000 |
| swap4 | full - worst | p99 tilt (deg) | 20 | -1.38 | -21% | 0.0010** |
| swap4 | full - worst | closest distance (m) | 20 | -0.0025 | -0% | 1.0000 |
| swap4 | sham - worst | urgent events / robot-min | 20 | -3.93 | -31% | 0.5157 |
| swap4 | sham - worst | p99 tilt (deg) | 20 | -0.704 | -11% | 0.0578 |
| swap4 | sham - worst | closest distance (m) | 20 | +0.0041 | +1% | 1.0000 |
| swap4 | sham - base | urgent events / robot-min | 20 | +3.66 | +70% | 0.3088 |
| swap4 | sham - base | p99 tilt (deg) | 20 | +0.0853 | +1% | 1.0000 |
| swap4 | sham - base | closest distance (m) | 20 | -0.00995 | -1% | 1.0000 |

Reading: phase choice matters if 'full - sham' is negative (urgent, tilt) and significant; a dose-response full < sham < worst strengthens it. 'full - sham' ~ 0 means the gain is the delay.
