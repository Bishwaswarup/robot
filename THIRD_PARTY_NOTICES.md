# Third-party notices

## spot_mini_mini (MIT)
The 12-point Bezier swing-curve control-point pattern in `go1_bezier/gait.py` is taken
from `spotmicro/GaitGenerator/Bezier.py` of
[OpenQuadruped/spot_mini_mini](https://github.com/OpenQuadruped/spot_mini_mini),
Copyright (c) 2020 Maurice Rahme, MIT License. The gait clock, stance integration,
lift-off-anchored swing and feedback terms are re-implemented here.
That curve follows Hyun et al., "High speed trot-running: Implementation of a
hierarchical controller using proprioceptive impedance control on the MIT Cheetah",
IJRR 2014.

## MuJoCo Menagerie: Unitree Go1 model (BSD-3-Clause)
The robot model is **not** included in this repository; it is loaded at runtime from a
separate clone of [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie).
`unitree_go1` is Copyright (c) 2016-2022 HangZhou YuShu Technology Co., Ltd.
("Unitree Robotics"), BSD 3-Clause License. Link and joint dimensions in
`go1_bezier/kinematics.py` are read from that model.
