"""The trick routine: height range, lie down / stand up, sit on hind legs and wave
the front paws, sit on front legs and wave the hind paws.

Each wave tests one leg's IK and joint control on its own; the height steps show the
usable stand-height range. Values tuned in MuJoCo with the Menagerie Go1.
"""
import numpy as np
from .poses import Keyframe

H = 0.27                      # nominal stand height (trunk)
SIT_PITCH = 0.45              # rad (~26 deg) for sitting on hind / front legs


def build_tricks(h=H, h_min=0.16, h_max=0.34, lie=0.11, sit_pitch=SIT_PITCH, sit_z=0.20,
                 shift=0.035):
    p, z = sit_pitch, sit_z
    wave = (0.04, 0.05, 1.5)       # (amp_y, amp_z, Hz)
    paw = {"FR": (0.14, -0.10, -0.12), "FL": (0.14, 0.10, -0.12),
           "RL": (-0.14, 0.10, -0.12), "RR": (-0.14, -0.10, -0.12)}

    def waving(body_x, pitch, leg, side):
        """shift weight away from `leg`, lift + wave it, put it down."""
        y, r = side * shift, side * 0.05
        b = (body_x, y, z, r, pitch)
        name = {"FR": "front RIGHT", "FL": "front LEFT", "RL": "hind LEFT", "RR": "hind RIGHT"}[leg]
        return [Keyframe(1.5, b, label=f"shift weight off {name}"),
                Keyframe(4.0, b, lift={leg: paw[leg]}, wave={leg: wave}, label=f"wave {name}"),
                Keyframe(1.2, b, label=f"put {name} down")]

    return ([
        Keyframe(1.5, (0, 0, h, 0, 0), label="stand"),
        Keyframe(2.0, (0, 0, h_min, 0, 0), label=f"lowest stand ({h_min:.2f} m)"),
        Keyframe(2.5, (0, 0, h_max, 0, 0), label=f"highest stand ({h_max:.2f} m)"),
        Keyframe(1.5, (0, 0, h, 0, 0), label="stand"),
        Keyframe(2.5, (0, 0, lie, 0, 0), label="lie down"),
        Keyframe(2.5, (0, 0, h, 0, 0), label="stand up"),
        Keyframe(2.5, (-0.05, 0, z, 0, -p), label="sit on hind legs")]
        + waving(-0.05, -p, "FR", +1) + waving(-0.05, -p, "FL", -1)
        + [Keyframe(2.5, (0, 0, h, 0, 0), label="stand"),
           Keyframe(2.5, (0.05, 0, z, 0, p), label="sit on front legs")]
        + waving(0.05, p, "RL", -1) + waving(0.05, p, "RR", +1)
        + [Keyframe(2.5, (0, 0, h, 0, 0), label="stand")])
