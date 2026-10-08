"""Velocity-command interface for one Go1 in MuJoCo, using the Bezier trot gait.

    ctrl = Go1BezierController(model, data, prefix="")
    ctrl.set_command(vx, vy, wz)      # body-frame m/s, m/s, rad/s
    ctrl.step()                       # call once per physics step, before mj_step
    ctrl.phase, ctrl.contact_schedule # gait state for the swarm layer

`prefix` lets many robots share one model (names like "go1_3/FR_thigh").
"""
import numpy as np
import mujoco
from .kinematics import HIP_OFFSETS, LEGS, body_ik
from .gait import BezierTrotGait

STAND_HEIGHT = 0.27          # hip-to-foot-centre height (m)
FOOT_Y = 0.08 + 0.04675      # nominal lateral foot position (m)


def nominal_feet(height=STAND_HEIGHT, x_shift=0.0):
    feet = HIP_OFFSETS.copy()
    feet[:, 1] = np.sign(feet[:, 1]) * FOOT_Y
    feet[:, 0] += x_shift
    feet[:, 2] = -height
    return feet


class Go1BezierController:
    """Bezier trot + PI velocity loop + Raibert foot placement.

    period, duty   : gait cycle time (s) and stance fraction (0.5 = trot)
    clearance      : swing foot height (m);  height: stand height (m)
    k_i, k_p, i_max: PI correction on the commanded body velocity (removes the
                     drift caused by the soft PD legs complying under load)
    k_raibert      : foot-placement feedback gain (s) - keeps the trot balanced
                     under abrupt command changes
    decimation     : gait/IK runs every N physics steps (2 -> 250 Hz)
    acc_max        : command slew-rate limit (m/s^2, m/s^2, rad/s^2); None = off.
                     Without it, a sudden reversal (e.g. -0.3 -> +0.5 m/s) makes
                     every stance foot reverse instantly and the robot hops.
    """
    def __init__(self, model, data, prefix="", period=0.36, duty=0.5,
                 clearance=0.07, height=STAND_HEIGHT, x_shift=0.0,
                 v_max=(0.8, 0.4), w_max=1.5, k_i=1.5, k_p=0.0, i_max=0.3, k_raibert=0.1,
                 decimation=2, acc_max=(1.5, 1.0, 3.0)):
        self.m, self.d = model, data
        self.prefix = prefix
        self.act_ids = np.array([
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{prefix}{leg}_{j}")
            for leg in LEGS for j in ("hip", "thigh", "calf")])
        assert (self.act_ids >= 0).all(), "actuator names not found; check prefix"
        trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}trunk")
        jnt = model.body_jntadr[trunk]
        self.qadr = model.jnt_qposadr[jnt]
        self.vadr = model.jnt_dofadr[jnt]
        self.gait = BezierTrotGait(nominal_feet(height, x_shift), period, duty, clearance)
        self.v_max = np.array(v_max)
        self.w_max = w_max
        # PI velocity loop: the soft PD legs comply under load (~1 cm per step),
        # which makes the open-loop gait drift. The integral term removes that bias.
        self.k_i, self.k_p, self.i_max = k_i, k_p, i_max
        self.v_int = np.zeros(3)
        self.k_raibert = k_raibert
        self.decimation = decimation   # gait/IK update every N physics steps
        self._tick = 0
        self.cmd = np.zeros(3)          # what the gait is tracking (rate-limited)
        self.cmd_target = np.zeros(3)   # what was requested
        self.acc_max = None if acc_max is None else np.asarray(acc_max, float)
        self.stand = True
        self.feet_target = self.gait.nominal.copy()
        self.height_target, self.height_rate = height, 0.1

    # ---- interface for the swarm layer -------------------------------
    def set_command(self, vx, vy, wz):
        self.cmd_target = np.array([np.clip(vx, -self.v_max[0], self.v_max[0]),
                                    np.clip(vy, -self.v_max[1], self.v_max[1]),
                                    np.clip(wz, -self.w_max, self.w_max)])
        if self.acc_max is None:
            self.cmd = self.cmd_target.copy()
        self.stand = False

    def set_height(self, height, rate=0.1):
        """Change stand height while walking (hip-to-foot, m; walkable ~0.16-0.34),
        ramped at `rate` m/s."""
        self.height_target = float(np.clip(height, 0.12, 0.36))
        self.height_rate = rate

    def set_stand(self):
        self.stand = True

    @property
    def phase(self):
        return self.gait.phi

    @property
    def contact_schedule(self):
        return self.gait.contact_schedule()

    # ---- state -------------------------------------------------------
    def base_pose(self):
        q = self.d.qpos[self.qadr:self.qadr + 7]
        return q[:3].copy(), q[3:7].copy()

    def yaw(self):
        w, x, y, z = self.d.qpos[self.qadr + 3:self.qadr + 7]
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def roll_pitch(self):
        w, x, y, z = self.d.qpos[self.qadr + 3:self.qadr + 7]
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
        return roll, pitch

    def body_velocity(self):
        """(vx, vy, wz) in the yaw-aligned body frame."""
        v_world = self.d.qvel[self.vadr:self.vadr + 3]
        wz = self.d.qvel[self.vadr + 5]          # free-joint angular vel is in body frame
        c, s = np.cos(self.yaw()), np.sin(self.yaw())
        return np.array([c * v_world[0] + s * v_world[1],
                         -s * v_world[0] + c * v_world[1], wz])

    # ---- control step ------------------------------------------------
    def step(self):
        """Call once per physics step (before mj_step)."""
        self._tick += 1
        if (self._tick - 1) % self.decimation:
            return
        dt = self.m.opt.timestep * self.decimation
        z = -self.gait.nominal[0, 2]
        if z != self.height_target:
            z += np.clip(self.height_target - z, -self.height_rate * dt, self.height_rate * dt)
            self.gait.nominal[:, 2] = -z
        if self.acc_max is not None:
            self.cmd += np.clip(self.cmd_target - self.cmd, -self.acc_max * dt, self.acc_max * dt)
        u = self.cmd.copy()
        if not self.stand and (self.k_i > 0 or self.k_p > 0):
            err = self.cmd - self.body_velocity()
            self.v_int = np.clip(self.v_int + err * dt, -self.i_max / max(self.k_i, 1e-9),
                                 self.i_max / max(self.k_i, 1e-9))
            u = self.cmd + self.k_p * err + self.k_i * self.v_int
        self.u = u
        vm = self.body_velocity()
        self.feet_target = self.gait.step(dt, u[:2], u[2], stand=self.stand,
                                          v_meas=vm[:2], w_meas=vm[2], k_raibert=self.k_raibert)
        self.d.ctrl[self.act_ids] = body_ik(self.feet_target)
