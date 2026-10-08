"""Watch a swarm scenario: ORCA + formation layer -> commander -> Go1 Bezier trot.

macOS:  mjpython examples/swarm_demo.py --scenario formation6_gate --config predictive:phase+limiter:q
        scenarios: swap4, swap8, formation9_pillars, formation6_gate
        configs:   layer:strategy[:q]  e.g. reactive:limiter, predictive:phase+gaslc_equal:q
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer
from go1_bezier import load_team, make_arena, Go1BezierController, CapabilityMap
from go1_bezier.swarm import SwarmAgent, goal_velocity, elastic_scale, queue_responsibility
from go1_bezier import PushEnvelope
from go1_bezier.gasl import make_commander
from go1_bezier.scenarios import SCENARIOS, R_ROBOT, add_obstacles
from swarm_eval import MAP, ENV, ttc, parse_config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="swap8", choices=list(SCENARIOS))
    ap.add_argument("--config", default="predictive:phase+limiter:q")
    ap.add_argument("--lookahead", type=float, default=0.36)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    layer, strategy, queue = parse_config(a.config)
    look = a.lookahead
    sc = SCENARIOS[a.scenario]()
    rng = np.random.default_rng(a.seed)
    m, d = load_team(sc["starts"], spec=add_obstacles(make_arena(), sc["obstacles"]))
    N, dt = len(sc["starts"]), m.opt.timestep
    cmap = CapabilityMap.load(MAP)
    ctrls = [Go1BezierController(m, d, prefix=f"go1_{i}/") for i in range(N)]
    for c in ctrls:
        c.gait.phi = rng.uniform()
    env = PushEnvelope.load(ENV) if os.path.exists(ENV) else None
    cache = {}
    coms = [make_commander(c, strategy, cmap, env, cache) for c in ctrls]
    agents = [SwarmAgent(radius=R_ROBOT) for _ in range(N)]
    trunk = [m.body(f"go1_{i}/trunk").id for i in range(N)]
    vel = np.zeros((N, 2))
    tick = int(round(0.1 / dt))
    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.lookat[:] = [7 if sc["plan"] is not None else 0, 0, 0]
        viewer.cam.distance = 14 if sc["plan"] is not None else 10
        viewer.cam.elevation = -55
        for _ in range(250):
            for cm_ in coms:
                cm_.step()
            mujoco.mj_step(m, d)
        t0, k = d.time, 0
        while viewer.is_running() and d.time - t0 < sc["T"]:
            w0 = time.time()
            for _ in range(10):
                t = d.time - t0
                pos = np.array([d.xpos[b][:2] for b in trunk])
                if k % tick == 0:
                    for i in range(N):
                        yaw = ctrls[i].yaw()
                        if sc["plan"] is not None:
                            plan = sc["plan"]
                            lam = elastic_scale(pos[i], plan.heading, sc["obstacles"])
                            target, heading, in_bn = plan.slot(i, t, lam)[0], plan.heading, lam < 0.6
                            pref_now = (lambda q, i=i, lam=lam: plan.preferred_velocity(i, q, t, lam))
                            pref_ahead = (lambda q, i=i, lam=lam: plan.preferred_velocity(i, q, t + look, lam))
                        else:
                            target = np.asarray(sc["goals"][i])
                            heading, in_bn = np.arctan2(*(target - pos[i])[::-1]), False
                            pref_now = pref_ahead = (lambda q, g=target: goal_velocity(q, g))
                        far = np.linalg.norm(target - pos[i]) > 1.0
                        nbs = [(pos[j], vel[j], R_ROBOT) for j in range(N) if j != i]
                        if queue:
                            nbs = queue_responsibility(pos[i], heading, nbs, in_bn)
                        Rz = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
                        v_cmd = Rz @ ctrls[i].cmd_target[:2]
                        urgent = ttc(pos[i], v_cmd, [n[:3] for n in nbs] +
                                     [(q, np.zeros(2), r) for q, r in sc["obstacles"]], R_ROBOT) < 0.8
                        if layer == "predictive" and not urgent:
                            v_w = agents[i].predictive(pos[i], v_cmd, pref_ahead, nbs, sc["obstacles"], look, far)
                            deadline = look
                        else:
                            v_w = agents[i].velocity(pos[i], pref_now(pos[i]), nbs, sc["obstacles"], far)
                            deadline = None
                        coms[i].request(*agents[i].body_command(v_w, yaw), urgent=bool(urgent), deadline=deadline)
                for cm_ in coms:
                    cm_.step()
                mujoco.mj_step(m, d)
                new = np.array([d.xpos[b][:2] for b in trunk])
                vel = 0.987 * vel + 0.013 * (new - pos) / dt
                k += 1
            viewer.sync()
            time.sleep(max(0.0, 10 * dt - (time.time() - w0)))


if __name__ == "__main__":
    main()
