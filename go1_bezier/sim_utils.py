"""Helpers to load one or many Go1s from MuJoCo Menagerie."""
import os
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MENAGERIE = os.environ.get(
    "MENAGERIE_PATH", os.path.join(HERE, "..", "mujoco_menagerie"))


# Menagerie default foot friction is 0.8 (sliding). Raised slightly for less slip.
FOOT_FRICTION = 0.9


def set_foot_friction(model, mu=FOOT_FRICTION):
    """Set sliding friction of every Go1 foot geom (names FR/FL/RR/RL, any prefix)."""
    for g in range(model.ngeom):
        name = model.geom(g).name
        if name.split("/")[-1] in ("FR", "FL", "RR", "RL"):
            model.geom_friction[g, 0] = mu


def go1_xml(menagerie=DEFAULT_MENAGERIE):
    p = os.path.join(menagerie, "unitree_go1", "scene.xml")
    if not os.path.exists(p):
        raise FileNotFoundError(f"{p} not found - set MENAGERIE_PATH or clone "
                                "mujoco_menagerie next to this folder")
    return p


def load_single(menagerie=DEFAULT_MENAGERIE, foot_friction=FOOT_FRICTION):
    model = mujoco.MjModel.from_xml_path(go1_xml(menagerie))
    set_foot_friction(model, foot_friction)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    return model, data


ARENA_XML = """
<mujoco model="arena">
  <option timestep="0.002" cone="elliptic" impratio="100"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="120" elevation="-25" offwidth="1280" offheight="720"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="grid" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
      rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="grid" texture="grid" texuniform="true" texrepeat="2 2" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="SIZE SIZE 0.05" material="grid"/>
  </worldbody>
</mujoco>
"""


def make_arena(size=50.0):
    """Empty world (floor, light, sky) as an MjSpec; add obstacles to spec.worldbody."""
    return mujoco.MjSpec.from_string(ARENA_XML.replace("SIZE", str(size)))


def load_team(positions, menagerie=DEFAULT_MENAGERIE, spec=None, visual_meshes=True,
              foot_friction=FOOT_FRICTION):
    """Build one scene with len(positions) Go1s named go1_0/, go1_1/, ...

    positions: list of (x, y, yaw).  spec: optional MjSpec arena (obstacles etc.).
    All robots share robot 0's mesh assets (otherwise memory grows ~130 MB/robot).
    visual_meshes=False drops the STL visuals entirely (fastest, headless runs).
    """
    spec = make_arena() if spec is None else spec
    robot_xml = os.path.join(menagerie, "unitree_go1", "go1.xml")
    for i, (x, y, yaw) in enumerate(positions):
        robot = mujoco.MjSpec.from_file(robot_xml)
        for k in list(robot.keys):
            robot.delete(k)
        frame = spec.worldbody.add_frame(pos=[x, y, 0.0],
                                         quat=[np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        frame.attach_body(robot.body("trunk"), f"go1_{i}/", "")
    # share meshes: point every visual geom at go1_0/'s meshes, drop the copies
    for g in list(spec.geoms):
        if g.type != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if not visual_meshes:
            spec.delete(g)
            continue
        pref, _, base = g.meshname.partition("/")
        g.meshname = f"go1_0/{base}"
    for msh in list(spec.meshes):
        if not visual_meshes or not msh.name.startswith("go1_0/"):
            spec.delete(msh)
    model = spec.compile()
    set_foot_friction(model, foot_friction)
    data = mujoco.MjData(model)
    for i in range(len(positions)):
        pref = f"go1_{i}/"
        for leg in ("FR", "FL", "RR", "RL"):
            for jn, val in (("hip", 0.0), ("thigh", 0.9), ("calf", -1.8)):
                data.qpos[model.joint(f"{pref}{leg}_{jn}_joint").qposadr[0]] = val
                data.ctrl[model.actuator(f"{pref}{leg}_{jn}").id] = val
        tj = model.body(f"{pref}trunk").jntadr[0]
        data.qpos[model.jnt_qposadr[tj] + 2] = 0.28
    mujoco.mj_forward(model, data)
    return model, data
