"""Turn a Neotics Scene into a USD stage, with the arm animated from an episode if you give one.

    python scene_to_usd.py samples/depot.scene.json depot.usda [samples/depot.episode.json]
"""
import math
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from neotics_scene import AXES, Scene, load_episode

STONE, INK, TENDON = (0.93, 0.92, 0.90), (0.07, 0.07, 0.06), (0.85, 0.32, 0.12)


def add_box(stage, path, center, size, color, collide=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def to_usd(scene: Scene, path: str, episode: dict | None = None) -> Usd.Stage:
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = f"/{scene.name}"
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, root).GetPrim())
    stage.GetRootLayer().customLayerData = {"neotics_scene": scene.name}
    UsdPhysics.Scene.Define(stage, f"{root}/physics")

    lx, ly = scene.floor
    add_box(stage, f"{root}/floor", (0, 0, -0.05), (lx, ly, 0.1), STONE)
    for box in scene.boxes:
        add_box(stage, f"{root}/{box.name}", box.center, box.size, STONE)
    for item in scene.items:
        cube = add_box(stage, f"{root}/{item.name}", item.center, item.size, TENDON)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(item.mass)

    parent = f"{root}/{scene.robot_name}"
    UsdGeom.Xform.Define(stage, parent)
    offset = scene.base
    rotations = []
    for link in scene.links:
        parent = f"{parent}/{link.joint}"
        xform = UsdGeom.Xform.Define(stage, parent)
        xform.AddTranslateOp().Set(Gf.Vec3d(*offset))
        axis = next(name for name, value in AXES.items() if value == link.axis)
        rotate = {"x": xform.AddRotateXOp, "y": xform.AddRotateYOp, "z": xform.AddRotateZOp}[axis]()
        rotate.Set(0.0)
        rotations.append(rotate)
        cylinder = UsdGeom.Cylinder.Define(stage, f"{parent}/shape")
        cylinder.GetAxisAttr().Set(UsdGeom.Tokens.z)
        cylinder.GetRadiusAttr().Set(link.radius)
        cylinder.GetHeightAttr().Set(link.length)
        cylinder.GetDisplayColorAttr().Set([Gf.Vec3f(*INK)])
        cylinder.AddTranslateOp().Set(Gf.Vec3d(0, 0, link.length / 2))
        offset = (0, 0, link.length)
    tool = UsdGeom.Xform.Define(stage, f"{parent}/tool")
    tool.AddTranslateOp().Set(Gf.Vec3d(*offset))

    if episode:
        if episode["joints"] != scene.joint_names:
            raise ValueError("the episode was recorded on a different robot")
        stage.SetTimeCodesPerSecond(episode["rate_hz"])
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(len(episode["time"]) - 1)
        for frame, qpos in enumerate(episode["qpos"]):
            for rotate, angle in zip(rotations, qpos):
                rotate.Set(math.degrees(angle), Usd.TimeCode(frame))
    stage.GetRootLayer().Save()
    return stage


if __name__ == "__main__":
    motion = load_episode(sys.argv[3]) if len(sys.argv) > 3 else None
    to_usd(Scene.load(sys.argv[1]), sys.argv[2], motion)
