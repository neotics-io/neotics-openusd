import math
import sys
from pathlib import Path

import numpy as np
import pytest
from pxr import Usd, UsdGeom, UsdPhysics

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))

from neotics_scene import Scene, load_episode  # noqa: E402
from scene_to_usd import to_usd  # noqa: E402


@pytest.fixture
def scene():
    return Scene.load(KIT / "samples" / "depot.scene.json")


@pytest.fixture
def episode():
    return load_episode(KIT / "samples" / "depot.episode.json")


def rot(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    return {"z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])}[axis]


def tool_position(scene, qpos):
    """Forward kinematics by hand, so the stage is checked against something other than itself."""
    axes = ["z", "y", "y", "y"]
    position, rotation = np.array(scene.base), np.eye(3)
    for link, axis, angle in zip(scene.links, axes, qpos):
        rotation = rotation @ rot(axis, angle)
        position = position + rotation @ np.array([0, 0, link.length])
    return position


def test_stage_reopens_in_metres_and_z_up_with_every_part(scene, tmp_path):
    path = tmp_path / "depot.usda"
    to_usd(scene, str(path))
    stage = Usd.Stage.Open(str(path))
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0 and UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert stage.GetDefaultPrim().GetName() == "depot"
    for box in scene.boxes:
        prim = stage.GetPrimAtPath(f"/depot/{box.name}")
        assert prim and prim.HasAPI(UsdPhysics.CollisionAPI)
    assert stage.GetPrimAtPath("/depot/crate").HasAPI(UsdPhysics.RigidBodyAPI)


def test_floor_and_walls_have_the_right_extent(scene, tmp_path):
    path = tmp_path / "depot.usda"
    stage = to_usd(scene, str(path))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetPrimAtPath("/depot/floor")).ComputeAlignedRange()
    assert box.GetSize()[0] == pytest.approx(scene.floor[0]) and box.GetSize()[1] == pytest.approx(scene.floor[1])


@pytest.mark.parametrize("frame", [0, 60, 125, 249])
def test_animated_tool_matches_hand_kinematics(scene, episode, tmp_path, frame):
    path = tmp_path / "depot.usda"
    to_usd(scene, str(path), episode)
    stage = Usd.Stage.Open(str(path))
    assert stage.GetEndTimeCode() == len(episode["time"]) - 1 and stage.GetTimeCodesPerSecond() == episode["rate_hz"]
    joints = "/".join(scene.joint_names)
    prim = stage.GetPrimAtPath(f"/depot/neotics_arm/{joints}/tool")
    matrix = UsdGeom.XformCache(Usd.TimeCode(frame)).GetLocalToWorldTransform(prim)
    assert np.array(matrix.ExtractTranslation()) == pytest.approx(tool_position(scene, episode["qpos"][frame]), abs=1e-4)


def test_refuses_an_episode_from_another_robot(scene, episode, tmp_path):
    episode["joints"] = ["a", "b", "c", "d"]
    with pytest.raises(ValueError):
        to_usd(scene, str(tmp_path / "x.usda"), episode)
