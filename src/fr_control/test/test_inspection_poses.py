"""Unit tests for stage 4 object / TCP pose math."""

from __future__ import annotations

import math
import os

from geometry_msgs.msg import Pose

from fr_control.gazebo_world_pose import parse_named_pose
from fr_control.grasp_poses import pose_inverse, pose_multiply, rpy_to_xyzw
from fr_control.inspection_poses import (
    angle_between_deg,
    as_vec3,
    frame_from_z_and_x,
    object_in_tcp,
    object_pose_for_face,
    rotate_pose_vector,
    tcp_pose_from_object,
)
from fr_control.moveit_arm import make_pose
from fr_control.stage4_config import (
    face_order,
    grasp_tcp_xyzw,
    joint_positions,
    load_yaml,
    validate_inspection_vectors,
)
from fr_control.stage4_world import write_world_sdf


def _config_path() -> str:
    """Return the source-tree stage4 YAML path."""
    return os.path.join(
        os.path.dirname(__file__),
        "..",
        "config",
        "stage4_config.yaml",
    )


def test_yaml_loads_and_joints_match_srdf():
    """Home joints must be the real FR3 names j1..j6."""
    config = load_yaml(_config_path())
    validate_inspection_vectors(config)
    joints = joint_positions(config)
    assert len(joints) == 6
    assert list(config["robot"]["initial_joint_positions"]) == [
        "j1",
        "j2",
        "j3",
        "j4",
        "j5",
        "j6",
    ]
    assert face_order(config) == ["face1", "face2", "face3"]
    world = write_world_sdf(config)
    text = open(world, encoding="utf-8").read()
    assert 'name="small_part"' in text
    assert "0.02 0.04 0.02" in text


def test_grasp_orientation_matches_stage3_rpy():
    """Approach z-up pair must equal approach_rpy=(pi, 0, 0)."""
    config = load_yaml(_config_path())
    computed = grasp_tcp_xyzw(config["grasp"])
    expected = rpy_to_xyzw(math.pi, 0.0, 0.0)
    dot = abs(sum(a * b for a, b in zip(computed, expected)))
    assert dot > 0.999


def test_three_faces_share_p1_and_aim_d1():
    """Object poses keep P1 and map each face normal onto D1."""
    config = load_yaml(_config_path())
    inspection = config["inspection"]
    p1 = as_vec3(inspection["position"])
    direction = as_vec3(inspection["direction"])
    up = as_vec3(inspection["up_direction"])
    poses = []
    for name in face_order(config):
        face = inspection["faces"][name]
        pose = object_pose_for_face(
            p1,
            face_normal=as_vec3(face["normal_in_object"]),
            face_up=as_vec3(face["up_in_object"]),
            inspection_direction=direction,
            inspection_up=up,
        )
        poses.append(pose)
        assert math.isclose(pose.position.x, p1[0], abs_tol=1e-9)
        assert math.isclose(pose.position.y, p1[1], abs_tol=1e-9)
        assert math.isclose(pose.position.z, p1[2], abs_tol=1e-9)
        actual = rotate_pose_vector(pose, as_vec3(face["normal_in_object"]))
        assert angle_between_deg(actual, direction) < 1e-4


def test_tcp_from_object_roundtrip():
    """Target TCP composed with T_tcp_object must recover the object pose."""
    object_pose = object_pose_for_face(
        (0.36, 0.08, 0.30),
        face_normal=(0.0, 0.0, 1.0),
        face_up=(0.0, -1.0, 0.0),
        inspection_direction=(0.0, 1.0, 0.0),
        inspection_up=(0.0, 0.0, 1.0),
    )
    rel = make_pose((0.0, 0.0, 0.041), rpy_to_xyzw(math.pi, 0.0, 0.0))
    tcp = tcp_pose_from_object(object_pose, rel)
    recovered = pose_multiply(tcp, rel)
    assert abs(recovered.position.x - object_pose.position.x) < 1e-9
    assert abs(recovered.position.y - object_pose.position.y) < 1e-9
    assert abs(recovered.position.z - object_pose.position.z) < 1e-9
    again = object_in_tcp(tcp, object_pose)
    assert abs(again.position.z - rel.position.z) < 1e-9


def test_frame_from_z_and_x_is_right_handed():
    """TCP +z down and +x forward must produce TCP +y = world -Y."""
    xyzw = frame_from_z_and_x((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
    pose = Pose()
    pose.orientation.x = xyzw[0]
    pose.orientation.y = xyzw[1]
    pose.orientation.z = xyzw[2]
    pose.orientation.w = xyzw[3]
    y_axis = rotate_pose_vector(pose, (0.0, 1.0, 0.0))
    assert angle_between_deg(y_axis, (0.0, -1.0, 0.0)) < 1e-4


def test_parse_named_pose_reads_orientation():
    """Ignition pose/info text must yield a full Pose, not just XYZ."""
    text = """
pose {
  name: "table"
  position { x: 1 y: 2 z: 3 }
  orientation { x: 0 y: 0 z: 0 w: 1 }
}
pose {
  name: "small_part"
  id: 12
  position {
    x: 0.5
    y: -0.01
    z: 0.085
  }
  orientation {
    x: 0
    y: 0.7071
    z: 0
    w: 0.7071
  }
}
"""
    pose = parse_named_pose(text, "small_part")
    assert pose is not None
    assert abs(pose.position.x - 0.5) < 1e-9
    assert abs(pose.position.y + 0.01) < 1e-9
    assert abs(pose.orientation.w - 0.7071) < 1e-4
    assert parse_named_pose(text, "missing") is None


def test_inverse_formula_matches_spec():
    """T_base_tcp = T_base_object * inverse(T_tcp_object)."""
    t_base_tcp = make_pose((0.4, 0.1, 0.2), rpy_to_xyzw(math.pi, 0.0, 0.1))
    t_base_object = make_pose((0.4, 0.1, 0.16), rpy_to_xyzw(0.0, 0.0, 0.1))
    t_tcp_object = pose_multiply(pose_inverse(t_base_tcp), t_base_object)
    rebuilt = pose_multiply(t_base_object, pose_inverse(t_tcp_object))
    assert abs(rebuilt.position.x - t_base_tcp.position.x) < 1e-9
    assert abs(rebuilt.position.y - t_base_tcp.position.y) < 1e-9
    assert abs(rebuilt.position.z - t_base_tcp.position.z) < 1e-9
