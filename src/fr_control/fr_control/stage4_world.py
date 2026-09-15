"""
Build a Gazebo world SDF from stage4_config.yaml.

Stage 4 experiment poses and sizes live in the YAML. This writer turns
those numbers into a world file so sim.launch.py does not keep a second
copy of object / table / robot-layout geometry.
"""

from __future__ import annotations

import os
import tempfile
from typing import Any

from fr_control.inspection_poses import as_rpy, as_vec3


def write_world_sdf(config: dict[str, Any], path: str | None = None) -> str:
    """Write a world SDF and return the absolute path."""
    column = config["column"]
    table = config["table"]
    obj = config["object"]

    column_pos = as_vec3(column["initial_pose"]["position"])
    column_rpy = as_rpy(column["initial_pose"]["orientation_rpy"])
    column_size = as_vec3(column["dimensions"])

    table_pos = as_vec3(table["initial_pose"]["position"])
    table_rpy = as_rpy(table["initial_pose"]["orientation_rpy"])
    table_size = as_vec3(table["dimensions"])

    obj_pos = as_vec3(obj["initial_pose"]["position"])
    obj_rpy = as_rpy(obj["initial_pose"]["orientation_rpy"])
    obj_size = as_vec3(obj["dimensions"])

    mass = float(obj.get("mass", 0.03))
    inertia = _box_inertia(mass, obj_size)
    sdf = _WORLD_TEMPLATE.format(
        column_name=_xml_name(column.get("name", "mounting_column")),
        column_pose=_pose_txt(column_pos, column_rpy),
        column_size=_vec_txt(column_size),

        table_name=_xml_name(table.get("name", "table")),
        table_pose=_pose_txt(table_pos, table_rpy),
        table_size=_vec_txt(table_size),

        object_name=_xml_name(obj.get("name", "small_part")),
        object_pose=_pose_txt(obj_pos, obj_rpy),
        object_size=_vec_txt(obj_size),

        object_mass=f"{mass:.6g}",
        ixx=f"{inertia[0]:.8e}",
        iyy=f"{inertia[1]:.8e}",
        izz=f"{inertia[2]:.8e}",
    )
    if path is None:
        handle = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".sdf",
            prefix="stage4_world_",
            delete=False,
            encoding="utf-8",
        )
        handle.write(sdf)
        handle.close()
        path = handle.name
    else:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(sdf)
    return os.path.abspath(path)


def _box_inertia(
    mass: float,
    size: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return ixx, iyy, izz for a solid box about its center."""
    x_len, y_len, z_len = size
    return (
        mass * (y_len * y_len + z_len * z_len) / 12.0,
        mass * (x_len * x_len + z_len * z_len) / 12.0,
        mass * (x_len * x_len + y_len * y_len) / 12.0,
    )


def _pose_txt(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> str:
    """Format an SDF pose string."""
    return (
        f"{xyz[0]:.6g} {xyz[1]:.6g} {xyz[2]:.6g} "
        f"{rpy[0]:.8g} {rpy[1]:.8g} {rpy[2]:.8g}"
    )


def _vec_txt(values: tuple[float, float, float]) -> str:
    """Format an SDF size string."""
    return f"{values[0]:.6g} {values[1]:.6g} {values[2]:.6g}"


def _xml_name(name: str) -> str:
    """Reject names that would break the SDF snippet."""
    text = str(name).strip()
    if not text or any(ch in text for ch in "<>&\""):
        raise ValueError(f"非法模型名：{name}")
    return text


_WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.8">
  <!-- Generated from stage4_config.yaml. Do not edit by hand. -->
  <world name="default">
    <physics name="physics" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <dart>
        <collision_detector>bullet</collision_detector>
        <solver>
          <solver_type>pgs</solver_type>
        </solver>
      </dart>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.35 0.35 0.35 1</ambient>
      <background>0.12 0.12 0.16 1</background>
    </scene>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
        <model name="{column_name}">
      <static>true</static>

      <pose>{column_pose}</pose>

      <link name="column_link">

        <collision name="collision">
          <geometry>
            <box>
              <size>{column_size}</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>{column_size}</size>
            </box>
          </geometry>

          <material>
            <ambient>0.35 0.35 0.35 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
            <specular>0.20 0.20 0.20 1</specular>
          </material>

        </visual>

      </link>
    </model>
    <model name="{table_name}">
      <static>true</static>
      <pose>{table_pose}</pose>
      <link name="table_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{table_size}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{table_size}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.60 0.08 0.35 1</ambient>
            <diffuse>1.00 0.20 0.60 1</diffuse>
            <emissive>0.16 0.02 0.08 1</emissive>
            <specular>0.25 0.08 0.18 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <model name="{object_name}">
      <static>false</static>
      <pose>{object_pose}</pose>
      <link name="part_link">
        <inertial>
          <mass>{object_mass}</mass>
          <inertia>
            <ixx>{ixx}</ixx>
            <iyy>{iyy}</iyy>
            <izz>{izz}</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>{object_size}</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.5</mu>
                <mu2>1.5</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>1e6</kp>
                <kd>1.0</kd>
                <min_depth>0.0005</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{object_size}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.80 0.55 0.05 1</ambient>
            <diffuse>1.00 0.75 0.10 1</diffuse>
            <emissive>0.18 0.10 0.00 1</emissive>
            <specular>0.35 0.28 0.08 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""
