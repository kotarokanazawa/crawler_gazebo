#!/usr/bin/env python3
"""Generate a Gazebo-ready crawler URDF from YAML or a small GUI.

The generated model intentionally uses only URDF primitive geometry
(`box` and `cylinder`) for visuals and collisions.  Joint and transmission
names match the existing crawler_gazebo controller configuration.
"""

from __future__ import annotations

import argparse
import copy
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple
from xml.sax.saxutils import escape

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config" / "robot" / "default.yaml"
DEFAULT_OUTPUT = PACKAGE_ROOT / "urdf" / "default_crawler.urdf"


DEFAULT_DATA: Dict[str, Any] = {
    "robot_config": {
        "general": {
            "robot_name": "crawler",
            "nonholonomic": True,
            "flipper_front": True,
            "flipper_back": True,
            "gazebo_ros_control": True,
            "gazebo_drive_plugin": False,
            "imu": True,
        },
        "geometry": {
            "base_length": 0.45,
            "base_width": 0.20,
            "base_height": 0.11,
            "body_length": 0.45,
            "body_width": 0.10,
            "body_height": 0.164,
            "body_gap": 0.24,
            "flipper_length": 0.29,
            "flipper_width": 0.025,
            "flipper_position": 0.225,
            "flipper_y_offset": 0.23,
            "wheel_radius": 0.082,
            "track_collision_margin": 0.03,
        },
        "mass": {
            "base": 5.0,
            "body_track": 30.0,
            "body_sprocket": 0.2,
            "flipper_arm": 1.0,
            "flipper_track": 2.0,
            "flipper_sprocket": 0.2,
        },
        "joints": {
            "flipper_lower": -2.35619449,
            "flipper_upper": 2.35619449,
            "flipper_torque_max": 20000.0,
            "flipper_effort": 20000.0,
            "flipper_velocity": 1.0,
            "flipper_damping": 10.0,
            "flipper_friction": 0.05,
            "flipper_position_kp": 1200.0,
            "flipper_position_ki": 0.0,
            "flipper_position_kd": 100.0,
            "flipper_position_max_integral_error": 1000.0,
            "sprocket_effort": 1000.0,
            "sprocket_velocity": 100.0,
        },
    "continuous_track": {
            "enabled": True,
            "pitch_diameter": 0.24,
            "elements_per_round": 20,
            "belt_element_length": 0.02,
            "belt_thickness": 0.02,
            "grouser_height": 0.01,
            "grouser_shape": "rectangle",
            "mu": 0.4,
            "min_depth": 0.0005,
            "contact_kp": 120000.0,
            "contact_kd": 2500.0,
            "contact_max_vel": 0.01,
            "implicit_spring_damper": 1,
        },
        "output": {
            "urdf": "../../urdf/default_crawler.urdf",
        },
    }
}


GUI_FIELDS: Tuple[Tuple[str, str], ...] = (
    ("general.robot_name", "Robot name"),
    ("general.flipper_front", "Front flippers"),
    ("general.flipper_back", "Rear flippers"),
    ("general.gazebo_ros_control", "gazebo_ros_control"),
    ("general.imu", "IMU sensor"),
    ("geometry.base_length", "Base length [m]"),
    ("geometry.base_width", "Base width [m]"),
    ("geometry.base_height", "Base height [m]"),
    ("geometry.body_length", "Body track length [m]"),
    ("geometry.body_width", "Body track width [m]"),
    ("geometry.body_height", "Body track height [m] (legacy, auto=2r)"),
    ("geometry.body_gap", "Body track center gap [m]"),
    ("geometry.flipper_length", "Flipper length [m]"),
    ("geometry.flipper_width", "Flipper width [m]"),
    ("geometry.flipper_position", "Flipper x offset [m] (legacy)"),
    ("geometry.flipper_y_offset", "Flipper y offset [m]"),
    ("geometry.wheel_radius", "Pulley radius [m]"),
    ("geometry.track_collision_margin", "Track collision margin [m]"),
    ("mass.base", "Base mass [kg]"),
    ("mass.body_track", "Body track mass [kg]"),
    ("mass.body_sprocket", "Body pulley mass [kg]"),
    ("mass.flipper_arm", "Flipper arm mass [kg]"),
    ("mass.flipper_track", "Flipper track mass [kg]"),
    ("mass.flipper_sprocket", "Flipper pulley mass [kg]"),
    ("joints.flipper_lower", "Flipper lower [rad]"),
    ("joints.flipper_upper", "Flipper upper [rad]"),
    ("joints.flipper_torque_max", "Flipper max torque"),
    ("joints.flipper_velocity", "Flipper velocity"),
    ("joints.flipper_damping", "Flipper damping"),
    ("joints.flipper_friction", "Flipper friction"),
    ("joints.flipper_position_kp", "Flipper position Kp"),
    ("joints.flipper_position_ki", "Flipper position Ki"),
    ("joints.flipper_position_kd", "Flipper position Kd"),
    ("joints.flipper_position_max_integral_error", "Flipper max integral error"),
    ("joints.sprocket_effort", "Sprocket effort"),
    ("joints.sprocket_velocity", "Sprocket velocity"),
    ("continuous_track.enabled", "ContinuousTrack enabled"),
    ("continuous_track.pitch_diameter", "Track pitch diameter [m]"),
    ("continuous_track.elements_per_round", "Track elements / round"),
    ("continuous_track.belt_element_length", "Belt element length [m]"),
    ("continuous_track.belt_thickness", "Belt thickness [m]"),
    ("continuous_track.grouser_height", "Grouser height [m]"),
    ("continuous_track.grouser_shape", "Grouser shape"),
    ("continuous_track.mu", "Track friction mu"),
    ("continuous_track.min_depth", "Track contact min_depth"),
    ("continuous_track.contact_kp", "Track contact Kp"),
    ("continuous_track.contact_kd", "Track contact Kd"),
    ("continuous_track.contact_max_vel", "Track contact max vel"),
    ("continuous_track.implicit_spring_damper", "Implicit spring damper"),
    ("output.urdf", "Output URDF"),
)


def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return copy.deepcopy(DEFAULT_DATA)
    with path.open("r", encoding="utf-8") as f:
        loaded = yaml.safe_load(f) or {}
    return deep_merge(DEFAULT_DATA, loaded)


def save_config(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def config_root(data: Dict[str, Any]) -> Dict[str, Any]:
    return data.setdefault("robot_config", {})


def get_path(data: Dict[str, Any], dotted: str) -> Any:
    node: Any = config_root(data)
    for part in dotted.split("."):
        node = node[part]
    return node


def set_path(data: Dict[str, Any], dotted: str, value: Any) -> None:
    node = config_root(data)
    parts = dotted.split(".")
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value


def parse_value(text: str, current: Any) -> Any:
    if isinstance(current, bool):
        return text.strip().lower() in {"1", "true", "yes", "on"}
    if isinstance(current, int) and not isinstance(current, bool):
        return int(text)
    if isinstance(current, float):
        return float(text)
    return text


def resolve_output_path(config_path: Path, data: Dict[str, Any], override: str | None) -> Path:
    output = override if override else str(get_path(data, "output.urdf"))
    path = Path(output).expanduser()
    if path.is_absolute():
        return path
    return (config_path.parent / path).resolve()


def fmt(value: float) -> str:
    if abs(value) < 1e-12:
        value = 0.0
    return f"{value:.9g}"


def inertia_box(mass: float, lx: float, ly: float, lz: float) -> str:
    ixx = mass * (ly * ly + lz * lz) / 12.0
    iyy = mass * (lx * lx + lz * lz) / 12.0
    izz = mass * (lx * lx + ly * ly) / 12.0
    return (
        f'<inertia ixx="{fmt(ixx)}" ixy="0" ixz="0" '
        f'iyy="{fmt(iyy)}" iyz="0" izz="{fmt(izz)}"/>'
    )


def inertia_cylinder_y(mass: float, radius: float, length: float) -> str:
    i_axis = 0.5 * mass * radius * radius
    i_side = mass * (3.0 * radius * radius + length * length) / 12.0
    return (
        f'<inertia ixx="{fmt(i_side)}" ixy="0" ixz="0" '
        f'iyy="{fmt(i_axis)}" iyz="0" izz="{fmt(i_side)}"/>'
    )


def sdf_inertia_box(mass: float, lx: float, ly: float, lz: float) -> str:
    ixx = mass * (ly * ly + lz * lz) / 12.0
    iyy = mass * (lx * lx + lz * lz) / 12.0
    izz = mass * (lx * lx + ly * ly) / 12.0
    return f"""
        <inertia>
          <ixx>{fmt(ixx)}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{fmt(iyy)}</iyy><iyz>0</iyz><izz>{fmt(izz)}</izz>
        </inertia>"""


def sdf_inertia_cylinder_y(mass: float, radius: float, length: float) -> str:
    i_axis = 0.5 * mass * radius * radius
    i_side = mass * (3.0 * radius * radius + length * length) / 12.0
    return f"""
        <inertia>
          <ixx>{fmt(i_side)}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{fmt(i_axis)}</iyy><iyz>0</iyz><izz>{fmt(i_side)}</izz>
        </inertia>"""


def material_xml() -> str:
    return """
  <material name="base_gray"><color rgba="0.45 0.47 0.50 1"/></material>
  <material name="track_green"><color rgba="0.224 0.72 0.556 1"/></material>
  <material name="flipper_blue"><color rgba="0.008 0.416 0.7 1"/></material>
  <material name="pulley_dark"><color rgba="0.10 0.10 0.10 1"/></material>
"""


def box_link(name: str, size: Tuple[float, float, float], mass: float, material: str,
             collision: bool = True) -> str:
    lx, ly, lz = size
    collision_xml = ""
    if collision:
        collision_xml = f"""
    <collision>
      <geometry><box size="{fmt(lx)} {fmt(ly)} {fmt(lz)}"/></geometry>
    </collision>"""
    return f"""
  <link name="{name}">
    <visual>
      <geometry><box size="{fmt(lx)} {fmt(ly)} {fmt(lz)}"/></geometry>
      <material name="{material}"/>
    </visual>
{collision_xml}
    <inertial>
      <mass value="{fmt(mass)}"/>
      {inertia_box(mass, lx, ly, lz)}
    </inertial>
  </link>
"""


def cylinder_link(name: str, radius: float, length: float, mass: float, material: str,
                  collision: bool = True, collision_radius: float = None) -> str:
    collision_xml = ""
    if collision:
        effective_collision_radius = radius if collision_radius is None else collision_radius
        collision_xml = f"""
    <collision>
      <origin xyz="0 0 0" rpy="{fmt(math.pi / 2.0)} 0 0"/>
      <geometry><cylinder radius="{fmt(effective_collision_radius)}" length="{fmt(length)}"/></geometry>
    </collision>"""
    return f"""
  <link name="{name}">
    <visual>
      <origin xyz="0 0 0" rpy="{fmt(math.pi / 2.0)} 0 0"/>
      <geometry><cylinder radius="{fmt(radius)}" length="{fmt(length)}"/></geometry>
      <material name="{material}"/>
    </visual>
{collision_xml}
    <inertial>
      <mass value="{fmt(mass)}"/>
      {inertia_cylinder_y(mass, radius, length)}
    </inertial>
  </link>
"""


def fixed_joint(name: str, parent: str, child: str, xyz: Tuple[float, float, float],
                rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> str:
    return f"""
  <joint name="{name}" type="fixed">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{fmt(xyz[0])} {fmt(xyz[1])} {fmt(xyz[2])}" rpy="{fmt(rpy[0])} {fmt(rpy[1])} {fmt(rpy[2])}"/>
  </joint>
"""


def revolute_joint(name: str, parent: str, child: str, xyz: Tuple[float, float, float],
                   axis: Tuple[float, float, float], lower: float, upper: float,
                   effort: float, velocity: float,
                   rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                   damping: float = 0.0, friction: float = 0.0) -> str:
    dynamics_xml = ""
    if damping > 0.0 or friction > 0.0:
        dynamics_xml = f"""
    <dynamics damping="{fmt(damping)}" friction="{fmt(friction)}"/>"""
    return f"""
  <joint name="{name}" type="revolute">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{fmt(xyz[0])} {fmt(xyz[1])} {fmt(xyz[2])}" rpy="{fmt(rpy[0])} {fmt(rpy[1])} {fmt(rpy[2])}"/>
    <axis xyz="{fmt(axis[0])} {fmt(axis[1])} {fmt(axis[2])}"/>
    <limit lower="{fmt(lower)}" upper="{fmt(upper)}" effort="{fmt(effort)}" velocity="{fmt(velocity)}"/>
{dynamics_xml}
  </joint>
"""


def continuous_joint(name: str, parent: str, child: str, xyz: Tuple[float, float, float],
                     axis: Tuple[float, float, float], effort: float, velocity: float) -> str:
    return f"""
  <joint name="{name}" type="continuous">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{fmt(xyz[0])} {fmt(xyz[1])} {fmt(xyz[2])}" rpy="0 0 0"/>
    <axis xyz="{fmt(axis[0])} {fmt(axis[1])} {fmt(axis[2])}"/>
    <limit effort="{fmt(effort)}" velocity="{fmt(velocity)}"/>
  </joint>
"""


def transmission(name: str, joint_name: str, interface: str) -> str:
    _ = (name, joint_name, interface)
    return ""


def gazebo_plugin_xml(enabled: bool) -> str:
    if not enabled:
        return ""
    return """
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>__CRAWLER_ROS2_CONTROL_YAML__</parameters>
    </plugin>
  </gazebo>
"""


def ros2_control_joint_xml(name: str, command_interface: str,
                           params: Dict[str, Any] | None = None) -> str:
    param_xml = ""
    for key, value in (params or {}).items():
        param_xml += f'\n      <param name="{key}">{fmt(float(value))}</param>'
    return f"""
    <joint name="{name}">
{param_xml}
      <command_interface name="{command_interface}"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>"""


def ros2_control_xml(enabled: bool, general: Dict[str, Any], joints_config: Dict[str, Any]) -> str:
    if not enabled:
        return ""

    velocity_joints = [
        "sprocket_axle_left",
        "sprocket_axle_right",
    ]
    position_joints = []
    for side in ("left", "right"):
        for where, _front_sign in enabled_flippers(general):
            velocity_joints.append(f"flipper_sprocket_axle_{side}_{where}")
            position_joints.append(f"joint_{side}_{where}")

    joints = [ros2_control_joint_xml(name, "velocity") for name in velocity_joints]
    position_pid_params = {
        "pos_kp": float(joints_config.get("flipper_position_kp", 140.0)),
        "pos_ki": float(joints_config.get("flipper_position_ki", 0.0)),
        "pos_kd": float(joints_config.get("flipper_position_kd", 28.0)),
        "pos_max_integral_error": float(
            joints_config.get("flipper_position_max_integral_error", 1000.0)
        ),
    }
    joints.extend(
        ros2_control_joint_xml(name, "position_pid", position_pid_params)
        for name in position_joints
    )

    return f"""
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
{''.join(joints)}
  </ros2_control>
"""


def imu_xml(enabled: bool) -> str:
    if not enabled:
        return ""
    return """
  <link name="imu_frame">
    <visual>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
      <material name="base_gray"/>
    </visual>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_frame"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <gazebo reference="imu_frame">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <topicName>/imu_data/data</topicName>
        <bodyName>remote_robot/imu_frame</bodyName>
        <updateRateHZ>100.0</updateRateHZ>
        <gaussianNoise>0.0</gaussianNoise>
        <xyzOffset>0 0 0</xyzOffset>
        <rpyOffset>0 0 0</rpyOffset>
        <frameName>remote_robot/imu_frame</frameName>
        <initialOrientationAsReference>false</initialOrientationAsReference>
      </plugin>
      <pose>0 0 0 0 0 0</pose>
    </sensor>
  </gazebo>
"""


def gazebo_material_xml(color: Tuple[float, float, float, float]) -> str:
    return f"""
          <material>
            <ambient>{fmt(color[0])} {fmt(color[1])} {fmt(color[2])} {fmt(color[3])}</ambient>
            <diffuse>{fmt(color[0])} {fmt(color[1])} {fmt(color[2])} {fmt(color[3])}</diffuse>
            <specular>0 0 0 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>"""


def track_contact_settings(params: Dict[str, Any]) -> Tuple[float, float, float, float]:
    return (
        float(params.get("min_depth", 0.0005)),
        float(params.get("contact_kp", 120000.0)),
        float(params.get("contact_kd", 2500.0)),
        float(params.get("contact_max_vel", 0.01)),
    )


def gazebo_box_collision_xml(name: str, pose: Tuple[float, float, float, float, float, float],
                             size: Tuple[float, float, float], mu: float,
                             min_depth: float, contact_kp: float,
                             contact_kd: float, contact_max_vel: float) -> str:
    return f"""
      <collision name="{name}">
        <pose>{fmt(pose[0])} {fmt(pose[1])} {fmt(pose[2])} {fmt(pose[3])} {fmt(pose[4])} {fmt(pose[5])}</pose>
        <geometry><box><size>{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}</size></box></geometry>
        <max_contacts>10</max_contacts>
        <surface>
          <bounce><restitution_coefficient>0</restitution_coefficient><threshold>100000</threshold></bounce>
          <friction><torsional><coefficient>1</coefficient><use_patch_radius>true</use_patch_radius><patch_radius>0</patch_radius><surface_radius>0</surface_radius><ode><slip>0</slip></ode></torsional><ode><mu>{fmt(mu)}</mu><mu2>{fmt(mu)}</mu2></ode></friction>
          <contact><ode><kp>{fmt(contact_kp)}</kp><kd>{fmt(contact_kd)}</kd><max_vel>{fmt(contact_max_vel)}</max_vel><min_depth>{fmt(min_depth)}</min_depth></ode></contact>
        </surface>
      </collision>"""


def gazebo_box_visual_xml(name: str, pose: Tuple[float, float, float, float, float, float],
                          size: Tuple[float, float, float],
                          color: Tuple[float, float, float, float]) -> str:
    return f"""
      <visual name="{name}">
        <pose>{fmt(pose[0])} {fmt(pose[1])} {fmt(pose[2])} {fmt(pose[3])} {fmt(pose[4])} {fmt(pose[5])}</pose>
        <geometry><box><size>{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}</size></box></geometry>
        {gazebo_material_xml(color)}
      </visual>"""


def gazebo_cylinder_collision_xml(name: str,
                                  pose: Tuple[float, float, float, float, float, float],
                                  radius: float, length: float, mu: float,
                                  min_depth: float, contact_kp: float,
                                  contact_kd: float, contact_max_vel: float) -> str:
    return f"""
      <collision name="{name}">
        <pose>{fmt(pose[0])} {fmt(pose[1])} {fmt(pose[2])} {fmt(pose[3])} {fmt(pose[4])} {fmt(pose[5])}</pose>
        <geometry><cylinder><radius>{fmt(radius)}</radius><length>{fmt(length)}</length></cylinder></geometry>
        <max_contacts>10</max_contacts>
        <surface>
          <bounce><restitution_coefficient>0</restitution_coefficient><threshold>100000</threshold></bounce>
          <friction><torsional><coefficient>1</coefficient><use_patch_radius>true</use_patch_radius><patch_radius>0</patch_radius><surface_radius>0</surface_radius><ode><slip>0</slip></ode></torsional><ode><mu>{fmt(mu)}</mu><mu2>{fmt(mu)}</mu2></ode></friction>
          <contact><ode><kp>{fmt(contact_kp)}</kp><kd>{fmt(contact_kd)}</kd><max_vel>{fmt(contact_max_vel)}</max_vel><min_depth>{fmt(min_depth)}</min_depth></ode></contact>
        </surface>
      </collision>"""


def gazebo_cylinder_visual_xml(name: str,
                               pose: Tuple[float, float, float, float, float, float],
                               radius: float, length: float,
                               color: Tuple[float, float, float, float]) -> str:
    return f"""
      <visual name="{name}">
        <pose>{fmt(pose[0])} {fmt(pose[1])} {fmt(pose[2])} {fmt(pose[3])} {fmt(pose[4])} {fmt(pose[5])}</pose>
        <geometry><cylinder><radius>{fmt(radius)}</radius><length>{fmt(length)}</length></cylinder></geometry>
        {gazebo_material_xml(color)}
      </visual>"""


def grouser_primitives(shape: str, grouser_width: float, track_width: float,
                       height: float) -> list:
    """Return (geometry, dx, dz, sx_or_radius, sy, sz) primitives."""
    shape = shape.lower()
    if shape == "trapezoid":
        layer_height = height / 3.0
        return [
            ("box", 0.0, (j + 0.5) * layer_height,
             grouser_width * scale, track_width, layer_height)
            for j, scale in enumerate((1.0, 0.75, 0.5))
        ]
    if shape == "spike":
        # Wide at the belt and progressively narrower toward the contact tip.
        layer_height = height / 3.0
        return [
            ("box", 0.0, (j + 0.5) * layer_height,
             grouser_width * scale, track_width, layer_height)
            for j, scale in enumerate((1.0, 2.0 / 3.0, 1.0 / 3.0))
        ]
    if shape == "semicircle":
        # The lower half is embedded in the belt, leaving a semicircular crown.
        return [("cylinder", 0.0, 0.0, height, track_width, 0.0)]
    if shape == "fillet":
        radius = min(height / 2.0, grouser_width / 2.0)
        straight = max(grouser_width - 2.0 * radius, 1e-4)
        return [
            ("box", 0.0, height / 2.0, straight, track_width, height),
            ("cylinder", -straight / 2.0, height / 2.0,
             radius, track_width, 0.0),
            ("cylinder", straight / 2.0, height / 2.0,
             radius, track_width, 0.0),
        ]
    return [("box", 0.0, height / 2.0,
             grouser_width, track_width, height)]


def static_straight_grousers_xml(count: int, length: float, width: float,
                                 grouser_height: float, grouser_width: float,
                                 pitch: float, mu: float, min_depth: float,
                                 contact_kp: float, contact_kd: float,
                                 contact_max_vel: float,
                                 color: Tuple[float, float, float, float],
                                 shape: str = "rectangle") -> str:
    if count <= 0 or grouser_height <= 0.0:
        return ""
    effective_width = min(grouser_width, pitch * 0.9)
    primitives = grouser_primitives(shape, effective_width, width, grouser_height)
    parts = []
    for i in range(count):
        x = min(pitch * (i + 0.5), length)
        for j, (geometry, dx, dz, sx, sy, sz) in enumerate(primitives):
            pose = (x + dx, 0.0, dz,
                    math.pi / 2.0 if geometry == "cylinder" else 0.0, 0.0, 0.0)
            key = f"{i}_{j}"
            if geometry == "cylinder":
                parts.append(gazebo_cylinder_collision_xml(
                    f"grouser_collision_{key}", pose, sx, sy, mu, min_depth,
                    contact_kp, contact_kd, contact_max_vel))
                parts.append(gazebo_cylinder_visual_xml(
                    f"grouser_visual_{key}", pose, sx, sy, color))
            else:
                size = (sx, sy, sz)
                parts.append(gazebo_box_collision_xml(
                    f"grouser_collision_{key}", pose, size, mu, min_depth,
                    contact_kp, contact_kd, contact_max_vel))
                parts.append(gazebo_box_visual_xml(
                    f"grouser_visual_{key}", pose, size, color))
    return "".join(parts)


def static_arc_grousers_xml(count: int, radius: float, length: float,
                            grouser_height: float, grouser_width: float,
                            pitch: float, mu: float, min_depth: float,
                            contact_kp: float, contact_kd: float,
                            contact_max_vel: float,
                            color: Tuple[float, float, float, float],
                            shape: str = "rectangle") -> str:
    if count <= 0 or grouser_height <= 0.0:
        return ""
    primitives = grouser_primitives(shape, grouser_width, length, grouser_height)
    angle_step = pitch / max(radius, 1e-6)
    parts = []
    for i in range(count):
        angle = angle_step * (i + 0.5)
        for j, (geometry, dx, dz, sx, sy, sz) in enumerate(primitives):
            radial = radius + dz
            px = radial * math.sin(angle) + dx * math.cos(angle)
            pz = -radius + radial * math.cos(angle) - dx * math.sin(angle)
            pose = (px, 0.0, pz,
                    math.pi / 2.0 if geometry == "cylinder" else 0.0,
                    angle, 0.0)
            key = f"{i}_{j}"
            if geometry == "cylinder":
                parts.append(gazebo_cylinder_collision_xml(
                    f"grouser_collision_{key}", pose, sx, sy, mu, min_depth,
                    contact_kp, contact_kd, contact_max_vel))
                parts.append(gazebo_cylinder_visual_xml(
                    f"grouser_visual_{key}", pose, sx, sy, color))
            else:
                size = (sx, sy, sz)
                parts.append(gazebo_box_collision_xml(
                    f"grouser_collision_{key}", pose, size, mu, min_depth,
                    contact_kp, contact_kd, contact_max_vel))
                parts.append(gazebo_box_visual_xml(
                    f"grouser_visual_{key}", pose, size, color))
    return "".join(parts)


def gazebo_box_link(name: str, pose: Tuple[float, float, float, float, float, float],
                    size: Tuple[float, float, float], mass: float,
                    mu: float, contact: Tuple[float, float, float, float],
                    color: Tuple[float, float, float, float],
                    grouser_count: int = 0, grouser_height: float = 0.0,
                    grouser_width: float = 0.0, grouser_pitch: float = 0.0,
                    grouser_shape: str = "rectangle") -> str:
    sx, sy, sz = size
    min_depth, contact_kp, contact_kd, contact_max_vel = contact
    grouser_xml = static_straight_grousers_xml(
        grouser_count, sx, sy, grouser_height, grouser_width,
        max(grouser_pitch, 1e-6), mu, min_depth,
        contact_kp, contact_kd, contact_max_vel, color, grouser_shape
    )
    return f"""
  <gazebo>
    <link name="{name}">
      <gravity>false</gravity>
      <pose>{fmt(pose[0])} {fmt(pose[1])} {fmt(pose[2])} {fmt(pose[3])} {fmt(pose[4])} {fmt(pose[5])}</pose>
      <inertial>
        <mass>{fmt(mass)}</mass>
        {sdf_inertia_box(mass, sx, sy, sz)}
      </inertial>
      <collision name="collision">
        <pose>{fmt(sx / 2.0)} 0 {fmt(-sz / 2.0)} 0 0 0</pose>
        <geometry><box><size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size></box></geometry>
        <surface>
          <friction><ode><mu>{fmt(mu)}</mu><mu2>{fmt(mu)}</mu2></ode></friction>
          <contact><ode><kp>{fmt(contact_kp)}</kp><kd>{fmt(contact_kd)}</kd><max_vel>{fmt(contact_max_vel)}</max_vel><min_depth>{fmt(min_depth)}</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name="visual">
        <pose>{fmt(sx / 2.0)} 0 {fmt(-sz / 2.0)} 0 0 0</pose>
        <geometry><box><size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size></box></geometry>
        {gazebo_material_xml(color)}
      </visual>
{grouser_xml}
    </link>
  </gazebo>
"""


def gazebo_cylinder_link(name: str, pose: Tuple[float, float, float, float, float, float],
                         radius: float, length: float, mass: float,
                         mu: float, contact: Tuple[float, float, float, float],
                         color: Tuple[float, float, float, float],
                         grouser_count: int = 0, grouser_height: float = 0.0,
                         grouser_width: float = 0.0, grouser_pitch: float = 0.0,
                         grouser_shape: str = "rectangle") -> str:
    min_depth, contact_kp, contact_kd, contact_max_vel = contact
    grouser_xml = static_arc_grousers_xml(
        grouser_count, radius, length, grouser_height, grouser_width,
        max(grouser_pitch, 1e-6), mu, min_depth,
        contact_kp, contact_kd, contact_max_vel, color, grouser_shape
    )
    return f"""
  <gazebo>
    <link name="{name}">
      <gravity>false</gravity>
      <pose>{fmt(pose[0])} {fmt(pose[1])} {fmt(pose[2])} {fmt(pose[3])} {fmt(pose[4])} {fmt(pose[5])}</pose>
      <inertial>
        <mass>{fmt(mass)}</mass>
        {sdf_inertia_cylinder_y(mass, radius, length)}
      </inertial>
      <collision name="collision">
        <pose>0 0 {fmt(-radius)} {fmt(math.pi / 2.0)} 0 0</pose>
        <geometry><cylinder><length>{fmt(length)}</length><radius>{fmt(radius)}</radius></cylinder></geometry>
        <surface>
          <friction><ode><mu>{fmt(mu)}</mu><mu2>{fmt(mu)}</mu2></ode></friction>
          <contact><ode><kp>{fmt(contact_kp)}</kp><kd>{fmt(contact_kd)}</kd><max_vel>{fmt(contact_max_vel)}</max_vel><min_depth>{fmt(min_depth)}</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name="visual">
        <pose>0 0 {fmt(-radius)} {fmt(math.pi / 2.0)} 0 0</pose>
        <geometry><cylinder><length>{fmt(length)}</length><radius>{fmt(radius)}</radius></cylinder></geometry>
        {gazebo_material_xml(color)}
      </visual>
{grouser_xml}
    </link>
  </gazebo>
"""


def gazebo_track_joint(name: str, joint_type: str, parent: str, child: str,
                       pose: Tuple[float, float, float, float, float, float],
                       axis: Tuple[float, float, float], implicit_spring_damper: Any) -> str:
    return f"""
  <gazebo>
    <joint name="{name}" type="{joint_type}">
      <parent>{parent}</parent>
      <child>{child}</child>
      <pose>{fmt(pose[0])} {fmt(pose[1])} {fmt(pose[2])} {fmt(pose[3])} {fmt(pose[4])} {fmt(pose[5])}</pose>
      <axis>
        <xyz>{fmt(axis[0])} {fmt(axis[1])} {fmt(axis[2])}</xyz>
        <use_parent_model_frame>0</use_parent_model_frame>
      </axis>
      <physics><ode><implicit_spring_damper>{implicit_spring_damper}</implicit_spring_damper></ode></physics>
    </joint>
  </gazebo>
"""


def continuous_track_plugin_xml(name: str, sprocket_joint: str, length: float,
                                params: Dict[str, Any], elements_per_round: int) -> str:
    return f"""
  <gazebo>
    <plugin name="{name}" filename="libContinuousTrack.so">
      <sprocket>
        <joint>{sprocket_joint}</joint>
        <pitch_diameter>{fmt(params["pitch_diameter"])}</pitch_diameter>
      </sprocket>
      <trajectory>
        <segment><joint>{name}_straight_segment_joint0</joint><end_position>{fmt(length)}</end_position></segment>
        <segment><joint>{name}_arc_segment_joint0</joint><end_position>{fmt(math.pi)}</end_position></segment>
        <segment><joint>{name}_straight_segment_joint1</joint><end_position>{fmt(length)}</end_position></segment>
        <segment><joint>{name}_arc_segment_joint1</joint><end_position>{fmt(math.pi)}</end_position></segment>
      </trajectory>
      <pattern>
        <elements_per_round>{int(elements_per_round)}</elements_per_round>
        <element/>
      </pattern>
    </plugin>
  </gazebo>
"""


def drive_plugin_xml(geometry: Dict[str, float], track: Dict[str, Any]) -> str:
    track_width = geometry["body_gap"] + geometry["body_width"]
    return f"""
  <gazebo>
    <plugin name="crawler_gazebo_drive" filename="libcrawler_gazebo_drive.so">
      <cmd_vel_topic>/target/cmd_vel</cmd_vel_topic>
      <track_width>{fmt(track_width)}</track_width>
      <sprocket_radius>{fmt(geometry["wheel_radius"])}</sprocket_radius>
      <motor_max_rpm>78.378</motor_max_rpm>
      <gear_ratio>2.5556</gear_ratio>
      <command_scale>0.55</command_scale>
      <max_linear_velocity>-1</max_linear_velocity>
      <max_angular_velocity>-1</max_angular_velocity>
      <command_timeout>0.5</command_timeout>
    </plugin>
  </gazebo>
"""


def continuous_track_xml(name: str, parent: str, sprocket_joint: str,
                         center: Tuple[float, float, float], length: float,
                         radius: float, width: float, mass: float,
                         params: Dict[str, Any]) -> str:
    if not bool(params.get("enabled", True)):
        return ""
    x, y, z = center
    mu = params["mu"]
    contact = track_contact_settings(params)
    damping = params["implicit_spring_damper"]
    color = (0.05, 0.05, 0.05, 1.0)
    half_mass = mass / 4.0
    belt_thickness = max(params["belt_thickness"], 1e-4)
    track_width = width + max(0.006, width * 0.08)
    straight_size = (length, track_width, belt_thickness)
    track_radius = max(radius + belt_thickness, 1e-4)
    grouser_height = max(float(params.get("grouser_height", 0.0)), 0.0)
    grouser_width = max(float(params.get("belt_element_length", 0.02)), 1e-4)
    grouser_shape = str(params.get("grouser_shape", "rectangle"))
    perimeter = 2.0 * length + 2.0 * math.pi * track_radius
    elements_per_round = max(int(params["elements_per_round"]), 4)
    grouser_pitch = perimeter / elements_per_round
    straight_grouser_count = (
        max(1, int(round(length / grouser_pitch)))
        if grouser_height > 0.0
        else 0
    )
    arc_grouser_count = (
        max(1, int(round(2.0 * math.pi * track_radius / grouser_pitch)))
        if grouser_height > 0.0
        else 0
    )

    return (
        gazebo_box_link(
            f"{name}_straight_segment_link0",
            (x - length / 2.0, y, z + track_radius, 0.0, 0.0, 0.0),
            straight_size,
            half_mass,
            mu,
            contact,
            color,
            straight_grouser_count,
            grouser_height,
            grouser_width,
            grouser_pitch,
            grouser_shape,
        )
        + gazebo_box_link(
            f"{name}_straight_segment_link1",
            (x + length / 2.0, y, z - track_radius, 0.0, math.pi, 0.0),
            straight_size,
            half_mass,
            mu,
            contact,
            color,
            straight_grouser_count,
            grouser_height,
            grouser_width,
            grouser_pitch,
            grouser_shape,
        )
        + gazebo_cylinder_link(
            f"{name}_arc_segment_link0",
            (x + length / 2.0, y, z + track_radius, 0.0, 0.0, 0.0),
            track_radius,
            track_width,
            half_mass,
            mu,
            contact,
            color,
            arc_grouser_count,
            grouser_height,
            grouser_width,
            grouser_pitch,
            grouser_shape,
        )
        + gazebo_cylinder_link(
            f"{name}_arc_segment_link1",
            (x - length / 2.0, y, z - track_radius, 0.0, math.pi, 0.0),
            track_radius,
            track_width,
            half_mass,
            mu,
            contact,
            color,
            arc_grouser_count,
            grouser_height,
            grouser_width,
            grouser_pitch,
            grouser_shape,
        )
        + gazebo_track_joint(
            f"{name}_straight_segment_joint0",
            "prismatic",
            parent,
            f"{name}_straight_segment_link0",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            damping,
        )
        + gazebo_track_joint(
            f"{name}_straight_segment_joint1",
            "prismatic",
            parent,
            f"{name}_straight_segment_link1",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            damping,
        )
        + gazebo_track_joint(
            f"{name}_arc_segment_joint0",
            "revolute",
            parent,
            f"{name}_arc_segment_link0",
            (0.0, 0.0, -track_radius, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            damping,
        )
        + gazebo_track_joint(
            f"{name}_arc_segment_joint1",
            "revolute",
            parent,
            f"{name}_arc_segment_link1",
            (0.0, 0.0, -track_radius, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            damping,
        )
        + continuous_track_plugin_xml(name, sprocket_joint, length, params, elements_per_round)
    )


def body_track_height(g: Dict[str, float]) -> float:
    return 2.0 * g["wheel_radius"]


def make_body_side(side: str, sign: int, g: Dict[str, float], m: Dict[str, float], j: Dict[str, float]) -> str:
    body_link = f"main_body_{side}"
    sprocket_link = f"sprocket_{side}"
    idler_link = f"idler_{side}"
    body_half = g["body_length"] / 2.0
    y = sign * g["body_gap"] / 2.0
    sprocket_x = sign * body_half
    idler_x = -sign * body_half
    return (
        fixed_joint(f"base_link2body_belt_{side}", "base_link", body_link, (0.0, y, 0.0))
        + box_link(
            body_link,
            (g["body_length"], g["body_width"], body_track_height(g)),
            m["body_track"],
            "track_green",
            collision=False,
        )
        + cylinder_link(
            sprocket_link,
            g["wheel_radius"],
            g["body_width"],
            m["body_sprocket"],
            "pulley_dark",
            collision=True,
            collision_radius=g["wheel_radius"] + g.get("track_collision_margin", 0.0),
        )
        + continuous_joint(
            f"sprocket_axle_{side}",
            body_link,
            sprocket_link,
            (sprocket_x, 0.0, 0.0),
            (0.0, -1.0, 0.0),
            j["sprocket_effort"],
            j["sprocket_velocity"],
        )
        + cylinder_link(
            idler_link,
            g["wheel_radius"],
            g["body_width"],
            m["body_sprocket"],
            "pulley_dark",
            collision=True,
            collision_radius=g["wheel_radius"] + g.get("track_collision_margin", 0.0),
        )
        + fixed_joint(f"idler_axle_{side}", body_link, idler_link, (idler_x, 0.0, 0.0))
        + transmission(
            f"sprocket_transmission_{side}",
            f"sprocket_axle_{side}",
            "velocity",
        )
    )


def make_flipper(side: str, side_sign: int, where: str, front_sign: int,
                 g: Dict[str, float], m: Dict[str, float], j: Dict[str, float]) -> str:
    joint_name = f"joint_{side}_{where}"
    arm_link = f"{side}_{where}"
    body_link = f"flipper_body_{side}_{where}"
    sprocket_link = f"flipper_sprocket_{side}_{where}"
    idler_link = f"flipper_idler_{side}_{where}"
    sprocket_joint = f"flipper_sprocket_axle_{side}_{where}"

    joint_x = front_sign * g["body_length"] / 2.0
    joint_y = side_sign * g["flipper_y_offset"]
    axis_y = -front_sign * side_sign
    track_length = g["flipper_length"]
    body_center_x = front_sign * track_length / 2.0
    sprocket_x = -front_sign * track_length / 2.0
    idler_x = front_sign * track_length / 2.0
    flipper_torque_max = float(j.get("flipper_torque_max", j.get("flipper_effort", 1000.0)))

    return (
        revolute_joint(
            joint_name,
            "base_link",
            arm_link,
            (joint_x, joint_y, 0.0),
            (0.0, axis_y, 0.0),
            j["flipper_lower"],
            j["flipper_upper"],
            flipper_torque_max,
            j["flipper_velocity"],
            damping=float(j.get("flipper_damping", 0.0)),
            friction=float(j.get("flipper_friction", 0.0)),
        )
        + box_link(arm_link, (0.01, g["flipper_width"], 0.01), m["flipper_arm"], "flipper_blue")
        + fixed_joint(f"{side}_{where}2flipper_body", arm_link, body_link, (body_center_x, 0.0, 0.0))
        + box_link(
            body_link,
            (track_length, g["flipper_width"], 2.0 * g["wheel_radius"]),
            m["flipper_track"],
            "flipper_blue",
            collision=False,
        )
        + cylinder_link(
            sprocket_link,
            g["wheel_radius"],
            g["flipper_width"],
            m["flipper_sprocket"],
            "pulley_dark",
            collision=True,
            collision_radius=g["wheel_radius"] + g.get("track_collision_margin", 0.0),
        )
        + continuous_joint(
            sprocket_joint,
            body_link,
            sprocket_link,
            (sprocket_x, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            j["sprocket_effort"],
            j["sprocket_velocity"],
        )
        + cylinder_link(
            idler_link,
            g["wheel_radius"],
            g["flipper_width"],
            m["flipper_sprocket"],
            "pulley_dark",
            collision=True,
            collision_radius=g["wheel_radius"] + g.get("track_collision_margin", 0.0),
        )
        + fixed_joint(f"flipper_idler_axle_{side}_{where}", body_link, idler_link, (idler_x, 0.0, 0.0))
        + transmission(f"trans_{side}_{where}", joint_name, "position")
        + transmission(
            f"flipper_sprocket_transmission_{side}_{where}",
            sprocket_joint,
            "velocity",
        )
    )


def enabled_flippers(general: Dict[str, Any]) -> Iterable[Tuple[str, int]]:
    if bool(general.get("flipper_front", True)):
        yield ("front", 1)
    if bool(general.get("flipper_back", True)):
        yield ("rear", -1)


def generate_urdf(data: Dict[str, Any]) -> str:
    root = config_root(data)
    general = root["general"]
    g = root["geometry"]
    m = root["mass"]
    j = root["joints"]
    track = root["continuous_track"]
    robot_name = escape(str(general.get("robot_name", "crawler")))

    body = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<robot name="{robot_name}">',
        material_xml(),
        box_link("base_link", (g["base_length"], g["base_width"], g["base_height"]), m["base"], "base_gray"),
        make_body_side("left", 1, g, m, j),
        make_body_side("right", -1, g, m, j),
        continuous_track_xml(
            "track_left",
            "main_body_left",
            "sprocket_axle_left",
            (0.0, g["body_gap"] / 2.0, 0.0),
            g["body_length"],
            g["wheel_radius"],
            g["body_width"],
            0.4,
            track,
        ),
        continuous_track_xml(
            "track_right",
            "main_body_right",
            "sprocket_axle_right",
            (0.0, -g["body_gap"] / 2.0, 0.0),
            g["body_length"],
            g["wheel_radius"],
            g["body_width"],
            0.4,
            track,
        ),
    ]

    for side, side_sign in (("left", 1), ("right", -1)):
        for where, front_sign in enabled_flippers(general):
            body.append(make_flipper(side, side_sign, where, front_sign, g, m, j))
            joint_x = front_sign * g["body_length"] / 2.0
            joint_y = side_sign * g["flipper_y_offset"]
            body.append(
                continuous_track_xml(
                    f"flipper_track_{side}_{where}",
                    f"flipper_body_{side}_{where}",
                    f"flipper_sprocket_axle_{side}_{where}",
                    (joint_x + front_sign * g["flipper_length"] / 2.0, joint_y, 0.0),
                    g["flipper_length"],
                    g["wheel_radius"],
                    g["flipper_width"],
                    0.4,
                    track,
                )
            )

    body.append(imu_xml(bool(general.get("imu", True))))
    if bool(general.get("gazebo_drive_plugin", False)):
        body.append(drive_plugin_xml(g, track))
    body.append(ros2_control_xml(bool(general.get("gazebo_ros_control", True)), general, j))
    body.append(gazebo_plugin_xml(bool(general.get("gazebo_ros_control", True))))
    body.append("</robot>\n")
    return "\n".join(body)


def write_urdf(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(generate_urdf(data), encoding="utf-8")


def apply_cli_sets(data: Dict[str, Any], assignments: Iterable[str]) -> None:
    for assignment in assignments:
        if "=" not in assignment:
            raise ValueError(f"--set expects key=value, got: {assignment}")
        key, raw_value = assignment.split("=", 1)
        current = get_path(data, key)
        set_path(data, key, parse_value(raw_value, current))


def rotate_point(point: Tuple[float, float, float], yaw_deg: float = -38.0,
                 pitch_deg: float = 24.0) -> Tuple[float, float, float]:
    x, y, z = point
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)

    view_x = cy * x - sy * y
    view_y = sy * x + cy * y
    view_z = z

    screen_x = view_x
    depth = view_y * cp + view_z * sp
    screen_y = view_z * cp - view_y * sp
    return screen_x, depth, screen_y


def project_point(point: Tuple[float, float, float], scale: float,
                  origin: Tuple[float, float], view: Dict[str, float]) -> Tuple[float, float]:
    if view.get("mode") == "top":
        x, y, _ = point
        return origin[0] + scale * x, origin[1] - scale * y
    x, _, y = rotate_point(point, view["yaw"], view["pitch"])
    return origin[0] + scale * x, origin[1] - scale * y


def box_corners(center: Tuple[float, float, float],
                size: Tuple[float, float, float]) -> Tuple[Tuple[float, float, float], ...]:
    cx, cy, cz = center
    sx, sy, sz = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
    return (
        (cx - sx, cy - sy, cz - sz),
        (cx + sx, cy - sy, cz - sz),
        (cx + sx, cy + sy, cz - sz),
        (cx - sx, cy + sy, cz - sz),
        (cx - sx, cy - sy, cz + sz),
        (cx + sx, cy - sy, cz + sz),
        (cx + sx, cy + sy, cz + sz),
        (cx - sx, cy + sy, cz + sz),
    )


def draw_box(canvas: Any, center: Tuple[float, float, float], size: Tuple[float, float, float],
             scale: float, origin: Tuple[float, float], view: Dict[str, float],
             outline: str, fill: str) -> None:
    corners = box_corners(center, size)
    faces = (
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    )
    face_depths = []
    for face in faces:
        depth = sum(rotate_point(corners[i], view["yaw"], view["pitch"])[1] for i in face) / len(face)
        face_depths.append((depth, face))
    for _, face in sorted(face_depths):
        points = []
        for index in face:
            points.extend(project_point(corners[index], scale, origin, view))
        canvas.create_polygon(points, fill=fill, outline=outline, width=1)


def draw_cylinder_marker(canvas: Any, center: Tuple[float, float, float], radius: float,
                         length: float, scale: float, origin: Tuple[float, float],
                         view: Dict[str, float], outline: str, fill: str) -> None:
    cx, cy, cz = center
    points_front = []
    points_back = []
    for step in range(18):
        angle = 2.0 * math.pi * step / 18.0
        x = cx + radius * math.cos(angle)
        z = cz + radius * math.sin(angle)
        points_front.append(project_point((x, cy - length / 2.0, z), scale, origin, view))
        points_back.append(project_point((x, cy + length / 2.0, z), scale, origin, view))

    def flatten(points: Iterable[Tuple[float, float]]) -> Any:
        flat = []
        for px, py in points:
            flat.extend((px, py))
        return flat

    canvas.create_polygon(flatten(points_back), fill=fill, outline=outline, width=1)
    canvas.create_polygon(flatten(points_front), fill=fill, outline=outline, width=1)
    for index in range(0, 18, 3):
        canvas.create_line(
            points_front[index][0],
            points_front[index][1],
            points_back[index][0],
            points_back[index][1],
            fill=outline,
            width=1,
        )


def draw_axis(canvas: Any, name: str, end: Tuple[float, float, float], color: str,
              scale: float, origin: Tuple[float, float], view: Dict[str, float]) -> None:
    x0, y0 = project_point((0.0, 0.0, 0.0), scale, origin, view)
    x1, y1 = project_point(end, scale, origin, view)
    canvas.create_line(x0, y0, x1, y1, fill=color, width=3, arrow="last")
    canvas.create_text(x1, y1, text=name, fill=color, font=("Helvetica", 10, "bold"), anchor="w")


def draw_coordinate_axes(canvas: Any, length: float, scale: float, origin: Tuple[float, float],
                         view: Dict[str, float]) -> None:
    draw_axis(canvas, "X", (length, 0.0, 0.0), "#c92a2a", scale, origin, view)
    draw_axis(canvas, "Y", (0.0, length, 0.0), "#2b8a3e", scale, origin, view)
    draw_axis(canvas, "Z", (0.0, 0.0, length), "#1971c2", scale, origin, view)


def preview_bounds(data: Dict[str, Any]) -> Tuple[float, float]:
    root = config_root(data)
    g = root["geometry"]
    flipper_joint_x = g["body_length"] / 2.0
    flipper_joint_y = g["flipper_y_offset"]
    x_extent = max(g["base_length"], g["body_length"], 2.0 * (flipper_joint_x + g["flipper_length"] * 1.5))
    y_extent = max(g["base_width"], g["body_gap"] + g["body_width"], 2.0 * (flipper_joint_y + g["flipper_width"]))
    return max(x_extent, 0.1), max(y_extent, 0.1)


def draw_preview(canvas: Any, data: Dict[str, Any], view: Dict[str, float] | None = None) -> None:
    if view is None:
        view = {"yaw": -38.0, "pitch": 24.0}
    canvas.delete("all")
    width = max(canvas.winfo_width(), 420)
    height = max(canvas.winfo_height(), 360)
    canvas.create_rectangle(0, 0, width, height, fill="#f7f8f8", outline="")

    x_extent, y_extent = preview_bounds(data)
    scale = min(width / (x_extent * 1.75), height / (max(x_extent, y_extent) * 1.25))
    origin = (width * 0.50, height * 0.52)

    root = config_root(data)
    general = root["general"]
    g = root["geometry"]
    axis_length = max(g["base_length"], g["body_length"], g["flipper_length"]) * 0.35

    draw_box(
        canvas,
        (0.0, 0.0, 0.0),
        (g["base_length"], g["base_width"], g["base_height"]),
        scale,
        origin,
        view,
        "#555f66",
        "#b9c0c5",
    )

    for sign in (1, -1):
        y = sign * g["body_gap"] / 2.0
        draw_box(
            canvas,
            (0.0, y, 0.0),
            (g["body_length"], g["body_width"], body_track_height(g)),
            scale,
            origin,
            view,
            "#216653",
            "#6ad0ad",
        )
        for x in (g["body_length"] / 2.0, -g["body_length"] / 2.0):
            draw_cylinder_marker(
                canvas,
                (x, y, 0.0),
                g["wheel_radius"],
                g["body_width"],
                scale,
                origin,
                view,
                "#272727",
                "#555555",
            )

    draw_coordinate_axes(canvas, axis_length, scale, origin, view)

    arm_length = max(g["flipper_length"] * 0.5, g["wheel_radius"])
    for side_sign in (1, -1):
        for where, front_sign in enabled_flippers(general):
            joint_x = front_sign * g["body_length"] / 2.0
            joint_y = side_sign * g["flipper_y_offset"]
            arm_center = (joint_x + front_sign * arm_length / 2.0, joint_y, 0.0)
            track_center = (joint_x + front_sign * arm_length, joint_y, 0.0)
            draw_box(
                canvas,
                arm_center,
                (arm_length, g["flipper_width"], 2.0 * g["wheel_radius"]),
                scale,
                origin,
                view,
                "#07517f",
                "#4e9bd1",
            )
            draw_box(
                canvas,
                track_center,
                (g["flipper_length"], g["flipper_width"], 2.0 * g["wheel_radius"]),
                scale,
                origin,
                view,
                "#07517f",
                "#66b3e6",
            )
            for local_x in (g["flipper_length"] / 2.0, -g["flipper_length"] / 2.0):
                draw_cylinder_marker(
                    canvas,
                    (track_center[0] + front_sign * local_x, joint_y, 0.0),
                    g["wheel_radius"],
                    g["flipper_width"],
                    scale,
                    origin,
                    view,
                    "#202020",
                    "#4c4c4c",
                )

    canvas.create_text(12, 12, anchor="nw", text="3D preview", fill="#202426", font=("Helvetica", 11, "bold"))
    canvas.create_text(
        12,
        32,
        anchor="nw",
        text="Drag to rotate. Primitive URDF geometry: boxes and cylinders",
        fill="#4c555a",
        font=("Helvetica", 9),
    )


def run_gui(config_path: Path, data: Dict[str, Any], output_override: str | None) -> int:
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox
    except Exception as exc:
        print(f"Failed to import tkinter: {exc}", file=sys.stderr)
        return 1

    root = tk.Tk()
    root.title("Crawler URDF Generator")
    root.geometry("980x760")

    variables: Dict[str, tk.StringVar] = {}
    content = tk.PanedWindow(root, orient=tk.HORIZONTAL, sashwidth=6)
    content.pack(fill="both", expand=True, padx=12, pady=12)

    form_panel = tk.Frame(content)
    preview_panel = tk.Frame(content)
    content.add(form_panel, minsize=380)
    content.add(preview_panel, minsize=420)

    form_canvas = tk.Canvas(form_panel, highlightthickness=0)
    form_scroll = tk.Scrollbar(form_panel, orient="vertical", command=form_canvas.yview)
    main = tk.Frame(form_canvas)
    main.bind("<Configure>", lambda event: form_canvas.configure(scrollregion=form_canvas.bbox("all")))
    form_canvas.create_window((0, 0), window=main, anchor="nw")
    form_canvas.configure(yscrollcommand=form_scroll.set)
    form_canvas.pack(side="left", fill="both", expand=True)
    form_scroll.pack(side="right", fill="y")

    tk.Label(preview_panel, text="Geometry Preview", anchor="w", font=("Helvetica", 12, "bold")).pack(fill="x")
    preview_canvas = tk.Canvas(preview_panel, width=520, height=600, bg="#f7f8f8", highlightthickness=1,
                               highlightbackground="#cfd6da")
    preview_canvas.pack(fill="both", expand=True, pady=(6, 0))

    preview_after_id = None
    preview_view = {"yaw": -38.0, "pitch": 24.0, "mode": "orbit"}
    drag_state = {"x": 0, "y": 0}
    undo_stack = []
    redo_stack = []
    suppress_history = False

    for row, (key, label) in enumerate(GUI_FIELDS):
        tk.Label(main, text=label, anchor="w").grid(row=row, column=0, sticky="w", padx=(0, 8), pady=2)
        value = get_path(data, key)
        var = tk.StringVar(value=str(value))
        variables[key] = var
        entry = tk.Entry(main, textvariable=var, width=28)
        entry.grid(row=row, column=1, sticky="ew", pady=2)
        if key == "output.urdf":
            def browse(v: tk.StringVar = var) -> None:
                selected = filedialog.asksaveasfilename(
                    title="Output URDF",
                    defaultextension=".urdf",
                    filetypes=(("URDF", "*.urdf"), ("XML", "*.xml"), ("All files", "*")),
                )
                if selected:
                    v.set(selected)

            tk.Button(main, text="Browse", command=browse).grid(row=row, column=2, padx=(6, 0), pady=2)

    main.columnconfigure(1, weight=1)

    def current_snapshot() -> Dict[str, str]:
        return {key: var.get() for key, var in variables.items()}

    last_snapshot = current_snapshot()

    def apply_snapshot(snapshot: Dict[str, str]) -> None:
        nonlocal suppress_history
        suppress_history = True
        try:
            for key, value in snapshot.items():
                variables[key].set(value)
        finally:
            suppress_history = False
        schedule_preview()

    def record_history() -> None:
        nonlocal last_snapshot
        if suppress_history:
            return
        snapshot = current_snapshot()
        if snapshot == last_snapshot:
            return
        undo_stack.append(last_snapshot)
        if len(undo_stack) > 100:
            del undo_stack[0]
        redo_stack.clear()
        last_snapshot = snapshot

    def undo_action() -> None:
        nonlocal last_snapshot
        if not undo_stack:
            return
        redo_stack.append(current_snapshot())
        snapshot = undo_stack.pop()
        apply_snapshot(snapshot)
        last_snapshot = snapshot

    def redo_action() -> None:
        nonlocal last_snapshot
        if not redo_stack:
            return
        undo_stack.append(current_snapshot())
        snapshot = redo_stack.pop()
        apply_snapshot(snapshot)
        last_snapshot = snapshot

    def collect(show_errors: bool = True) -> Dict[str, Any] | None:
        new_data = copy.deepcopy(data)
        try:
            for key, var in variables.items():
                current = get_path(new_data, key)
                set_path(new_data, key, parse_value(var.get(), current))
        except Exception as exc:
            if show_errors:
                messagebox.showerror("Invalid value", str(exc))
            return None
        return new_data

    def redraw_preview() -> None:
        new_data = collect(show_errors=False)
        if new_data is not None:
            draw_preview(preview_canvas, new_data, preview_view)

    def schedule_preview(*_: Any) -> None:
        nonlocal preview_after_id
        record_history()
        if preview_after_id is not None:
            root.after_cancel(preview_after_id)
        preview_after_id = root.after(120, redraw_preview)

    for var in variables.values():
        var.trace_add("write", schedule_preview)
    preview_canvas.bind("<Configure>", lambda event: schedule_preview())

    def start_drag(event: Any) -> None:
        drag_state["x"] = event.x
        drag_state["y"] = event.y

    def drag_preview(event: Any) -> None:
        dx = event.x - drag_state["x"]
        dy = event.y - drag_state["y"]
        drag_state["x"] = event.x
        drag_state["y"] = event.y
        preview_view["mode"] = "orbit"
        preview_view["yaw"] += dx * 0.45
        preview_view["pitch"] = max(-89.9, min(89.9, preview_view["pitch"] - dy * 0.35))
        redraw_preview()

    def reset_preview(_: Any = None) -> None:
        preview_view["yaw"] = -38.0
        preview_view["pitch"] = 24.0
        preview_view["mode"] = "orbit"
        redraw_preview()

    def top_preview() -> None:
        preview_view["yaw"] = 0.0
        preview_view["pitch"] = 90.0
        preview_view["mode"] = "top"
        redraw_preview()

    preview_canvas.bind("<ButtonPress-1>", start_drag)
    preview_canvas.bind("<B1-Motion>", drag_preview)
    preview_canvas.bind("<Double-Button-1>", reset_preview)

    view_buttons = tk.Frame(preview_panel)
    view_buttons.pack(fill="x", pady=(6, 0))
    tk.Button(view_buttons, text="Top", command=top_preview).pack(side="left")
    tk.Button(view_buttons, text="Reset View", command=reset_preview).pack(side="left", padx=(8, 0))

    def save_yaml_action() -> None:
        new_data = collect()
        if new_data is None:
            return
        save_config(config_path, new_data)
        data.clear()
        data.update(new_data)
        messagebox.showinfo("Saved", f"Saved YAML:\n{config_path}")

    def generate_action(save_yaml: bool = False) -> None:
        new_data = collect()
        if new_data is None:
            return
        output_path = resolve_output_path(config_path, new_data, output_override)
        write_urdf(output_path, new_data)
        if save_yaml:
            save_config(config_path, new_data)
        data.clear()
        data.update(new_data)
        messagebox.showinfo("Generated", f"Generated URDF:\n{output_path}")

    buttons = tk.Frame(root, padx=12, pady=0)
    buttons.pack(fill="x", pady=(0, 12))
    tk.Button(buttons, text="Undo", command=undo_action).pack(side="left")
    tk.Button(buttons, text="Redo", command=redo_action).pack(side="left", padx=(8, 0))
    tk.Button(buttons, text="Save YAML", command=save_yaml_action).pack(side="left")
    tk.Button(buttons, text="Generate URDF", command=lambda: generate_action(False)).pack(side="left", padx=8)
    tk.Button(buttons, text="Save + Generate", command=lambda: generate_action(True)).pack(side="left")
    tk.Button(buttons, text="Close", command=root.destroy).pack(side="right")

    redraw_preview()
    root.mainloop()
    return 0


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="YAML config path")
    parser.add_argument("--output", help="URDF output path")
    parser.add_argument("--no-gui", action="store_true", help="Generate from YAML without showing the GUI")
    parser.add_argument("--write-default", action="store_true", help="Write the default YAML config and exit")
    parser.add_argument("--save-config", action="store_true", help="Save merged config after applying --set")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="Override a YAML value")
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] = sys.argv[1:]) -> int:
    args = parse_args(argv)
    config_path = args.config.expanduser().resolve()

    if args.write_default:
        save_config(config_path, DEFAULT_DATA)
        print(f"Wrote default config: {config_path}")
        return 0

    data = load_config(config_path)
    apply_cli_sets(data, args.set)

    if args.save_config:
        save_config(config_path, data)
        print(f"Saved config: {config_path}")

    if args.no_gui:
        output_path = resolve_output_path(config_path, data, args.output)
        write_urdf(output_path, data)
        print(f"Generated URDF: {output_path}")
        return 0

    return run_gui(config_path, data, args.output)


if __name__ == "__main__":
    raise SystemExit(main())
