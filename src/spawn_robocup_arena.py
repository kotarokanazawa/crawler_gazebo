#!/usr/bin/env python3
import math
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class RobocupArenaSpawner(Node):
    def __init__(self):
        super().__init__("spawn_robocup_arena")
        self.declare_parameter("arena_yaml", "")
        self.declare_parameter("use_wall_arg", False)
        self.declare_parameter("wall_sdf", "")
        self.client = self.create_client(SpawnEntity, "/spawn_entity")

    def find_sdf_from_gazebo_model_path(self, base_name):
        gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
        if not gazebo_model_path:
            self.get_logger().error("GAZEBO_MODEL_PATH is not set.")
            return ""

        for root in gazebo_model_path.split(":"):
            if not root:
                continue
            candidate = Path(root) / base_name / "model.sdf"
            if candidate.exists():
                return str(candidate)

        self.get_logger().error("model.sdf for '%s' not found in GAZEBO_MODEL_PATH.", base_name)
        return ""

    def load_config(self):
        arena_yaml = self.get_parameter("arena_yaml").get_parameter_value().string_value
        if not arena_yaml:
            return {}
        path = Path(arena_yaml)
        if not path.exists():
            self.get_logger().error("Arena yaml not found: %s", arena_yaml)
            return {}
        with path.open("r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}

    def spawn_one(self, yaml_key, sdf_path, x, y, z, roll, pitch, yaw,
                  scale_x=1.0, scale_y=1.0, scale_z=1.0):
        with open(sdf_path, "r", encoding="utf-8") as stream:
            xml = stream.read()

        try:
            root = ET.fromstring(xml)
            for mesh in root.findall(".//mesh"):
                scale_tag = mesh.find("scale")
                if scale_tag is not None and scale_tag.text:
                    parts = scale_tag.text.split()
                    if len(parts) == 3:
                        try:
                            sx, sy, sz = map(float, parts)
                        except ValueError:
                            sx, sy, sz = 1.0, 1.0, 1.0
                    else:
                        sx, sy, sz = 1.0, 1.0, 1.0
                else:
                    sx, sy, sz = 1.0, 1.0, 1.0
                    if scale_tag is None:
                        scale_tag = ET.SubElement(mesh, "scale")
                scale_tag.text = f"{sx * scale_x} {sy * scale_y} {sz * scale_z}"

            model_tag = root.find(".//model")
            base_name = model_tag.attrib["name"] if model_tag is not None and "name" in model_tag.attrib else yaml_key
            xml = ET.tostring(root, encoding="unicode")
        except Exception as exc:
            self.get_logger().warn("Failed to parse/modify SDF %s: %s", sdf_path, exc)
            base_name = yaml_key

        match = re.match(r"^(.+)_([0-9]+)$", yaml_key)
        model_name = f"{base_name}_{match.group(2)}" if match else base_name

        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = xml
        request.robot_namespace = ""
        request.reference_frame = "world"
        request.initial_pose = Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=quaternion_from_euler(roll, pitch, yaw),
        )

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("Failed to spawn %s: service call failed", yaml_key)
            return
        if not response.success:
            self.get_logger().error("Failed to spawn %s: %s", yaml_key, response.status_message)

    def run(self):
        if not self.client.wait_for_service(timeout_sec=120.0):
            self.get_logger().error("/spawn_entity service is not available.")
            return

        config = self.load_config()
        objects = config.get("robocup_arena", {}).get("objects", {})
        wall_cfg = config.get("robocup_arena", {}).get("wall", {})

        if not objects:
            self.get_logger().warn("No objects found in arena yaml.")

        for key, cfg in objects.items():
            match = re.match(r"^(.+)_([0-9]+)$", key)
            if not match:
                self.get_logger().warn("Object key '%s' does not match '<name>_<number>'. Skip.", key)
                continue
            sdf_path = self.find_sdf_from_gazebo_model_path(match.group(1))
            if not sdf_path:
                continue
            self.spawn_one(
                key,
                sdf_path,
                cfg.get("x", 0.0),
                cfg.get("y", 0.0),
                cfg.get("z", 0.0),
                math.radians(float(cfg.get("roll", 0.0))),
                math.radians(float(cfg.get("pitch", 0.0))),
                math.radians(float(cfg.get("yaw", 0.0))),
                float(cfg.get("scale_x", 1.0)),
                float(cfg.get("scale_y", 1.0)),
                float(cfg.get("scale_z", 1.0)),
            )

        use_wall_arg = self.get_parameter("use_wall_arg").get_parameter_value().bool_value
        wall_sdf = self.get_parameter("wall_sdf").get_parameter_value().string_value
        enable_wall = bool(wall_cfg.get("enable_default", False)) or use_wall_arg
        if enable_wall and wall_sdf:
            self.spawn_one(
                "wall",
                wall_sdf,
                wall_cfg.get("x", 0.0),
                wall_cfg.get("y", 0.0),
                wall_cfg.get("z", 0.0),
                0.0,
                0.0,
                math.radians(float(wall_cfg.get("yaw", 0.0))),
            )

        self.get_logger().info("All arena models spawned.")


def main():
    rclpy.init()
    node = RobocupArenaSpawner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
