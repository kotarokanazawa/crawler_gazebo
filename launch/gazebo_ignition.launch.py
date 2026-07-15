#!/usr/bin/env python3
import os
import math
import re
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in {"1", "true", "yes", "on"}


def _read_yaml(path):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}
    except FileNotFoundError:
        return {}


def _setting(settings, key, default):
    node = settings
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _strip_xml_declaration(xml):
    stripped = xml.lstrip()
    if stripped.startswith("<?xml"):
        return stripped.split("?>", 1)[1].lstrip()
    return xml


def _xacro_to_string(model, xacro_args):
    cmd = ["xacro", model] + [f"{name}:={value}" for name, value in xacro_args.items()]
    return subprocess.check_output(cmd, text=True)


def _default_spawn_z(crawler_gazebo_share, robot_size):
    robot_yaml = crawler_gazebo_share / "config" / "robot" / f"{robot_size}.yaml"
    config = _read_yaml(robot_yaml).get("robot_config", {})
    wheel_radius = float(_setting(config, "geometry.wheel_radius", 0.082))
    belt_thickness = float(_setting(config, "continuous_track.belt_thickness", 0.02))
    grouser_height = float(_setting(config, "continuous_track.grouser_height", 0.01))
    return f"{wheel_radius + belt_thickness + grouser_height + 0.05:.3f}"


def _optional_package_lib(package_name):
    try:
        return str(Path(get_package_prefix(package_name)) / "lib")
    except PackageNotFoundError:
        print(f"[crawler_gazebo] Optional package not found, skipping plugin path: {package_name}")
        return ""


def _world_name(worldfile):
    try:
        root = ET.parse(worldfile).getroot()
        world = root.find("world")
        if world is not None and world.get("name"):
            return world.get("name")
    except ET.ParseError:
        pass
    return "default"


def _append_sun_light(world):
    light = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    _set_child_text(light, "cast_shadows", "true")
    _set_child_text(light, "pose", "0 0 10 0 0 0")
    _set_child_text(light, "diffuse", "0.8 0.8 0.8 1")
    _set_child_text(light, "specular", "0.2 0.2 0.2 1")
    attenuation = ET.SubElement(light, "attenuation")
    _set_child_text(attenuation, "range", "1000")
    _set_child_text(attenuation, "constant", "0.9")
    _set_child_text(attenuation, "linear", "0.01")
    _set_child_text(attenuation, "quadratic", "0.001")
    _set_child_text(light, "direction", "-0.5 0.1 -0.9")


def _make_ignition_world_file(worldfile, max_step_size, real_time_update_rate,
                              physics_engine="dart", dart_collision_detector="bullet"):
    try:
        tree = ET.parse(worldfile)
    except ET.ParseError:
        return worldfile
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        return worldfile

    replaced_sun = False
    for include in list(world.findall("include")):
        uri = _child_text(include, "uri")
        if uri == "model://sun":
            world.remove(include)
            replaced_sun = True
    if replaced_sun:
        _append_sun_light(world)

    # A crawler has many small, fast-moving track collision shapes. Use a fine
    # step and select the gz-physics backend explicitly.
    physics = world.find("physics")
    if physics is None:
        physics = ET.SubElement(
            world, "physics", {"name": "crawler_physics", "type": "ignored", "default": "true"})
    else:
        physics.set("type", "ignored")
        physics.set("default", "true")
    _set_child_text(physics, "max_step_size", str(max_step_size))
    _set_child_text(physics, "real_time_update_rate", str(real_time_update_rate))
    _set_child_text(physics, "max_contacts", "50")
    old_dart = physics.find("dart")
    if old_dart is not None:
        physics.remove(old_dart)
    if physics_engine == "dart":
        dart = ET.SubElement(physics, "dart")
        _set_child_text(dart, "collision_detector", dart_collision_detector)

    # Once an SDF declares a world system explicitly, Ignition no longer adds
    # the default server systems. Keep the normal systems and add Contact.
    engine_plugins = {
        "dart": "libignition-physics-dartsim-plugin.so",
        "bullet": "libignition-physics-bullet-plugin.so",
        "tpe": "libignition-physics-tpe-plugin.so",
    }
    physics_plugin = ET.SubElement(world, "plugin", {
        "filename": "ignition-gazebo-physics-system",
        "name": "ignition::gazebo::systems::Physics",
    })
    engine = ET.SubElement(physics_plugin, "engine")
    _set_child_text(engine, "filename", engine_plugins[physics_engine])
    for filename, name in (
        ("ignition-gazebo-user-commands-system", "ignition::gazebo::systems::UserCommands"),
        ("ignition-gazebo-scene-broadcaster-system", "ignition::gazebo::systems::SceneBroadcaster"),
        ("ignition-gazebo-contact-system", "ignition::gazebo::systems::Contact"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": name})

    path = Path(tempfile.gettempdir()) / f"crawler_gazebo_ignition_{Path(worldfile).stem}.sdf"
    tree.write(path, encoding="unicode", xml_declaration=True)
    return path


def _quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return {
        "w": cr * cp * cy + sr * sp * sy,
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
    }


def _find_model_sdf(model_root, base_name):
    candidate = model_root / base_name / "model.sdf"
    return candidate if candidate.exists() else None


def _crawler_gazebo_package_root(package_share):
    env_source = os.environ.get("CRAWLER_GAZEBO_SOURCE_DIR", "")
    if env_source:
        source = Path(env_source).expanduser()
        if (source / "package.xml").exists():
            return source

    for parent in [package_share, *package_share.parents]:
        if parent.name != "install":
            continue
        workspace = parent.parent
        for package_xml in (workspace / "src").glob("**/crawler_gazebo/package.xml"):
            return package_xml.parent
    return package_share


def _resolve_package_file(value, package_share, subdir=""):
    package_root = _crawler_gazebo_package_root(package_share)
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    if str(value).startswith("package://crawler_gazebo/"):
        return package_root / str(value)[len("package://crawler_gazebo/"):]
    return package_root / subdir / path


def _scaled_mesh_value(value, factor, positive_only=False):
    scaled = value * factor
    if abs(scaled) < 1e-6:
        scaled = 1e-6 if scaled >= 0.0 else -1e-6
    return abs(scaled) if positive_only else scaled


def _set_mesh_scale(mesh, scale_x, scale_y, scale_z, positive_only=False):
    scale = mesh.find("scale")
    if scale is not None and scale.text:
        parts = scale.text.split()
        sx, sy, sz = map(float, parts) if len(parts) == 3 else (1.0, 1.0, 1.0)
    else:
        sx, sy, sz = 1.0, 1.0, 1.0
        if scale is None:
            scale = ET.SubElement(mesh, "scale")
    scale.text = (
        f"{_scaled_mesh_value(sx, scale_x, positive_only)} "
        f"{_scaled_mesh_value(sy, scale_y, positive_only)} "
        f"{_scaled_mesh_value(sz, scale_z, positive_only)}"
    )


def _parse_pose_text(text):
    if not text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    parts = text.split()
    if len(parts) != 6:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    try:
        return tuple(float(part) for part in parts)
    except ValueError:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat_vec(matrix, vector):
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(vector):
    norm = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if norm < 1e-12:
        return (0.0, 0.0, 0.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def _mesh_scale(mesh):
    scale = mesh.find("scale")
    if scale is not None and scale.text:
        parts = scale.text.split()
        if len(parts) == 3:
            try:
                return tuple(float(part) for part in parts)
            except ValueError:
                pass
    return (1.0, 1.0, 1.0)


def _resolve_mesh_uri(uri, sdf_path):
    if uri.startswith("model://"):
        rest = uri[len("model://"):]
        model_name, _, rel = rest.partition("/")
        if model_name and rel:
            return sdf_path.parent.parent / model_name / rel
    if uri.startswith("file://"):
        return Path(uri[len("file://"):])
    return sdf_path.parent / uri


def _read_stl_triangles(path):
    data = path.read_bytes()
    if len(data) >= 84:
        tri_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + tri_count * 50 == len(data):
            triangles = []
            offset = 84
            for _ in range(tri_count):
                values = struct.unpack_from("<12fH", data, offset)
                triangles.append((values[0:3], values[3:6], values[6:9], values[9:12]))
                offset += 50
            return triangles

    triangles = []
    vertices = []
    for line in data.decode("utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            if len(vertices) == 3:
                normal = _normalize(_cross(_sub(vertices[1], vertices[0]), _sub(vertices[2], vertices[0])))
                triangles.append((normal, vertices[0], vertices[1], vertices[2]))
                vertices = []
    return triangles


def _write_stl_triangles(path, triangles):
    header = b"crawler_gazebo baked ignition mesh".ljust(80, b" ")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(struct.pack("<I", len(triangles)))
        for normal, v1, v2, v3 in triangles:
            stream.write(struct.pack("<12fH", *(normal + v1 + v2 + v3), 0))


def _bake_stl_mesh(mesh, sdf_path, model_pose, out_stem, mesh_index):
    uri = mesh.find("uri")
    if uri is None or not uri.text:
        return False
    source = _resolve_mesh_uri(uri.text.strip(), sdf_path)
    if source.suffix.lower() != ".stl" or not source.exists():
        return False

    sx, sy, sz = _mesh_scale(mesh)
    tx, ty, tz, roll, pitch, yaw = model_pose
    rotation = _rpy_matrix(roll, pitch, yaw)

    def transform(vertex):
        scaled = (vertex[0] * sx, vertex[1] * sy, vertex[2] * sz)
        rotated = _mat_vec(rotation, scaled)
        return (rotated[0] + tx, rotated[1] + ty, rotated[2] + tz)

    baked = []
    for _, v1, v2, v3 in _read_stl_triangles(source):
        tv1, tv2, tv3 = transform(v1), transform(v2), transform(v3)
        normal = _normalize(_cross(_sub(tv2, tv1), _sub(tv3, tv1)))
        baked.append((normal, tv1, tv2, tv3))

    out_dir = Path(tempfile.gettempdir()) / "crawler_gazebo_ignition_meshes"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{out_stem}_{mesh_index}.stl"
    _write_stl_triangles(out_path, baked)
    uri.text = str(out_path)
    scale = mesh.find("scale")
    if scale is None:
        scale = ET.SubElement(mesh, "scale")
    scale.text = "1 1 1"
    return True


def _scaled_sdf_file(sdf_path, model_name, scale_x=1.0, scale_y=1.0, scale_z=1.0):

    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find(".//model")
        model_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if model is not None:
            model.set("name", model_name)
            model_pose = _parse_pose_text(_child_text(model, "pose"))
        meshes = root.findall(".//mesh")
        baked_all = bool(meshes)
        for index, mesh in enumerate(meshes):
            if not _bake_stl_mesh(mesh, Path(sdf_path), model_pose, model_name, index):
                baked_all = False
                break
        if baked_all and model is not None:
            _set_child_text(model, "pose", "0 0 0 0 0 0")
        handled = set()
        for visual in root.findall(".//visual"):
            # Arena objects are temporary scaled copies. Override only their
            # visuals with an earthy brown; collision properties are unchanged.
            old_material = visual.find("material")
            if old_material is not None:
                visual.remove(old_material)
            material = ET.SubElement(visual, "material")
            _set_child_text(material, "ambient", "0.24 0.12 0.045 1")
            _set_child_text(material, "diffuse", "0.42 0.23 0.08 1")
            _set_child_text(material, "specular", "0.06 0.04 0.02 1")
            _set_child_text(material, "emissive", "0 0 0 1")
            for mesh in visual.findall(".//mesh"):
                _set_mesh_scale(mesh, scale_x, scale_y, scale_z, positive_only=False)
                handled.add(id(mesh))
        for collision in root.findall(".//collision"):
            for mesh in collision.findall(".//mesh"):
                _set_mesh_scale(mesh, scale_x, scale_y, scale_z, positive_only=True)
                handled.add(id(mesh))
        for mesh in root.findall(".//mesh"):
            if id(mesh) not in handled:
                _set_mesh_scale(mesh, scale_x, scale_y, scale_z, positive_only=True)
        out = Path(tempfile.gettempdir()) / f"crawler_gazebo_ignition_arena_{model_name}.sdf"
        tree.write(out, encoding="unicode", xml_declaration=True)
        return out
    except Exception:
        return sdf_path


def _arena_spawn_actions(arena_yaml, model_root, world_name, spawn_delay, use_wall_arg=False, wall_sdf=""):
    config = _read_yaml(arena_yaml)
    arena = config.get("robocup_arena", {})
    objects = arena.get("objects", {})
    actions = []
    delay = spawn_delay

    def add_spawn(name, sdf_path, cfg):
        nonlocal delay
        q = _quaternion_from_euler(
            math.radians(float(cfg.get("roll", 0.0))),
            math.radians(float(cfg.get("pitch", 0.0))),
            math.radians(float(cfg.get("yaw", 0.0))),
        )
        request = (
            f'sdf_filename: "{sdf_path}", '
            f'name: "{name}", '
            f'pose: {{position: {{x: {float(cfg.get("x", 0.0))}, '
            f'y: {float(cfg.get("y", 0.0))}, z: {float(cfg.get("z", 0.0))}}}, '
            f'orientation: {{w: {q["w"]}, x: {q["x"]}, y: {q["y"]}, z: {q["z"]}}}}}'
        )
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ign",
                            "service",
                            "-s",
                            f"/world/{world_name}/create",
                            "--reqtype",
                            "ignition.msgs.EntityFactory",
                            "--reptype",
                            "ignition.msgs.Boolean",
                            "--timeout",
                            "10000",
                            "--req",
                            request,
                        ],
                        output="log",
                    )
                ],
            )
        )
        delay += 0.35

    for key, cfg in objects.items():
        match = re.match(r"^(.+)_([0-9]+)$", key)
        if not match:
            continue
        base_name = match.group(1)
        sdf_path = _find_model_sdf(model_root, base_name)
        if sdf_path is None:
            continue
        model_name = f"{base_name}_{match.group(2)}"
        scaled = _scaled_sdf_file(
            sdf_path,
            model_name,
            float(cfg.get("scale_x", 1.0)),
            float(cfg.get("scale_y", 1.0)),
            float(cfg.get("scale_z", 1.0)),
        )
        add_spawn(model_name, scaled, cfg)

    wall_cfg = arena.get("wall", {})
    if (bool(wall_cfg.get("enable_default", False)) or use_wall_arg) and wall_sdf:
        add_spawn("wall", Path(wall_sdf), wall_cfg)

    return actions


def _remove_element(parent, child):
    try:
        parent.remove(child)
    except ValueError:
        pass


def _child_text(element, tag, default=""):
    child = element.find(tag)
    return child.text.strip() if child is not None and child.text else default


def _set_child_text(element, tag, value):
    child = element.find(tag)
    if child is None:
        child = ET.SubElement(element, tag)
    child.text = str(value)
    return child


def _continuous_track_translation_period(plugin):
    pitch_diameter = float(_child_text(plugin.find("sprocket"), "pitch_diameter", "0.24"))
    length = 0.7
    trajectory = plugin.find("trajectory")
    if trajectory is not None:
        for segment in trajectory.findall("segment"):
            joint = _child_text(segment, "joint")
            if "straight_segment" in joint:
                length = float(_child_text(segment, "end_position", str(length)))
                break
    elements_per_round = 40
    pattern = plugin.find("pattern")
    if pattern is not None:
        elements_per_round = int(float(_child_text(pattern, "elements_per_round", "40")))
    return (2.0 * length + 3.141592653589793 * pitch_diameter) / max(elements_per_round, 1)


def _convert_continuous_track_plugin(
        plugin, update_rate, segment_mode, translation_radius):
    track_name = plugin.get("name", "continuous_track")
    pitch_diameter = _child_text(plugin.find("sprocket"), "pitch_diameter", "0.24")
    translation_period = _continuous_track_translation_period(plugin)
    trajectory = plugin.find("trajectory")

    plugin.set("filename", "libIgnitionContinuousTrackSimple.so")
    plugin.set("name", "gazebo_continuous_track_ros2_ignition::IgnitionContinuousTrackSimple")
    _set_child_text(plugin, "track_name", track_name)

    for tag in ("trajectory", "pattern"):
        child = plugin.find(tag)
        if child is not None:
            plugin.remove(child)

    track = ET.SubElement(plugin, "track")
    if trajectory is not None:
        for old_segment in trajectory.findall("segment"):
            joint = _child_text(old_segment, "joint")
            segment = ET.SubElement(track, "segment")
            _set_child_text(segment, "joint", joint)
            if "arc_segment" in joint:
                _set_child_text(segment, "pitch_diameter", pitch_diameter)
            else:
                _set_child_text(segment, "translation_period", f"{translation_period:.9g}")
                _set_child_text(segment, "translation_radius", str(translation_radius))
    _set_child_text(plugin, "velocity_deadband", "0.02")
    _set_child_text(plugin, "update_segments", "true")
    _set_child_text(plugin, "update_rate", str(update_rate))
    _set_child_text(plugin, "segment_mode", segment_mode)


def _append_ignition_joint_controllers(root, flipper_pd):
    velocity_joints = [
        "sprocket_axle_left",
        "sprocket_axle_right",
        "flipper_sprocket_axle_left_front",
        "flipper_sprocket_axle_left_rear",
        "flipper_sprocket_axle_right_front",
        "flipper_sprocket_axle_right_rear",
    ]
    position_joints = [
        "joint_left_front",
        "joint_left_rear",
        "joint_right_front",
        "joint_right_rear",
    ]

    gazebo = ET.SubElement(root, "gazebo")
    ET.SubElement(
        gazebo,
        "plugin",
        {
            "filename": "ignition-gazebo-joint-state-publisher-system",
            "name": "gz::sim::systems::JointStatePublisher",
        },
    )

    for joint in velocity_joints:
        gazebo = ET.SubElement(root, "gazebo")
        plugin = ET.SubElement(
            gazebo,
            "plugin",
            {
                "filename": "ignition-gazebo-joint-controller-system",
                "name": "gz::sim::systems::JointController",
            },
        )
        _set_child_text(plugin, "joint_name", joint)
        _set_child_text(plugin, "initial_velocity", "0.0")

    for joint in position_joints:
        gazebo = ET.SubElement(root, "gazebo")
        plugin = ET.SubElement(
            gazebo,
            "plugin",
            {
                "filename": "ignition-gazebo-joint-position-controller-system",
                "name": "gz::sim::systems::JointPositionController",
            },
        )
        _set_child_text(plugin, "joint_name", joint)
        _set_child_text(plugin, "topic", f"/model/crawler/joint/{joint}/0/cmd_pos")
        _set_child_text(plugin, "p_gain", str(flipper_pd["kp"]))
        _set_child_text(plugin, "i_gain", "0")
        _set_child_text(plugin, "d_gain", str(flipper_pd["kd"]))
        _set_child_text(plugin, "cmd_max", str(flipper_pd["effort"]))
        _set_child_text(plugin, "cmd_min", str(-flipper_pd["effort"]))


def _reverse_right_flipper_joint_axes(root):
    right_flipper_joints = {"joint_right_front", "joint_right_rear"}
    for joint in root.findall("joint"):
        if joint.get("name") not in right_flipper_joints:
            continue
        axis = joint.find("axis")
        if axis is None:
            continue
        values = axis.get("xyz", "").split()
        if len(values) != 3:
            continue
        try:
            reversed_axis = [-float(value) for value in values]
        except ValueError:
            continue
        axis.set(
            "xyz",
            " ".join(f"{0.0 if abs(value) < 1e-12 else value:g}" for value in reversed_axis),
        )


def _make_ignition_robot_description(
        robot_description, flipper_pd, track_update_rate, track_segment_mode,
        track_contact_radius):
    root = ET.fromstring(_strip_xml_declaration(robot_description))

    # Keep a common command convention for all flippers: positive raises the
    # flipper.  The right-side joints in the source URDF use the opposite axis.
    _reverse_right_flipper_joint_axes(root)
    # Keep the discrete grouser collisions.  Replacing them with a smooth
    # envelope prevents the tread bars from engaging vertical obstacle edges.

    for parent in root.iter():
        for plugin in list(parent.findall("plugin")):
            filename = plugin.get("filename", "")
            name = plugin.get("name", "")
            if filename in {
                "libgazebo_ros2_control.so",
                "libgazebo_ros_imu_sensor.so",
                "libcrawler_gazebo_drive.so",
            } or name == "gazebo_ros2_control":
                _remove_element(parent, plugin)

    for gazebo in list(root.findall("gazebo")):
        for plugin in list(gazebo.findall("plugin")):
            filename = plugin.get("filename", "")
            name = plugin.get("name", "")
            if filename == "libContinuousTrack.so":
                _convert_continuous_track_plugin(
                    plugin, track_update_rate, track_segment_mode, track_contact_radius)
        if len(gazebo) == 0 and not gazebo.attrib:
            _remove_element(root, gazebo)

    ros2_control = root.find("ros2_control")
    if ros2_control is not None:
        _remove_element(root, ros2_control)

    _append_ignition_joint_controllers(root, flipper_pd)
    return ET.tostring(root, encoding="unicode")


def _write_spawn_urdf(xml, name):
    path = Path(tempfile.gettempdir()) / f"crawler_gazebo_ignition_{name}.urdf"
    path.write_text(xml, encoding="utf-8")
    return path


def _launch_setup(context, *args, **kwargs):
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    crawler_description_share = Path(get_package_share_directory("crawler_description"))
    ignition_track_lib = _optional_package_lib("gazebo_continuous_track_ros2_ignition")
    simsetting_yaml = _resolve_package_file(
        LaunchConfiguration("simsetting_yaml").perform(context), crawler_gazebo_share, "config")
    settings = _read_yaml(simsetting_yaml)

    robot_size = LaunchConfiguration("robot_size").perform(context)
    robot_config_yaml = crawler_gazebo_share / "config" / "robot" / f"{robot_size}.yaml"
    robot_config = _read_yaml(robot_config_yaml).get("robot_config", {})
    flipper_pd = {
        "kp": float(_setting(robot_config, "joints.flipper_position_kp", 220.0)),
        "kd": float(_setting(robot_config, "joints.flipper_position_kd", 70.0)),
        "effort": float(_setting(robot_config, "joints.flipper_effort", 8000.0)),
    }
    robot_urdf_arg = LaunchConfiguration("robot_urdf").perform(context)
    robot_urdf = Path(robot_urdf_arg) if robot_urdf_arg else crawler_gazebo_share / "urdf" / f"{robot_size}_crawler.urdf"
    grouser_shape = str(_setting(
        settings, "crawler_gazebo.simulator.grouser_shape", "auto")).lower()
    valid_grouser_shapes = {
        "auto", "rectangle", "trapezoid", "spike",
        "semicircle", "fillet",
    }
    if grouser_shape not in valid_grouser_shapes:
        raise ValueError(
            "crawler_gazebo.simulator.grouser_shape must be auto, rectangle, "
            "trapezoid, spike, semicircle, or fillet")
    # An explicit robot_urdf always wins. Otherwise a simsetting override is
    # generated into /tmp so the checked-in/default generated URDF is untouched.
    if not robot_urdf_arg and grouser_shape != "auto":
        generator = (
            Path(get_package_prefix("crawler_gazebo"))
            / "lib" / "crawler_gazebo" / "generate_crawler_urdf.py")
        generated_urdf = (
            Path(tempfile.gettempdir())
            / f"crawler_gazebo_{robot_size}_{grouser_shape}.urdf")
        subprocess.check_call([
            str(generator), "--no-gui",
            "--config", str(robot_config_yaml),
            "--output", str(generated_urdf),
            "--set", f"continuous_track.grouser_shape={grouser_shape}",
        ])
        robot_urdf = generated_urdf
    model = LaunchConfiguration("model").perform(context)
    max_step_size = float(LaunchConfiguration("max_step_size").perform(context))
    real_time_update_rate = float(LaunchConfiguration("real_time_update_rate").perform(context))
    physics_engine = LaunchConfiguration("physics_engine").perform(context).lower()
    if physics_engine == "auto":
        physics_engine = str(_setting(
            settings, "crawler_gazebo.simulator.physics_engine", "dart")).lower()
    if physics_engine not in {"dart", "bullet", "tpe"}:
        raise ValueError("physics_engine must be dart, bullet, or tpe")
    dart_collision_detector = str(_setting(
        settings, "crawler_gazebo.simulator.dart_collision_detector", "bullet")).lower()
    if dart_collision_detector not in {"bullet", "fcl", "ode", "dart"}:
        raise ValueError("dart_collision_detector must be bullet, fcl, ode, or dart")
    worldfile_value = LaunchConfiguration("worldfile").perform(context)
    if worldfile_value == "auto":
        worldfile_value = str(_setting(
            settings, "crawler_gazebo.simulator.worldfile", "base_fields.world"))
    arena_yaml_value = LaunchConfiguration("arena_yaml").perform(context)
    if arena_yaml_value == "auto":
        arena_yaml_value = str(_setting(
            settings, "crawler_gazebo.simulator.arena_yaml", "singlerane/bridge.yaml"))
    arena_yaml_path = _resolve_package_file(
        arena_yaml_value, crawler_gazebo_share, "config/gazebo_environment")
    worldfile = _make_ignition_world_file(
        _resolve_package_file(
            worldfile_value, crawler_gazebo_share, "world"),
        max_step_size,
        real_time_update_rate,
        physics_engine,
        dart_collision_detector,
    )
    spawn_robot = _as_bool(_setting(settings, "crawler_gazebo.simulator.spawn_robot", True))
    if spawn_robot and physics_engine != "dart":
        print(
            f"[crawler_gazebo] WARNING: physics_engine={physics_engine} can load the world, "
            "but this Fortress gz-physics backend does not fully support the crawler's "
            "articulated prismatic/revolute track joints. Use dart for driven simulation.")
    spawn_arena_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.spawn_arena", True))
    start_gui_tools_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.start_gui_tools", True))
    start_flipper_joint_gui_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.start_flipper_joint_gui", True))
    start_cloudmap_publisher_setting = _as_bool(_setting(settings, "crawler_gazebo.simulator.start_cloudmap_publisher", True))

    norobot = _as_bool(LaunchConfiguration("norobot").perform(context))
    spawn_arena = _as_bool(LaunchConfiguration("spawn_arena").perform(context)) and spawn_arena_setting
    start_gui_tools = _as_bool(LaunchConfiguration("start_gui_tools").perform(context)) and start_gui_tools_setting
    start_flipper_joint_gui = (
        _as_bool(LaunchConfiguration("start_flipper_joint_gui").perform(context))
        and start_flipper_joint_gui_setting
    )
    start_cloudmap_publisher = (
        _as_bool(LaunchConfiguration("start_cloudmap_publisher").perform(context))
        and start_cloudmap_publisher_setting
    )
    start_rviz = _as_bool(LaunchConfiguration("rviz").perform(context))
    initial_flipper_angle = float(LaunchConfiguration("initial_flipper_angle").perform(context))
    use_generated_robot_urdf = _as_bool(LaunchConfiguration("use_generated_robot_urdf").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    ign_partition = LaunchConfiguration("ign_partition").perform(context)
    if ign_partition == "auto":
        ign_partition = f"crawler_gazebo_{os.getpid()}"

    def spawn_value(name, default):
        launch_value = LaunchConfiguration(f"spawn_{name}").perform(context)
        if launch_value != "auto":
            return launch_value
        return str(_setting(settings, f"crawler_gazebo.simulator.spawn_pose.{name}", default))

    spawn_x = spawn_value("x", 0.0)
    spawn_y = spawn_value("y", 0.0)
    spawn_z = spawn_value("z", "auto")
    if spawn_z == "auto":
        spawn_z = _default_spawn_z(crawler_gazebo_share, robot_size)
    spawn_roll = math.radians(float(spawn_value("roll", 0.0)))
    spawn_pitch = math.radians(float(spawn_value("pitch", 0.0)))
    spawn_yaw = math.radians(float(spawn_value("yaw", 0.0)))
    spawn_orientation = _quaternion_from_euler(spawn_roll, spawn_pitch, spawn_yaw)
    track_update_rate = float(
        _setting(settings, "crawler_gazebo.simulator.track_update_rate", 200.0))
    track_contact_radius_arg = LaunchConfiguration("track_contact_radius").perform(context)
    if track_contact_radius_arg == "auto":
        track_contact_radius = (
            float(_setting(robot_config, "geometry.wheel_radius", 0.082))
            + float(_setting(robot_config, "continuous_track.belt_thickness", 0.02))
            + 0.5 * float(_setting(robot_config, "continuous_track.grouser_height", 0.018))
        )
    else:
        track_contact_radius = float(track_contact_radius_arg)
    track_segment_mode = LaunchConfiguration("track_segment_mode").perform(context)
    if track_segment_mode == "auto":
        track_segment_mode = str(_setting(
            settings, "crawler_gazebo.simulator.track_segment_mode", "all"))
    if track_segment_mode not in {"all", "arc_only", "straight_only"}:
        raise ValueError(
            "track_segment_mode must be all, arc_only, or straight_only")
    cloud_resolution = float(_setting(settings, "gazebo_to_octomap_publisher.resolution", 0.025))

    ign_cmd = ["ign", "gazebo"]
    if not _as_bool(LaunchConfiguration("gui").perform(context)):
        ign_cmd.append("-s")
    else:
        render_engine_gui = LaunchConfiguration("render_engine_gui").perform(context)
        if render_engine_gui:
            ign_cmd.extend(["--render-engine-gui", render_engine_gui])
    ign_cmd.append(str(worldfile))
    if not _as_bool(LaunchConfiguration("paused").perform(context)):
        ign_cmd.append("-r")

    ignition_plugin_paths = [
        path for path in [
            ignition_track_lib,
            os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
        ]
        if path
    ]

    actions = [
        SetEnvironmentVariable(
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
            ":".join(ignition_plugin_paths),
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            f"{crawler_gazebo_share / 'gazebo_model' / 'model'}:{crawler_gazebo_share / 'gazebo_model'}:{crawler_gazebo_share.parent}:{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}",
        ),
        SetEnvironmentVariable("IGN_PARTITION", ign_partition),
        ExecuteProcess(cmd=ign_cmd, output="screen"),
    ]
    world_name = _world_name(worldfile)

    if spawn_arena:
        actions.extend(
            _arena_spawn_actions(
                arena_yaml_path,
                crawler_gazebo_share / "gazebo_model" / "model",
                world_name,
                3.0,
                _as_bool(LaunchConfiguration("use_wall_arg").perform(context)),
                LaunchConfiguration("wall_sdf").perform(context),
            )
        )

    if spawn_robot and not norobot:
        if use_generated_robot_urdf:
            robot_description_raw = robot_urdf.read_text(encoding="utf-8")
        else:
            robot_description_raw = _xacro_to_string(
                model,
                {
                    "enable_body_tracks": LaunchConfiguration("enable_body_tracks").perform(context),
                    "enable_flipper_tracks": LaunchConfiguration("enable_flipper_tracks").perform(context),
                    "enable_gazebo_ros_control": "false",
                    "enable_imu": LaunchConfiguration("enable_imu").perform(context),
                    "enable_position_transmissions": LaunchConfiguration("enable_position_transmissions").perform(context),
                    "enable_velocity_transmissions": LaunchConfiguration("enable_velocity_transmissions").perform(context),
                    "enable_body_velocity_transmissions": LaunchConfiguration("enable_body_velocity_transmissions").perform(context),
                    "enable_flipper_velocity_transmissions": LaunchConfiguration("enable_flipper_velocity_transmissions").perform(context),
                },
            )
        robot_description = _make_ignition_robot_description(
            robot_description_raw, flipper_pd, track_update_rate, track_segment_mode,
            track_contact_radius)
        spawn_urdf = _write_spawn_urdf(robot_description, robot_urdf.stem)
        request = (
            f'sdf_filename: "{spawn_urdf}", '
            'name: "crawler", '
            f'pose: {{position: {{x: {spawn_x}, y: {spawn_y}, z: {spawn_z}}}, '
            f'orientation: {{w: {spawn_orientation["w"]}, x: {spawn_orientation["x"]}, '
            f'y: {spawn_orientation["y"]}, z: {spawn_orientation["z"]}}}}}'
        )

        actions.extend([
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="log",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
                remappings=[("joint_states", "/crawler/joint_states")],
            ),
            TimerAction(
                period=5.5 if spawn_arena else 3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ign",
                            "service",
                            "-s",
                            f"/world/{world_name}/create",
                            "--reqtype",
                            "ignition.msgs.EntityFactory",
                            "--reptype",
                            "ignition.msgs.Boolean",
                            "--timeout",
                            "10000",
                            "--req",
                            request,
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="crawler_gazebo",
                executable="crawler_ignition_control_bridge",
                name="crawler_ignition_control_bridge",
                output="screen",
                parameters=[
                    {
                        "model_name": "crawler",
                        "cmd_vel_topic": "/target/cmd_vel",
                        "joint_state_topic": "/target/joint_states",
                        "limit_cmd_vel": False,
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "model_pose_topic": f"/world/{world_name}/dynamic_pose/info",
                        "model_joint_state_topic":
                            f"/world/{world_name}/model/crawler/joint_state",
                        "initial_flipper_angle": initial_flipper_angle,
                        "joint_state_output_topic": "/crawler/joint_states",
                    }
                ],
            ),
        ])

        if start_gui_tools:
            actions.append(
                Node(
                    package="rqt_robot_steering",
                    executable="rqt_robot_steering",
                    name="rqt_robot_steering",
                    output="log",
                    remappings=[("/cmd_vel", "/target/cmd_vel")],
                )
            )

        if start_flipper_joint_gui:
            flipper_model = crawler_description_share / "xacro" / "flipper_joint_gui.xacro"
            try:
                flipper_description = _xacro_to_string(str(flipper_model), {})
                actions.append(
                    Node(
                        package="joint_state_publisher_gui",
                        executable="joint_state_publisher_gui",
                        name="flipper_joint_state_publisher_gui",
                        output="screen",
                        parameters=[{
                            "robot_description": flipper_description,
                            "use_sim_time": False,
                            "zeros.joint_left_front": initial_flipper_angle,
                            "zeros.joint_left_rear": initial_flipper_angle,
                            "zeros.joint_right_front": initial_flipper_angle,
                            "zeros.joint_right_rear": initial_flipper_angle,
                        }],
                        remappings=[("joint_states", "/target/joint_states")],
                    )
                )
            except Exception as exc:
                print(f"[crawler_gazebo] flipper_joint_gui xacro skipped: {exc}")

    if start_cloudmap_publisher:
        actions.extend([
            Node(
                package="crawler_gazebo",
                executable="gazebo_to_octomap_publisher",
                name="gazebo_to_octomap_publisher",
                output="screen",
                parameters=[
                    {
                        "arena_yaml": str(arena_yaml_path),
                        "model_root": str(crawler_gazebo_share / "gazebo_model" / "model"),
                        "use_gazebo_services": False,
                        "use_sim_time": use_sim_time,
                        "resolution": cloud_resolution,
                        "plane_step": cloud_resolution,
                    },
                ],
            ),
            Node(
                package="crawler_gazebo",
                executable="voxel_overhang_removal",
                name="voxel_overhang_removal",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/octomap_pointcloud",
                        "output_topic": "/octomap_pointcloud/filtering",
                        "voxel_size": 0.05,
                        "height_mode": "max",
                        "durability": "transient_local",
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ])

    if start_rviz:
        rviz_config = _resolve_package_file(
            LaunchConfiguration("rviz_config").perform(context),
            crawler_gazebo_share,
            "launch/rviz",
        )
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
            )
        )

    return actions


def generate_launch_description():
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    crawler_description_share = Path(get_package_share_directory("crawler_description"))

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=str(crawler_description_share / "xacro" / "crawler_track.xacro")),
        DeclareLaunchArgument("robot_size", default_value="default"),
        DeclareLaunchArgument("robot_urdf", default_value=""),
        DeclareLaunchArgument("use_generated_robot_urdf", default_value="true"),
        DeclareLaunchArgument("worldfile", default_value="auto"),
        DeclareLaunchArgument("simsetting_yaml", default_value="simsetting.yaml"),
        DeclareLaunchArgument("arena_yaml", default_value="auto"),
        DeclareLaunchArgument("spawn_arena", default_value="true"),
        DeclareLaunchArgument("use_wall_arg", default_value="false"),
        DeclareLaunchArgument("wall_sdf", default_value=""),
        DeclareLaunchArgument("norobot", default_value="false"),
        DeclareLaunchArgument("enable_body_tracks", default_value="true"),
        DeclareLaunchArgument("enable_flipper_tracks", default_value="true"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("enable_position_transmissions", default_value="true"),
        DeclareLaunchArgument("enable_velocity_transmissions", default_value="true"),
        DeclareLaunchArgument("enable_body_velocity_transmissions", default_value="true"),
        DeclareLaunchArgument("enable_flipper_velocity_transmissions", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ign_partition", default_value="auto"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("render_engine_gui", default_value="ogre"),
        DeclareLaunchArgument("spawn_x", default_value="auto"),
        DeclareLaunchArgument("spawn_y", default_value="auto"),
        DeclareLaunchArgument("spawn_z", default_value="auto"),
        DeclareLaunchArgument("spawn_roll", default_value="auto"),
        DeclareLaunchArgument("spawn_pitch", default_value="auto"),
        DeclareLaunchArgument("spawn_yaw", default_value="auto"),
        DeclareLaunchArgument("track_segment_mode", default_value="auto"),
        DeclareLaunchArgument("track_contact_radius", default_value="auto"),
        DeclareLaunchArgument("physics_engine", default_value="auto"),
        DeclareLaunchArgument("max_step_size", default_value="0.0005"),
        DeclareLaunchArgument("real_time_update_rate", default_value="2000"),
        DeclareLaunchArgument("start_gui_tools", default_value="true"),
        DeclareLaunchArgument("start_flipper_joint_gui", default_value="true"),
        DeclareLaunchArgument("start_cloudmap_publisher", default_value="true"),
        DeclareLaunchArgument("initial_flipper_angle", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value="crawler.rviz"),
        OpaqueFunction(function=_launch_setup),
    ])
