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


def _make_ignition_world_file(worldfile):
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


def _convert_continuous_track_plugin(plugin):
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
    _set_child_text(plugin, "velocity_deadband", "0.02")


def _append_ignition_joint_controllers(root):
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
        _set_child_text(plugin, "p_gain", "1200")
        _set_child_text(plugin, "i_gain", "0")
        _set_child_text(plugin, "d_gain", "80")
        _set_child_text(plugin, "cmd_max", "20000")
        _set_child_text(plugin, "cmd_min", "-20000")


def _make_ignition_robot_description(robot_description):
    root = ET.fromstring(_strip_xml_declaration(robot_description))

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
                _convert_continuous_track_plugin(plugin)
        if len(gazebo) == 0 and not gazebo.attrib:
            _remove_element(root, gazebo)

    ros2_control = root.find("ros2_control")
    if ros2_control is not None:
        _remove_element(root, ros2_control)

    _append_ignition_joint_controllers(root)
    return ET.tostring(root, encoding="unicode")


def _write_spawn_urdf(xml, name):
    path = Path(tempfile.gettempdir()) / f"crawler_gazebo_ignition_{name}.urdf"
    path.write_text(xml, encoding="utf-8")
    return path


def _launch_setup(context, *args, **kwargs):
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    crawler_description_share = Path(get_package_share_directory("crawler_description"))
    ignition_track_lib = _optional_package_lib("gazebo_continuous_track_ros2_ignition")

    robot_size = LaunchConfiguration("robot_size").perform(context)
    robot_urdf_arg = LaunchConfiguration("robot_urdf").perform(context)
    robot_urdf = Path(robot_urdf_arg) if robot_urdf_arg else crawler_gazebo_share / "urdf" / f"{robot_size}_crawler.urdf"
    model = LaunchConfiguration("model").perform(context)
    worldfile = _make_ignition_world_file(
        _resolve_package_file(LaunchConfiguration("worldfile").perform(context), crawler_gazebo_share, "world"))
    simsetting_yaml = _resolve_package_file(
        LaunchConfiguration("simsetting_yaml").perform(context), crawler_gazebo_share, "config")
    settings = _read_yaml(simsetting_yaml)

    spawn_robot = _as_bool(_setting(settings, "crawler_gazebo.simulator.spawn_robot", True))
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
    use_generated_robot_urdf = _as_bool(LaunchConfiguration("use_generated_robot_urdf").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    ign_partition = LaunchConfiguration("ign_partition").perform(context)
    if ign_partition == "auto":
        ign_partition = f"crawler_gazebo_{os.getpid()}"
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    if spawn_z == "auto":
        spawn_z = _default_spawn_z(crawler_gazebo_share, robot_size)
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
                _resolve_package_file(
                    LaunchConfiguration("arena_yaml").perform(context),
                    crawler_gazebo_share,
                    "config/gazebo_environment"),
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
        robot_description = _make_ignition_robot_description(robot_description_raw)
        spawn_urdf = _write_spawn_urdf(robot_description, robot_urdf.stem)
        request = (
            f'sdf_filename: "{spawn_urdf}", '
            'name: "crawler", '
            f'pose: {{position: {{x: 0, y: 0, z: {spawn_z}}}, orientation: {{w: 1, x: 0, y: 0, z: 0}}}}'
        )

        actions.extend([
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="log",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
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
                        parameters=[{"robot_description": flipper_description, "use_sim_time": False}],
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
                        "arena_yaml": str(_resolve_package_file(
                            LaunchConfiguration("arena_yaml").perform(context),
                            crawler_gazebo_share,
                            "config/gazebo_environment")),
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

    return actions


def generate_launch_description():
    crawler_gazebo_share = Path(get_package_share_directory("crawler_gazebo"))
    crawler_description_share = Path(get_package_share_directory("crawler_description"))

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=str(crawler_description_share / "xacro" / "crawler_track.xacro")),
        DeclareLaunchArgument("robot_size", default_value="default"),
        DeclareLaunchArgument("robot_urdf", default_value=""),
        DeclareLaunchArgument("use_generated_robot_urdf", default_value="true"),
        DeclareLaunchArgument("worldfile", default_value="base_fields.world"),
        DeclareLaunchArgument("simsetting_yaml", default_value="simsetting.yaml"),
        DeclareLaunchArgument("arena_yaml", default_value="benchmark.yaml"),
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
        DeclareLaunchArgument("spawn_z", default_value="auto"),
        DeclareLaunchArgument("start_gui_tools", default_value="true"),
        DeclareLaunchArgument("start_flipper_joint_gui", default_value="true"),
        DeclareLaunchArgument("start_cloudmap_publisher", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
