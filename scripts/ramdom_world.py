#!/usr/bin/env python3
import argparse
import math
import random
import sys
import xml.dom.minidom
import xml.etree.ElementTree as ET
from pathlib import Path


DEFAULT_EXCLUDES = {
    "default_plane",
    "default_slope",
    "origin",
    "sin_wave",
}


def _script_root():
    return Path(__file__).resolve().parents[1]


def _split_csv(value):
    if not value:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


def _has_nonpositive_mesh_scale(model_sdf):
    try:
        root = ET.parse(model_sdf).getroot()
    except ET.ParseError:
        return True

    for scale in root.findall(".//mesh/scale"):
        if not scale.text:
            continue
        try:
            values = [float(part) for part in scale.text.split()]
        except ValueError:
            return True
        if any(value <= 0.0 for value in values):
            return True
    return False


def _available_models(model_dir, allow_nonpositive_scale=False):
    models = []
    skipped = []
    for model_sdf in sorted(model_dir.glob("*/model.sdf")):
        name = model_sdf.parent.name
        if name in DEFAULT_EXCLUDES:
            continue
        if not allow_nonpositive_scale and _has_nonpositive_mesh_scale(model_sdf):
            skipped.append(name)
            continue
        models.append(name)
    return models, skipped


def _pose_text(x, y, z, yaw):
    return f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.6f}"


def _add_text_child(parent, tag, text):
    child = ET.SubElement(parent, tag)
    child.text = str(text)
    return child


def _add_ground_plane(world, size):
    model = ET.SubElement(world, "model", {"name": "ground_plane"})
    _add_text_child(model, "static", "true")
    link = ET.SubElement(model, "link", {"name": "link"})

    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(element, "geometry")
        plane = ET.SubElement(geometry, "plane")
        _add_text_child(plane, "normal", "0 0 1")
        _add_text_child(plane, "size", f"{size:.1f} {size:.1f}")

        if tag == "collision":
            surface = ET.SubElement(element, "surface")
            friction = ET.SubElement(surface, "friction")
            ode = ET.SubElement(friction, "ode")
            _add_text_child(ode, "mu", "1.0")
            _add_text_child(ode, "mu2", "1.0")
        else:
            _add_text_child(element, "cast_shadows", "false")
            material = ET.SubElement(element, "material")
            script = ET.SubElement(material, "script")
            _add_text_child(script, "uri", "file://media/materials/scripts/gazebo.material")
            _add_text_child(script, "name", "Gazebo/Grey")
            _add_text_child(material, "ambient", "0.55 0.55 0.55 1")
            _add_text_child(material, "diffuse", "0.55 0.55 0.55 1")
            _add_text_child(material, "specular", "0.0 0.0 0.0 1")
            _add_text_child(material, "emissive", "0.0 0.0 0.0 1")


def _add_sun(world):
    include = ET.SubElement(world, "include")
    _add_text_child(include, "uri", "model://sun")


def _random_position(rng, half_extent, clear_radius, placed, min_distance, max_attempts=200):
    for _ in range(max_attempts):
        x = rng.uniform(-half_extent, half_extent)
        y = rng.uniform(-half_extent, half_extent)
        if math.hypot(x, y) < clear_radius:
            continue
        if all(math.hypot(x - px, y - py) >= min_distance for px, py in placed):
            return x, y
    return None


def _build_world(args, models):
    rng = random.Random(args.seed)
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": args.world_name})

    _add_ground_plane(world, args.ground_size)
    _add_sun(world)

    half_extent = args.area_size * 0.5
    placed = []
    for index in range(args.count):
        position = _random_position(
            rng,
            half_extent,
            args.clear_radius,
            placed,
            args.min_distance,
        )
        if position is None:
            print(
                f"warning: placed {len(placed)} models; no more free space found",
                file=sys.stderr,
            )
            break

        model = rng.choice(models)
        yaw = rng.uniform(args.yaw_min, args.yaw_max)
        x, y = position
        placed.append(position)

        include = ET.SubElement(world, "include")
        _add_text_child(include, "uri", f"model://{model}")
        _add_text_child(include, "name", f"{model}_{index + 1:02d}")
        _add_text_child(include, "pose", _pose_text(x, y, args.z, yaw))

    return sdf, placed


def _write_xml(root, output):
    rough = ET.tostring(root, encoding="utf-8")
    pretty = xml.dom.minidom.parseString(rough).toprettyxml(indent="  ")
    lines = [line for line in pretty.splitlines() if line.strip()]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _parse_args():
    package_root = _script_root()
    parser = argparse.ArgumentParser(
        description="Generate a Gazebo world with randomly placed model:// includes."
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=package_root / "world" / "random.world",
        help="output world path",
    )
    parser.add_argument(
        "--model-dir",
        type=Path,
        default=package_root / "gazebo_model" / "model",
        help="directory containing Gazebo model folders",
    )
    parser.add_argument("-n", "--count", type=int, default=20, help="number of models")
    parser.add_argument("--seed", type=int, default=None, help="random seed")
    parser.add_argument("--area-size", type=float, default=12.0, help="square placement area size [m]")
    parser.add_argument("--ground-size", type=float, default=100.0, help="ground plane size [m]")
    parser.add_argument("--min-distance", type=float, default=1.4, help="minimum XY distance between models [m]")
    parser.add_argument("--clear-radius", type=float, default=1.8, help="keep this radius around the origin empty [m]")
    parser.add_argument("--z", type=float, default=0.0, help="model spawn height [m]")
    parser.add_argument("--yaw-min", type=float, default=-math.pi, help="minimum random yaw [rad]")
    parser.add_argument("--yaw-max", type=float, default=math.pi, help="maximum random yaw [rad]")
    parser.add_argument("--world-name", default="field_world", help="SDF world name")
    parser.add_argument(
        "--models",
        default="",
        help="comma-separated model names to sample from; default uses all valid models",
    )
    parser.add_argument(
        "--exclude",
        default="",
        help="comma-separated model names to exclude",
    )
    parser.add_argument(
        "--allow-nonpositive-scale",
        action="store_true",
        help="include models whose mesh scale contains zero or negative values",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    if args.count < 0:
        raise SystemExit("--count must be non-negative")
    if args.area_size <= 0.0:
        raise SystemExit("--area-size must be positive")
    if args.min_distance < 0.0:
        raise SystemExit("--min-distance must be non-negative")

    available, skipped = _available_models(args.model_dir, args.allow_nonpositive_scale)
    requested = _split_csv(args.models)
    excludes = set(_split_csv(args.exclude))

    if requested:
        missing = sorted(set(requested) - set(available))
        if missing:
            raise SystemExit(f"unknown or skipped model(s): {', '.join(missing)}")
        models = requested
    else:
        models = available

    models = [model for model in models if model not in excludes]
    if not models and args.count > 0:
        raise SystemExit("no models available for random placement")

    world, placed = _build_world(args, models)
    _write_xml(world, args.output)

    print(f"generated: {args.output}")
    print(f"models: {len(placed)}")
    if args.seed is not None:
        print(f"seed: {args.seed}")
    if skipped:
        print("skipped non-positive mesh scale models: " + ", ".join(skipped))


if __name__ == "__main__":
    main()
