# Crawler Gazebo

`crawler_gazebo` is a ROS 2 package for running the crawler robot in Gazebo Classic 11 or Ignition Gazebo. It contains launch files, robot/environment configuration, Gazebo models, world files, and point cloud utility nodes.

## Directory Layout

- `gazebo_model/model/`: Gazebo model directory. Each model contains `model.config`, `model.sdf`, and mesh or texture files such as STL, DAE, and images.
- `world/`: Gazebo world files. These define the ground plane, sun, and fixed models in SDF/world format.
- `config/gazebo_environment/`: Arena YAML files. These define which models are spawned after the world starts, together with their position, orientation, and scale.
- `config/robot/`: Robot YAML files for robot dimensions, masses, tracks, and joint limits. Available presets include `default.yaml`, `compact.yaml`, and `large.yaml`.
- `config/simsetting.yaml`: Global simulator switches and point cloud processing parameters.
- `launch/gazebo.launch.py`: Launch file for Gazebo Classic 11.
- `launch/gazebo_ignition.launch.py`: Launch file for Ignition Gazebo.
- `launch/model_pointcloud.launch.py`: Launch file for spawning only environment models and publishing a point cloud, without spawning the robot or ROS controllers.
- `launch/gazebo_cloudmap_publisher.launch.py`: Helper launch file for generating a point cloud map from a world.
- `scripts/ramdom_world.py`: Script that randomly places models from `gazebo_model/model/` into a generated world.

## Gazebo Model Files

Place models in this structure:

```text
gazebo_model/model/<model_name>/
  model.config
  model.sdf
  mesh.stl, mesh.dae, texture.png, ...
```

`model.config` stores Gazebo metadata such as the model name, version, and description. `model.sdf` defines the actual collision geometry, visual geometry, mesh URI, scale, friction, and related SDF properties.

A typical mesh reference in `model.sdf` looks like this:

```xml
<mesh>
  <scale>0.5 0.5 0.5</scale>
  <uri>model://30cmstep/30cmstep.stl</uri>
</mesh>
```

In `model://30cmstep/...`, `30cmstep` corresponds to `gazebo_model/model/30cmstep/`. The launch files add this model directory to `GAZEBO_MODEL_PATH` or `IGN_GAZEBO_RESOURCE_PATH` so Gazebo can resolve `model://` URIs.

Note: Ignition Gazebo with DART physics does not accept mesh scale components that are zero or negative. Some older Gazebo Classic models use mirrored scales such as `-0.01 0.01 0.01`; the Ignition launch file creates temporary SDF files with positive mesh scales for those models.

Continuous track simulation plugins are optional for building `crawler_gazebo` itself. If `gazebo_continuous_track_ros2_gazebo11` or `gazebo_continuous_track_ros2_ignition` is installed, the launch files add its `lib` directory to the Gazebo plugin path. If it is not installed, model-only and point-cloud workflows still work, but robot simulations that rely on `libContinuousTrack.so` or `libIgnitionContinuousTrackSimple.so` need the corresponding plugin package at runtime.

## YAML Files

### `config/gazebo_environment/*.yaml`

Arena YAML files define obstacles and terrain models that are spawned after the world starts.

```yaml
robocup_arena:
  objects:
    30cmstep_1: { x: -1.5, y: -1.5, z: 0.0, roll: 0, pitch: 0, yaw: 0 }
    pallet30_1: { x: 3.5, y: 1.5, z: 0.0, roll: 0, pitch: 0, yaw: -90.0 }
    stepfield_1: { x: 1.0, y: 0.0, z: 0.0, roll: 0, pitch: 0, yaw: 0, scale_x: 1.0, scale_y: 1.0, scale_z: 1.0 }
```

Object keys should use the form `<model_name>_<number>`. The `<model_name>` part must match `gazebo_model/model/<model_name>/model.sdf`.

- `x`, `y`, `z`: Spawn position in world coordinates.
- `roll`, `pitch`, `yaw`: Spawn orientation in degrees.
- `scale_x`, `scale_y`, `scale_z`: Multipliers applied to the mesh scales in `model.sdf`. Defaults to `1.0` when omitted.

Examples:

- `config/gazebo_environment/benchmark.yaml`: Benchmark environment with several representative obstacles.
- `config/gazebo_environment/singlerane/step.yaml`: Single-lane step environment.
- `config/gazebo_environment/singlerane/stair.yaml`: Stair environment.

### `config/robot/*.yaml`

Robot YAML files define the robot shape and physical parameters. Select one with a launch argument such as `robot_size:=compact`.

- `general`: Robot name, nonholonomic flag, flipper/IMU/Gazebo control enable flags.
- `geometry`: Body, track, flipper, and wheel dimensions.
- `mass`: Mass values for the base, tracks, sprockets, and flippers.
- `joints`: Flipper limits, effort, PID gains, and sprocket velocity limits.
- `continuous_track`: Track pitch diameter, belt thickness, friction, and contact parameters.
- `output.urdf`: Output path for the generated URDF.

### `config/simsetting.yaml`

`simsetting.yaml` controls default launch behavior and point cloud processing settings.

- `crawler_gazebo.simulator.spawn_robot`: Whether to spawn the robot.
- `crawler_gazebo.simulator.spawn_arena`: Whether to spawn models from the arena YAML.
- `start_controllers`, `start_control_nodes`: Whether to start controllers and control helper nodes.
- `start_gui_tools`, `start_flipper_joint_gui`: Whether to start GUI control tools.
- `start_cloudmap_publisher`: Whether to publish a point cloud map generated from Gazebo models.
- `gazebo_to_octomap_publisher.resolution`: Point cloud generation resolution.
- `gazebo_to_octomap_publisher_gap_filter`: Voxel, gap, outlier, and overhang filter settings.
- `gridmap_publisher`: Grid map resolution, frame, and CSV output path.

## Launch

Build and source the workspace:

```bash
cd $HOME/CrawlerRobotSimulation/ros2
colcon build --packages-select crawler_gazebo
source install/setup.bash
```

Gazebo Classic 11:

```bash
ros2 launch crawler_gazebo gazebo.launch.py
```

Ignition Gazebo:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py
```

You can also launch directly from the source tree before installing:

```bash
ros2 launch $HOME/CrawlerRobotSimulation/ros2/src/Gazebo/crawler_gazebo/launch/gazebo.launch.py
ros2 launch $HOME/CrawlerRobotSimulation/ros2/src/Gazebo/crawler_gazebo/launch/gazebo_ignition.launch.py
```

If the Ignition GUI opens as a blank white window, use the OGRE1 GUI renderer:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py render_engine_gui:=ogre
```

The default value of `render_engine_gui` is `ogre` because some machines fail to initialize EGL with the Ignition default `ogre2` renderer. To try the original default:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py render_engine_gui:=ogre2
```

## Model-Only Point Cloud Launch

Use `model_pointcloud.launch.py` when you want Gazebo to spawn only environment models and publish a point cloud map, without spawning the robot and without starting ROS controllers.

Ignition Gazebo:

```bash
ros2 launch crawler_gazebo model_pointcloud.launch.py
```

Gazebo Classic 11:

```bash
ros2 launch crawler_gazebo model_pointcloud.launch.py simulator:=gazebo11
```

Switch the arena YAML:

```bash
ros2 launch crawler_gazebo model_pointcloud.launch.py \
  arena_yaml:=singlerane/stair.yaml
```

Use a world that already contains models and disable additional arena spawning:

```bash
ros2 launch crawler_gazebo model_pointcloud.launch.py \
  worldfile:=random.world \
  spawn_arena:=false
```

This launch internally sets:

- `norobot:=true`
- `start_gui_tools:=false`
- `start_flipper_joint_gui:=false`
- `start_cloudmap_publisher:=true`
- `start_controllers:=false` and `start_control_nodes:=false` for Gazebo Classic 11

## World Switching

Use `worldfile:=...` to switch the base world.

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py \
  worldfile:=base_fields.world
```

Example using another world:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py \
  worldfile:=random.world
```

`worldfile` is the base Gazebo world. When `spawn_arena:=true`, models from `arena_yaml` are spawned after the world starts.

Example switching the arena YAML:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py \
  arena_yaml:=singlerane/stair.yaml
```

To use only the models already included in the world and disable additional arena spawning:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py \
  worldfile:=random.world \
  spawn_arena:=false
```

To inspect the environment without spawning the robot:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py norobot:=true
```

To run without the Gazebo GUI:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py gui:=false
```

Gazebo Classic 11 accepts the same `worldfile` and `arena_yaml` arguments.

```bash
ros2 launch crawler_gazebo gazebo.launch.py \
  worldfile:=base_fields.world \
  arena_yaml:=benchmark.yaml
```

## Robot Size Switching

Select the robot configuration with `robot_size`.

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py robot_size:=compact
ros2 launch crawler_gazebo gazebo_ignition.launch.py robot_size:=default
ros2 launch crawler_gazebo gazebo_ignition.launch.py robot_size:=large
```

When `spawn_z:=auto`, the initial spawn height is computed from `wheel_radius`, `belt_thickness`, and `grouser_height` in `config/robot/<robot_size>.yaml`.

## Random World Generation

Generate a world with randomly placed models:

```bash
cd $HOME/CrawlerRobotSimulation/ros2/src
Gazebo/crawler_gazebo/scripts/ramdom_world.py --seed 42 -n 20 --area-size 12 --min-distance 1.4
```

The default output path is `Gazebo/crawler_gazebo/world/random.world`. To launch the generated world:

```bash
ros2 launch crawler_gazebo gazebo_ignition.launch.py \
  worldfile:=random.world \
  spawn_arena:=false
```

## Point Cloud Utilities

Main point cloud nodes:

- `gazebo_to_octomap_publisher`: Publishes `/loaded_pointcloud` and related point cloud topics from Gazebo models.
- `voxel_overhang_removal`: Converts `/octomap_pointcloud` into a 5 cm voxelized 2.5D point cloud on `/octomap_pointcloud/filtering`. The Gazebo launch files start this filter automatically whenever `start_cloudmap_publisher:=true`.
- `cloudmap_to_pcd`: Subscribes to `/loaded_pointcloud` and saves it as a PCD file.

Examples:

```bash
ros2 run crawler_gazebo voxel_overhang_removal --ros-args \
  -p input_topic:=/octomap_pointcloud \
  -p output_topic:=/octomap_pointcloud/filtering \
  -p voxel_size:=0.05

ros2 run crawler_gazebo cloudmap_to_pcd --ros-args \
  -p input_topic:=/loaded_pointcloud \
  -p output_path:=package://crawler_gazebo/pcd/cloudmap.pcd
```
