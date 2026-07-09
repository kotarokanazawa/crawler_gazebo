# crawler_gazebo robot config

`*.yaml` defines the crawler geometry, mass, joint limits, and continuous-track parameters used to generate static URDF files under `../../urdf`.

Generate URDF after editing a config:

```bash
rosrun crawler_gazebo generate_crawler_urdf.py \
  --config $(rospack find crawler_gazebo)/config/robot/default.yaml \
  --no-gui
```

Launch a generated robot size:

```bash
roslaunch crawler_gazebo gazebo.launch robot_size:=compact
roslaunch crawler_gazebo gazebo.launch robot_size:=default
roslaunch crawler_gazebo gazebo.launch robot_size:=large
```

You can also pass a specific URDF:

```bash
roslaunch crawler_gazebo gazebo.launch robot_urdf:=/path/to/crawler.urdf
```
