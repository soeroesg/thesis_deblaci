# ROS wrapper

## Supported platforms

* ROS2 Humble on Linux

## Dependencies

ROS2 Humble framework:

* https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html

On the Turtlebot 4 install the Spectacular AI Python package from its wheel. It is available on our Google Drive folder. Select the wheel that supports the Python version of the robot.
```
pip install /PATH/TO/*.whl
```

## Build

Make sure to have your ROS environment sourced i.e. `ros2` on command line works and that you have OAK-D device connected

From this folder, run on the laptop and on Turtlebot 4:
```
colcon build
source install/setup.bash
```

## Run

Run this on the laptop:
```
ros2 lauch launch/mapping_rviz.py
```

In case you do not want to record the mapping, run this on the robot:
```
ros2 launch launch/mapping_turtlebot.py
```

In case you only want to record the mapping for further use (e.g creating a gsplat or training a nerf), but do not want SLAM, run this on the robot:
```
ros2 launch launch/mapping_turtlebot.py recordingFolder:=recordings
# OR
ros2 launch launch/mapping_turtlebot.py recordingFolder:=recordings recordingAndSlam:=false
```

In case you want to record the mapping and do SLAM simultaneously, run this on the robot:
```
ros2 launch launch/mapping_turtlebot.py recordingFolder:=recordings recordingAndSlam:=true
```
This way the SLAM will be a bit slower but it did not fail at all for me.

## Published topics by the node

- /slam/odometry
- /slam/keyframe
- /slam/left
- /tf
- /slam/pointcloud
- /slam/camera_info
