To start the person detection modify and run: python depthai_combination.py
To start the rtabmap run source /opt/ros/humble/setup.bash and ros2 launch depthai_ros_driver rtabmap.launch.py 

# Person detection

Run ``python depthai_combination.py``.

# Rtabmap

On the laptop run: ``ros2 launch /WORKSPACE/thesis/src/rtabmap_custom.launch.py``.
On the robot run: ``ros2 launch depthai_ros_driver camera.launch.py params_file:=/opt/ros/humble/share/depthai_ros_driver/config/spatial_yolo.yaml``
