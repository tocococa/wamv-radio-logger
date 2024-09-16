# wamv-radio-logger

## Description

This is a small ROS2 Humble node used to log status data from a Ubiquiti RocketM5, or any other radio that supports ssh and dumps iwconfig data; and some GNSS/Odom combo topic.

## Instructions

Build dependencies with

```
rosdep install -i --from-path src --rosdistro humble -y

pip3 install -r requirements.txt
```

Build the package with `colcon build --packages-select wamv_logger` and source `source install/setup.bash`.

Launch with `ros2 run wamv_logger wamv_logger`


Don't forget to `source /opt/ros/humble/setup.bash` and set the user, password and IP address for the radio's ssh interface in a ./secrets.txt file on the top directory.
