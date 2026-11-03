# Mapping using Lidar

This package is suposed to be used with Navigation ROS2 package, as well as the rf2o_laser_odometry and cartographer. We are using the Lidar from Slamtec RPLidar C1, so we use the sllidar_ros2 package

```bash
 apt-get install ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cartographer \
    ros-humble-cartographer-ros
```

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

```bash
git clone https://github.com/Slamtec/sllidar_ros2.git
``` 

```bash
cd ~/ros2_ws/
source /opt/ros/<rosdistro>/setup.bash
colcon build --symlink-install
```