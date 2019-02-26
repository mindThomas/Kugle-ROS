# Kugle-ROS
ROS workspace for the Kugle robot including packages for high-level navigation tasks including localization/SLAM with LiDAR, path planning and obstacle avoidance

# Install tool
```bash
sudo apt-get install python-catkin-tools
sudo apt-get install python-rosdep
```

# Cloning
To set up the simulation environment you need to clone the necessary repositories into an existing or new catkin workspace.
Follow the steps below to set up a new catkin workspace and clone:
```bash
mkdir -p ~/kugle_simulation_ws/src
cd ~/kugle_simulation_ws/src
catkin_init_workspace
git clone https://github.com/mindThomas/Kugle-Gazebo
git clone https://github.com/mindThomas/Kugle-ROS
git clone https://github.com/mindThomas/realsense_gazebo_plugin
rosdep install --from-paths src --ignore-src -r -y
```

# Building
Build the project with catkin build
```bash
cd ~/kugle_simulation_ws
catkin build
source devel/setup.bash
```

# Notes
Descriptions and guides how to use this ROS project can be found in the notes below.

## USB rules file for automatic device detection
The MCU can automatically be detected and assigned to `/dev/kugle` when connecting it over USB if the rules file, `99-kugle.rules`, is installed.

To install the rules file, copy `99-kugle.rules` to `/etc/udev/rules.d/`.

## Minimal bringup launch
The bringup launch includes both the driver (as described below) and the SICK LiDAR drivers. This launch script should only be used on the onboard computer with the full system connected.
```bash
roslaunch kugle_bringup minimal.launch 
```

## Launching the driver
The driver which communicates with the MCU over USB can be launched either on a laptop connected through USB to the MCU or on the onboard connected over USB. Launch the driver by running
```bash
roslaunch kugle_driver kugle_driver.launch
```

## RVIZ display
An RVIZ visualization of the robot can be launched by running
```bash
roslaunch kugle_launch rviz.launch
```

## kugle_driver CPU load
When the `kugle_driver` is running the MCU load is published on the topic `mcu_load` which can be displayed by running
```bash
rostopic echo /mcu_load -p
```
