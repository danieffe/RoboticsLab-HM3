# Fly your drone

## Available Packages in this Repository ##
- `ros2_iiwa` 
- `ros2_kdl_package`
- `aruco_ros`

## Getting Started
```shell
git clone https://github.com/danieffe/RoboticsLab-HM3.git
colcon build 
source install/setup.bash

```

```shell
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.16.0
git clone https://github.com/PX4/px4_msgs.git
git checkout release/1.16
```

Don't forget to move the folder hm3_drone in the      directory and the airframe file 6003_gz_hm3_drone in the       directory.


# **Usage**

 ## **1. Fly custom hm3_drone**
Open QGROUNDCONTROL then in a new terminal:

```shell
make px4_sitl gz_hm3_drone
```
A new gazebo simulation will open and from now you can fly the custom drone by using the sticks.


 ## **2. Force landing**
In a second terminal run the following command::

```shell
./DDS_run.sh
```
And in a third terminal launch:

```shell
ros2 run force_land force_land
```

From now, you can takeoff your drone. The landing procedure will activate only once, after that the threshold of 20m is surpassed.
To test the correct functioning, it is advisable to do the takeoff directly above 20 m, and then resume control by using the sticks.


## **3. Flying in offboard mode**

```shell
cd /ros2_ws
ros2 run offboard_rl spline_flight_node
```


