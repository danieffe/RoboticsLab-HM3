# Fly your drone

## Available Packages in this Repository ##
- `force_land` 
- `offboard_rl`

# Requirements

Before running the simulation and ROS 2 nodes, install the following external components:

### **1. PX4 Autopilot (v1.16.0)**  
Required for SITL simulation and integration with Gazebo.

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.16.0
```

### **2. px4_msgs**  
```bash
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.16
```

### **3. QGroundControl**
Used to monitor flight data and arm/take off your drone.
Download [here](https://qgroundcontrol.com)

To use your custom drone, place the following files inside the `PX4-Autopilot` directory, more precisely:
`hm3_drone` folder → `PX4-Autopilot/Tools/simulation/gz/models/` directory<br>
`6003_gz_hm3_drone` airframe → `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/` directory
This enables PX4 to spawn the custom UAV in the Gazebo simulator.

## Getting Started
```shell
git clone https://github.com/danieffe/RoboticsLab-HM3.git
colcon build 
source install/setup.bash
```

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


