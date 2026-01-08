
# sim_px4_drone

@author: Peixuan Shu (shupeixuan@qq.com), Beihang University, China.

Install dependencies first:

```bash
./install_dep.sh
```

Then build this project with catkin_make or catkin build:

```bash
cd sim_px4_drone/
catkin_make

# source devel/setup.bash everytime you open a new terminal
source devel/setup.bash
```

## trimmed PX4 controller+sim mavros+dynamics ODE

#### px4_rotor_sim, sss_sim_env

Ultra-lightweight PX4 drone and mavros simulation (with PX4 pos,vel,att controller from v1.13.3). Currently we only support `OFFBOARD` mode, other PX4 modes do not function. 

> The dynamic model receives thrust and ideal angular velocity as input, and thus the PX4 attitude rate controller is not simulated. The core PX4 parameters can be modified in px4_rotor_sim_param.yaml

**No need for PX4 source code and Gazebo!**

github: https://github.com/shupx/swarm_sync_sim.git (original version)

Author: Peixuan Shu (Beihang University)

```bash
# single px4 drone + sim mavros + visualize (wallclock time)
roslaunch px4_rotor_sim px4_rotor_sim_single_no_prefix.launch
# specify the positioning type (local/gps) and initial position in the launch file.

# UI control (send command to mavros/setpoint_raw/local and set OFFBOARD mode, initial pz cmd is 1.0m for takeoff)
rosrun px4_rotor_sim keyboard_control_ros1.py
```

```bash
# single drone with /uav1 namespace
roslaunch px4_rotor_sim px4_rotor_sim_single.launch use_sim_time:=false namespace:=uav1

# UI control (send command to /uav1/mavros/setpoint_raw/local and set OFFBOARD mode, initial pz cmd is 1.0m for takeoff)
rosrun px4_rotor_sim keyboard_control_ros1.py __ns:=/uav1
```

We also offer a lock-stepped multi-drone simulation so that you can accelerate the simulation:

```bash
### 1. Launch sim clock UI
roslaunch sss_sim_env sim_clock.launch max_speed_ratio:=1 auto_start:=true

### 2. Launch multiple mavros-px4-rotor sim nodes (Specify initial positions in the launch file)
roslaunch px4_rotor_sim multi_px4_rotor_sim.launch use_sim_time:=true num_drones:=5
```

The sim_clock is not mandatory, and you can set `use_sim_time:=false` for wallclock time.

For visualization of a drone in real experiments:

```bash
# single drone without topic prefix
roslaunch px4_rotor_sim drone_visualizer_single_no_prefix.launch
# multi drones with seperate namespaces
roslaunch px4_rotor_sim drone_visualizer_multi.launch
```

![img](misc/px4rotor-100.gif)


## Perfect MAVROS Drone (Ideal Simulation)

A perfect ideal drone simulator that instantly follows setpoint commands with zero delay and error.

**Features:**
- Subscribes to `mavros/setpoint_raw/local`
- Directly publishes to `mavros/local_position/pose`, `mavros/local_position/velocity_local`, `mavros/local_position/odom`
- Always armed and in OFFBOARD mode
- No dynamics, no delay - perfect for testing high-level control algorithms

**Quick Start:**

```bash
# Single drone (no namespace)
roslaunch px4_rotor_sim perfect_mavros_drone_no_prefix.launch

# Single drone with namespace
roslaunch px4_rotor_sim perfect_mavros_drone.launch namespace:=uav1

# Multiple drones (3 by default)
roslaunch px4_rotor_sim perfect_mavros_drone_multi.launch num_drones:=3

# Custom initial position (m) and yaw (degrees)
roslaunch px4_rotor_sim perfect_mavros_drone.launch init_x:=1.0 init_y:=2.0 init_z:=1.5 init_yaw:=90.0
```

UI control:

```bash
# UI control (send command to mavros/setpoint_raw/local and set OFFBOARD mode, initial pz cmd is 1.0m for takeoff)
rosrun px4_rotor_sim keyboard_control_ros1.py
rosrun px4_rotor_sim keyboard_control_ros1.py __ns:=/uav1 # /uav1 topic prefix
```

## local point cloud render

#### marsim_render 

It is a OpenGL render library (ROS independent) from MARSlab of HKU. It generates local point cloud and simulates a perfect drone.

The `marsim_render` folder is part of the super planner from https://github.com/hku-mars/SUPER/tree/master/mars_uav_sim (with ROS1 and ROS2) but with improvements by Peixuan Shu.

The orignial complete old version of marsim is at https://github.com/hku-mars/MARSIM

#### local_sensing_sim

##### 1. ROS wrapper of marsim_render

```bash
# local point cloud publisher (receiving odom or pose topic and pub local point clouds and global clouds from a pcd file, set lidar params in config/local_pointcloud_sim) + px4 drone sim (/uav1 prefix)
## perfect drone + lidar sensing
roslaunch local_sensing_sim sim_px4_drone_lidar.launch 
## px4_rotor_sim + lidar sensing
# roslaunch local_sensing_sim sim_px4_drone_lidar.launch use_perfect_drone:=false 

## UI control
rosrun px4_rotor_sim keyboard_control_ros1.py __ns:=/uav1
```

```bash
## no uav1/ topic prefix
roslaunch local_sensing_sim sim_px4_drone_lidar_no_prefix.launch 
# roslaunch local_sensing_sim sim_px4_drone_lidar_no_prefix.launch use_perfect_drone:=false

## UI control
rosrun px4_rotor_sim keyboard_control_ros1.py

## manually choose Nvidia GPU for opengl render (recommended if you have a Nvidia GPU)
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia roslaunch local_sensing_sim sim_px4_drone_lidar_no_prefix.launch 
```

![img](misc/pc_render.png)

##### 2. Other sensors (TODO).