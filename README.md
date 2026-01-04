
# sim_px4_drone

## simulated mavros + trimmed PX4 controller

#### px4_rotor_sim, sss_sim_env

Ultra-lightweight PX4 drone and mavros simulation (with PX4 pos,vel,att controller from v1.13.3). Currently we only support `OFFBOARD` mode.

**No need for PX4 source code and Gazebo!**

github: https://github.com/shupx/swarm_sync_sim.git (complete version)

Author: Peixuan Shu (Beihang University)

```bash
# single px4 drone + sim mavros + visualize (wallclock time)
roslaunch px4_rotor_sim px4_rotor_sim_single_no_namespace.launch
# specify the positioning type (local/gps) and initial position in the launch file.

# UI control
rosrun px4_rotor_sim keyboard_control_ros1.py
```

```bash
# single drone with /uav1 namespace
roslaunch px4_rotor_sim px4_rotor_sim_single.launch use_sim_time:=false namespace:=uav1

# UI control
rosrun px4_rotor_sim keyboard_control_ros1.py __ns:=/uav1
```

We also offer a lock-stepped multi-drone simulation so that you can accelerate the simulation as you want:

```bash
### 1. Launch sim clock UI
roslaunch sss_sim_env sim_clock.launch max_speed_ratio:=1 auto_start:=true

### 2. Launch multiple mavros-px4-rotor sim nodes (Specify initial positions in the launch file)
roslaunch px4_rotor_sim multi_px4_rotor_sim.launch
```

The sim_clock is not mandatory, and you can set `use_sim_time:=false` for wallclock time.

For visualization of a drone in real experiments:

```bash
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
roslaunch px4_rotor_sim perfect_mavros_drone_no_namespace.launch

# Single drone with namespace
roslaunch px4_rotor_sim perfect_mavros_drone.launch namespace:=uav1

# Multiple drones (3 by default)
roslaunch px4_rotor_sim perfect_mavros_drone_multi.launch

# Custom initial position (m) and yaw (degrees)
roslaunch px4_rotor_sim perfect_mavros_drone.launch init_x:=1.0 init_y:=2.0 init_z:=1.5 init_yaw:=90.0
```

## local point cloud render

#### marsim_render 

It is a OpenGL render library (ROS independent) from MARSlab of HKU. It generates local point cloud and simulates a perfect drone.

The `marsim_render` folder is part of the super planner from https://github.com/hku-mars/SUPER/tree/master/mars_uav_sim (with ROS1 and ROS2) but with improvements by Peixuan Shu.

The orignial complete old version of marsim is at https://github.com/hku-mars/MARSIM

> In marsim, the y=0 plane is a virtual floor even if there is no floor point clouds in the pcd file.

#### local_sensing_sim

ROS wrapper of marsim_render, and other sensors (TODO).

```bash
# local point cloud publisher (receiving odom/pose and pub local point clouds and global clouds from a pcd file)
roslaunch local_sensing_sim local_pointcloud_sim.launch  # modify the config yaml file first

# manually choose Nvidia GPU for opengl render
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia roslaunch local_sensing_sim local_pointcloud_sim.launch 
```

![img](misc/pc_render.png)
