#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

# px4_rotor_sim Dependencies
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-robot-state-publisher -y
pip3 install PyQt5

# marsim_render
sudo apt-get install libglfw3-dev libglew-dev libncurses5-dev libncursesw5-dev -y

# local_sensing_sim
# pip3 install open3d numpy pyyaml # for map_generator.py (optional)


