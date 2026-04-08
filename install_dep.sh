#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

ROS_DISTRO=${ROS_DISTRO:-noetic}

apt_install_if_missing() {
	pkg="$1"
	if dpkg -s "$pkg" >/dev/null 2>&1; then
		echo "[install_dep] $pkg already installed, skip"
		return 0
	fi
	echo "[install_dep] installing $pkg"
	sudo apt-get install -y "$pkg"
}

# px4_rotor_sim Dependencies
apt_install_if_missing ros-${ROS_DISTRO}-mavros
apt_install_if_missing ros-${ROS_DISTRO}-mavros-extras
apt_install_if_missing ros-${ROS_DISTRO}-robot-state-publisher
if python3 -c "import PyQt5" >/dev/null 2>&1; then
	echo "[install_dep] PyQt5 already installed, skip"
else
	echo "[install_dep] installing PyQt5"
	if ! pip3 install PyQt5; then
		echo -e "\033[33m[install_dep] WARNING: PyQt5 install failed. Please install it manually later.\033[0m"
	fi
fi

# marsim_render
apt_install_if_missing libglfw3-dev
apt_install_if_missing libglew-dev
apt_install_if_missing libncurses5-dev
apt_install_if_missing libncursesw5-dev

# local_sensing_sim
# pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple open3d numpy pyyaml # for map_generator.py (optional)


