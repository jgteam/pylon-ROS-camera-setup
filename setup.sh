#!/bin/bash

# Setup-Script written by Jannis Guensche
# https://github.com/jgteam/pylon-ROS-camera-setup

# More info: https://github.com/basler/pylon-ros-camera

BLUE='\033[1;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
RESET='\033[0m'
OUTPUT_SIG="${BLUE}[P-R-C Setup]"
OUTPUT_SIG_E="${RED}[P-R-C Setup ERROR]"
OUTPUT_SIG_W="${YELLOW}[P-R-C Setup NOTICE]"


# Feedback function
p () {
	echo -e "$OUTPUT_SIG $1 $RESET"
}
e () {
	echo -e "$OUTPUT_SIG_E $1 $RESET"
}
w () {
	echo -e "$OUTPUT_SIG_W $1 $RESET"
}

# Getting the right permissions
if [ $EUID -eq 0 ]; then
	e "Don't run as root. Some commands need to be run as non-root user! quitting..."
	exit 1
fi
sudo echo "Getting sudo..."
if [ $? -eq 0 ]; then
	p "Access to sudo granted."
else
	e "Coudn't get sudo access... quitting."
	exit 1
fi

# Some helpful functions
run () {
	eval "$1"

	if [ $? -ne 0 ]; then
		e "Last command did not succeed... quitting."
		e "FAILED CMD: \n\n	$1\n"
		exit 1
	else
		p "Finished command: \n	$1"
	fi
}

try () {
	eval "$1"

	if [ $? -ne 0 ]; then
		w "Last command did not succeed... "
		w "FAILED CMD: \n\n	$1\n"
	else
		p "Finished command: \n	$1"
	fi
}

check_installation () {

	p "Checking if $1 is installed..."

	eval "$2 2>&1 >/dev/null"

	if [ $? -ne 0 ]; then 
		w "$1 not found! Installing..."
		run "$3"
	else 
		p "$1 found."
	fi

}

# Beginning of setup

# Get Workspace dir:

p "Enter workspace directory (e.g. 'catkin_ws'). It will be created if it does not exist."
echo -e -n "$OUTPUT_SIG Workspace> $RESET"
read WSDIR
if [ -z "$WSDIR" ]
then
	e "Workspace directory cannot be empty!"
	exit 1
fi
w "\$WSDIR set to $WSDIR"

p "Enter ROS_DISTRO (e.g. 'noetic'). It will be set to noetic if left empty."
echo -e -n "$OUTPUT_SIG Workspace> $RESET"
read ROS_DISTRO
if [ -z "$ROS_DISTRO" ]
then
	ROS_DISTRO=noetic
fi
w "\$ROS_DISTRO set to $ROS_DISTRO"


check_installation "git" "git --version" "sudo apt-get -y install git"
check_installation "pip" "pip --version" "sudo apt-get -y install python3-pip"
check_installation "rosdep" "rosdep --version" "sudo pip install rosdep"

run "sudo mkdir -p ${WSDIR}/src"
run "cd ${WSDIR}/src"
run "git clone https://github.com/basler/pylon-ros-camera"
run "git clone https://github.com/dragandbot/dragandbot_common.git"
run "echo 'deb http://packages.ros.org/ros/ubuntu focal main' | sudo tee /etc/apt/sources.list.d/ros-focal.list"
try "sudo rosdep init"
run "rosdep update"
run "sudo sh -c 'echo 'yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml' > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list' "
run "rosdep update"
run "echo 'set(PYLON_ROOT \"/opt/pylon\")' >> pylon-ros-camera/pylon_camera/cmake/FindPylon.cmake"
run "echo ${ROS_DISTRO}"
run "sudo -E rosdep install --from-paths . --ignore-src --rosdistro=${ROS_DISTRO} -y"


