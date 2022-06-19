#!/bin/bash

# Setup-Script written by Jannis Guensche
# https://github.com/jgteam/pylon-ROS-camera-setup

# More info: https://github.com/basler/pylon-ros-camera

# Script is written for the noetic distro

ROS_INSTALLATION_PACKAGE=ros-noetic-ros-base
# All suitable options: ros-noetic-desktop-full, ros-noetic-desktop, ros-noetic-ros-base, ros-noetic-ros-core

BLUE='\033[1;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[1;32m'
RESET='\033[0m'
OUTPUT_SIG="${BLUE}[P-R-C Setup]"
OUTPUT_SIG_E="${RED}[P-R-C Setup ERROR]"
OUTPUT_SIG_W="${YELLOW}[P-R-C Setup NOTICE]"
OUTPUT_SIG_F="${GREEN}[P-R-C Setup]"

# Feedback function
p () { # Print
	echo -e "$OUTPUT_SIG $1 $RESET"
}
e () { # Error
	echo -e "$OUTPUT_SIG_E $1 $RESET"
}
w () { # Warning / Notice
	echo -e "$OUTPUT_SIG_W $1 $RESET"
}
f () { # Finished
	echo -e "$OUTPUT_SIG_F $1 $RESET"
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
	p "Executing command: \n	$1"
	
	eval "$1"

	if [ $? -ne 0 ]; then
		e "Last command did not succeed... quitting."
		e "FAILED CMD: \n\n	$1\n"
		exit 1
	fi
}

try () {
	p "Executing command: \n	$1"

	eval "$1"

	if [ $? -ne 0 ]; then
		w "Last command did not succeed... "
		w "FAILED CMD: \n\n	$1\n"
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

add_source () {

	run "source $1"
	run "sudo sh -c 'echo source $1 >> ~/.bashrc'"
	run "source ~/.bashrc"

}

# Beginning of setup

w "\$ROS_INSTALLATION_PACKAGE set to $ROS_INSTALLATION_PACKAGE"
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


check_installation "git" "git --version" "sudo apt-get -y install git"
check_installation "python3" "python3 --version" "sudo apt-get -y install python3"
check_installation "pip" "pip --version" "sudo apt-get -y install python3-pip"
check_installation "rosdep" "rosdep --version" "sudo pip install rosdep"
check_installation "curl" "curl --version" "sudo apt-get -y install curl"

run "mkdir -p ${WSDIR}/src"
run "cd ${WSDIR}/src"
run "git clone https://github.com/basler/pylon-ros-camera"
run "git clone https://github.com/dragandbot/dragandbot_common.git"
run "echo 'deb http://packages.ros.org/ros/ubuntu focal main' | sudo tee /etc/apt/sources.list.d/ros-focal.list"
run "sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
run "curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -"
run "sudo apt-get -y update"

run "sudo apt-get -y install $ROS_INSTALLATION_PACKAGE"

add_source "/opt/ros/noetic/setup.bash"


try "sudo rosdep init"
#run "rosdep update"
run "sudo sh -c 'echo yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list' "
run "rosdep update"
TMP_PYLON_ROOT="'set(PYLON_ROOT '/opt/pylon')'"
run "echo ${TMP_PYLON_ROOT} >> pylon-ros-camera/pylon_camera/cmake/FindPylon.cmake"
run "source ~/.bashrc"
if [ -z "$ROS_DISTRO" ]
then
	e "\$ROS_DISTRO NOT SET!"
	w "Using 'noetic' as distro!"
	ROS_DISTRO=noetic
fi
w "\$ROS_DISTRO set to $ROS_DISTRO"
run "sudo -E rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y"
run "cd .."
run "catkin_make clean"
run "catkin_make"

SCRIPT=`realpath $0`
echo ${SCRIPT}
SCRIPTPATH=`dirname $SCRIPT`
add_source "${SCRIPTPATH}/devel/setup.bash"

f "Installation finished! You can run now\n\n	roslaunch pylon_camera pylon_camera_node.launch\n	roslaunch pylon_camera pylon_camera_ip_configuration.launch"

