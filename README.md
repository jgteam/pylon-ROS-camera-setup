# pylon-ROS-camera-setup
Improved installation setup for the pylon ROS driver needed by [FaSTDa Driverless](https://fastda-racing.de/wordpress/).

More information: https://github.com/basler/pylon-ros-camera

*Use the installation-script provided or follow the instructions below.*

## Software needed

- git
- python3 and pip
- curl

*The script will automatically install these packages with apt-get if not found.*

## Installation-script

Execute:

`wget -O - https://raw.githubusercontent.com/jgteam/pylon-ROS-camera-setup/main/setup.sh | bash <(cat) </dev/tty`

or download the ```setup.sh``` manually and run it.

**You will need to run this script as NON-ROOT USER!** 

## Manual installation

1. Make sure to have git, python3, pip and curl installed (`sudo apt-get install git python3 python3-pip curl`)
2. `sudo pip install rosdep`
3. *`mkdir -p catkin_ws/src`*
4. *`cd catkin_ws/src`*
5. `git clone https://github.com/basler/pylon-ros-camera`
6. `git clone https://github.com/dragandbot/dragandbot_common.git`
7. `echo 'deb http://packages.ros.org/ros/ubuntu focal main' | sudo tee /etc/apt/sources.list.d/ros-focal.list`
8. `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
9. `curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -`
10. `sudo apt-get update`
11. `sudo apt-get install ros-noetic-ros-base`
    
    All suitable options: `ros-noetic-desktop-full`, `ros-noetic-desktop`, `ros-noetic-ros-base`, `ros-noetic-ros-core`
    
12. `echo source /opt/ros/noetic/setup.bash >> ~/.bashrc`
13. `source ~/.bashrc`
14. `sudo rosdep init`
15. `sudo sh -c 'echo yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list'`
16. `rosdep update`
17. `echo "set(PYLON_ROOT '/opt/pylon')" >> pylon-ros-camera/pylon_camera/cmake/FindPylon.cmake`
18. Run `source ~/.bashrc` again and check if `echo $ROS_DISTRO` outputs `noetic`!

    *If it doesn't output `noetic` you could also change `$ROS_DISTRO` in the next command to `noetic`*

20. `sudo -E rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y`
21. *`cd ..`*
22. `catkin_make clean`
23. `catkin_make` 
24. *`echo source /YOUR/PATH/TO/catkin_ws/src/devel/setup.bash >> ~/.bashrc`*
25. `source ~/.bashrc`
