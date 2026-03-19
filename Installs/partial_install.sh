#!/bin/bash
cd

sudo apt-get install python3-vcstool
sudo apt install -y docker.io

export COLCON_WS=~/workspace/ros_humble_ur_driver
mkdir -p $COLCON_WS/src

echo "source ~/workspace/ros_humble_ur_driver/install/local_setup.bash" >> ~/.bashrc

echo -e "\e[36m---------UR DRIVER GIT DOWNLOAD---------\e[0m \n" 

sudo apt install python3-colcon-common-extensions
cd $COLCON_WS

git clone  -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos

sudo rosdep init
rosdep update
rosdep install --ignore-src --from-paths src -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Releases

echo -e "\e[36m---------Docker configuration---------\e[0m \n" 

sudo groupadd Docker
sudo usermod -aG docker ${USER}
sudo chmod 666 /var/run/docker.sock
#docker network create --subnet=192.168.56.0/24 ursim_net
