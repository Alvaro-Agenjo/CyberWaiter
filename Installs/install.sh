#!/bin/bash
cd

echo -e "\e[36m---------SETUP SOURCES-----------\e[0m \n" 
echo -e "Ubuntu Universe repository ..." 
sudo apt install software-properties-common
sudo add-apt-repository universe

echo -e "Installing ros2-apt-source package ..." 
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt-get install python3-vcstool
sudo apt install -y docker.io

echo -e "\e[36m---------INSTALL ROS2 PACKAGES---------\e[0m \n" 
echo -e "Update & Upgrade ..." 
sudo apt update
sudo apt upgrade

echo -e "Instal Humble Desktop version"
sudo apt install ros-humble-desktop



echo -e "\e[36m---------Configuracion del terminal---------\e[0m \n" 
# echo -e "Añada las siguientes lineas en el final del documento: \n
# source /opt/ros/humble/setup.bash
# export ROS_DOMAIN_ID=1  #elige un número de 1-101. Esta opcion permite la comunicación entre dispositivos con mismo ID y conectados a la misma red
# export ROS_LOCALHOST_ONLY=1 #al estar activa solo permite la comunicación con elementos del propio dispositivo"
# gedit .bashrc

echo "source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1  #elige un número de 1-101. Esta opcion permite la comunicación entre dispositivos con mismo ID y conectados a la misma red
export ROS_LOCALHOST_ONLY=1 #al estar activa solo permite la comunicación con elementos del propio dispositivo" >> ~/.bashrc


echo -e "\e[36m---------Creacion y Configuración del workspace---------\e[0m \n" 
export COLCON_WS=~/workspace/ros_humble_ur_driver
mkdir -p $COLCON_WS/src

# echo -e "Añada la siguiente linea en el archivo ~/.bashcr: \n
# source ~/ros2_ws/install/local_setup.bash"
# gedit .bashrc
echo "source ~/workspace/ros_humble_ur_driver/install/local_setup.bash" >> ~/.bashrc

echo "Compilacion del workspace"
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


echo -e "\e[36m---------VISION PACKAGES---------\e[0m \n" 
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-cv-bridge
pip3 install opencv-python numpy



echo -e "\e[36m---------Instalacion Finalizada---------\e[0m \n" 

echo -e "\e[36m---------Extras---------\e[0m \n" 
echo -e "\e[2m\t-tf2 related tools\e[0m \n"
sudo apt-get install ros-humble-tf2-tools
sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf-transformations

echo -e "\e[2m\t-rqt tool\e[0m \n"
sudo apt-get install ros-humble-rqt-robot-steering

echo -e "\e[2m\t-Navigation\e[0m \n"
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

echo -e "\e[2m\t-Slam\e[0m \n"
sudo apt install ros-humble-slam-toolbox




