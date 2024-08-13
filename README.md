# DDPM Trajectory plan

## moveit

[source install moveit2](https://moveit.ros.org/install-moveit2/source/)

---
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget


sudo apt update
sudo apt dist-upgrade
rosdep update

source /opt/ros/$ROS_DISTRO/setup.bash

sudo apt remove ros-$ROS_DISTRO-moveit*

export COLCON_WS=~/ws_moveit2/
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src

git clone https://github.com/moveit/moveit2.git -b $ROS_DISTRO
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd $COLCON_WS
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

---





## push t
[![Video Label](http://img.youtube.com/vi/KKPK5vLKkyY/0.jpg)](https://youtu.be/KKPK5vLKkyY?si=07jjEXkW9wyNHSjk)

