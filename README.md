# DDPM Trajectory plan

![image](https://github.com/user-attachments/assets/25c3c1ea-1f74-4b5a-8114-20a6fa01099f)


## moveit

[source install moveit2](https://moveit.ros.org/install-moveit2/source/)

``` bash
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

```


## diffusion env setting

``` bash
sudo pip3 install torch==1.13.1 torchvision==0.14.1 diffusers==0.18.2 \
scikit-image==0.19.3 scikit-video==1.1.11 zarr==2.12.0 numcodecs==0.10.2 \
pygame==2.1.2 pymunk==6.2.1 gym==0.26.2 shapely==1.8.4

```


## push t
[![Video Label](http://img.youtube.com/vi/KKPK5vLKkyY/0.jpg)](https://youtu.be/KKPK5vLKkyY?si=07jjEXkW9wyNHSjk)

