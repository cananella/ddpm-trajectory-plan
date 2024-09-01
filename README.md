# DDPM Trajectory plan

## preview

![uploadT-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/36050038-def9-49a7-83f7-900e180c593b)
![3-ezgif com-video-to-gif-converter (2)](https://github.com/user-attachments/assets/d4136d90-8a25-4be1-8f52-d454160163b7)



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

## doosan robotics m0609 manipulator moveit2

``` bash
ros2 launch m0609_moveit demo.launch.py
```
![image](https://github.com/user-attachments/assets/69402356-cd1d-4921-a729-82d6ce1f40e9)


## real robot controller

``` bash
ros2 launch m0609_controller m0609_controller.launch.py
```


## hello moveit pkg
``` bash
# if you want to simulate on moveit use
ros2 launch m0609_moveit demo.launch.py

# if you want use real robot
ros2 launch m0609_controller m0609_controller.launch.py
```

### get end effector pose
```bash
ros2 run hello_moveit get_eef_pos
```
![image](https://github.com/user-attachments/assets/2ac1ecf2-660a-4f56-b5ef-1eae21a7d27f)


### key controller
```bash
ros2 run hello_moveit key_controll
```
![image](https://github.com/user-attachments/assets/ddc0b7b6-3d8b-4647-acac-f6add83718e4)




## diffusion env setting

``` bash
sudo pip3 install torch==1.13.1 torchvision==0.14.1 diffusers==0.18.2 \
scikit-image==0.19.3 scikit-video==1.1.11 zarr==2.12.0 numcodecs==0.10.2 \
pygame==2.1.2 pymunk==6.2.1 gym==0.26.2 shapely==1.8.4

```


## push t
[![Video Label](http://img.youtube.com/vi/KKPK5vLKkyY/0.jpg)](https://youtu.be/KKPK5vLKkyY?si=07jjEXkW9wyNHSjk)

