# naoqi

This repo intends to simplify the installation and usage of naoqi system with ros2. This repository contains only the core packages of naoqi. For visualization with rviz, use [AIResearchLab/naoqi_rviz](https://github.com/AIResearchLab/naoqi_rviz). 

## Docker based usage



## ROS based installation

Create the workspace
```bash
mkdir -p workspace/src
cd workspace/src
```

### Core Packages (For non gui based usage)

Clone the repository with submodules

```bash
git clone --recursive https://github.com/AIResearchLab/naoqi.git
```

### Core Packages with RVIZ (For gui based usage)

Clone the repository with submodules

```bash
git clone --recursive https://github.com/AIResearchLab/naoqi.git
git clone --recursive https://github.com/AIResearchLab/naoqi_rviz.git
```

### RVIZ Packages only (To connect and visualize data from a connected robot) 

For visalization only tasks, follow instructions at [AIResearchLab/naoqi_rviz](https://github.com/AIResearchLab/naoqi_rviz)

### Build workspace 

According to [ros-naoqi/naoqi_driver2](https://github.com/ros-naoqi/naoqi_driver2.git) developers comments, [AIResearchLab/naoqi](https://github.com/AIResearchLab/naoqi) works with both AMD64 and ARM64 computers while [AIResearchLab/naoqi_rviz](https://github.com/AIResearchLab/naoqi_rviz.git) can only be build on AMD64 computers.

```bash
cd workspace
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build
```

to agree to and skip [license](https://github.com/ros-naoqi/naoqi_driver2#license-of-the-meshes) prompt, run,

```bash
I_AGREE_TO_NAO_MESHES_LICENSE=1 I_AGREE_TO_PEPPER_MESHES_LICENSE=1 colcon build
```

## Pre-launch configuration on robot

ssh into your robot and run the following commands to disable automonus behaviours

```bash
ssh nao@<robot_host>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```

## Starting the system

```bash
source install/setup.bash
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=10.0.0.244 qi_listen_url:=tcp://0.0.0.0:0
```

## Post-run configuration on robot

ssh into your robot and run the following commands to enable automonus behaviours

```bash
ssh nao@<robot_host>
qicli call ALAutonomousLife.setState solitary
qicli call ALMotion.wakeUp
```