# naoqi

This repo intends to simplify the installation and usage of naoqi system with ros2. This repository contains only the core packages of naoqi. For visualization with rviz, use [AIResearchLab/naoqi_rviz](https://github.com/AIResearchLab/naoqi_rviz). 

## Docker based usage

### Clone the repository

```bash
git clone https://github.com/AIResearchLab/naoqi.git
```

### Before starting Docker container

Before starting the docker container, ssh into your robot and run the following commands to disable automonus behaviours

```bash
ssh nao@<robot_host>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```

### Start the docker container

```bash
cd src/naoqi/docker
docker compose -f compose.amd64.yaml pull
docker compose -f compose.amd64.yaml up
```

### Once the container shutdown,

After working with the docker container, before shutting down the robot, ssh into your robot and run the following commands to enable automonus behaviours

```bash
ssh nao@<robot_host>
qicli call ALAutonomousLife.setState solitary
qicli call ALMotion.wakeUp
```

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

### Before starting ROS system

Before connecting with ROS, ssh into your robot and run the following commands to disable automonus behaviours

```bash
ssh nao@<robot_host>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```

### Starting the system

```bash
source install/setup.bash
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=10.0.0.244 qi_listen_url:=tcp://0.0.0.0:0
```

### Once the ROS system shutdown,

After working with ROS, before shutting down the robot, ssh into your robot and run the following commands to enable automonus behaviours

```bash
ssh nao@<robot_host>
qicli call ALAutonomousLife.setState solitary
qicli call ALMotion.wakeUp
```

## Examples

<details> 
<summary> <h3> Check that the node is running correctly </h3> </summary>

Check that the driver is connected:

```sh
ros2 node info /naoqi_driver
```
</details>

<details> 
<summary> <h3> Hello, world </h3> </summary>

Make the robot say hello:

```sh
ros2 topic pub --once /speech std_msgs/String "data: hello"
```
</details>

<details> 
<summary> <h3> Listen to words </h3> </summary>

You can setup speech recognition and get one result.

```sh
ros2 action send_goal listen naoqi_bridge_msgs/action/Listen "expected: [\"hello\"]
language: \"en\""
```

</details>

<details> 
<summary> <h3> Move the head </h3> </summary>

Check that you can move the head by publishing on `/joint_angles`:

```sh
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "{header: {stamp: now, frame_id: ''}, joint_names: ['HeadYaw', 'HeadPitch'], joint_angles: [0.5,0.1], speed: 0.1, relative: 0}"
```

You can see the published message with `ros2 topic echo /joint_angles`

</details>

<details> 
<summary> <h3> Move around </h3> </summary>

Check that you can move the robot by publishing on `cmd_vel` to make the robot move:

```sh
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.785"
```

> Make some room around the robot!

To stop the robot, you must send a new message with linear and angular velocities set to 0:

```sh
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

</details>