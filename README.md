# GNC orbital

![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![humble](https://github.com/IntelligentRoboticsLabs/GNC_orbital/actions/workflows/humble.yaml/badge.svg)](https://github.com/IntelligentRoboticsLabs/GNC_orbital/actions/workflows/humble.yaml)

Simulator for guidance, navigation and control of proximity orbital operations.

#### Table of Contents

- [Instalation](#instalation)
- [Launch the simulation environment](#launch-the-simulation-environment)
- [Interfaces](#interfaces)
- [Usage](#usage)
- [Demos](#demos)
- [About](#about)

## Instalation

You need to have previously installed ROS2. Please follow this [guide](https://docs.ros.org/en/humble/Installation.html) if you don't have it.

```bash
source /opt/ros/humble/setup.bash
```

Create workspace and clone the repository

```bash
mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/IntelligentRoboticsLabs/GNC_orbital.git
```

Install dependencies and build workspace
```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install 
```

Setup the workspace
```bash
source ~/ros2_ws/install/setup.bash
```

## Launch the simulation environment

To launch the simulation environment once everything is compiled, we must use the following command:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch gnc_orbital ur_sim_control.launch.py
```

Once launched, the rviz viewer with both arms, camera and rail, and the gazebo simulation environment should open.

![Screenshot from 2024-05-27 11-48-34](https://github.com/IntelligentRoboticsLabs/GNC_orbital/assets/44479765/83673c64-a232-448d-9c75-023ef940c916)

If what you want is for the rviz or gazebo viewer not to open, use one of these two commands or a combination of both:

```bash
ros2 launch gnc_orbital ur_sim_control.launch.py launch_rviz:=false
ros2 launch gnc_orbital ur_sim_control.launch.py gazebo_gui:=false
```

## Interfaces

<details>
<summary>Topics</summary>

Once the simulation is launched, you can use the `ros2 topic list` command to obtain the following result:

```bash
/clicked_point
/clock
/dynamic_joint_states
/goal_pose
/initialpose
/joint_state_broadcaster/transition_event
/joint_states
/parameter_events
/performance_metrics
/robot_description
/rosout
/tf
/tf_static
/ur10_joint_state_broadcaster/transition_event
/ur10_joint_trajectory_controller/controller_state
/ur10_joint_trajectory_controller/joint_trajectory
/ur10_joint_trajectory_controller/state
/ur10_joint_trajectory_controller/transition_event
/ur5_camera/camera_info
/ur5_camera/depth/camera_info
/ur5_camera/depth/image_raw
/ur5_camera/depth/image_raw/compressed
/ur5_camera/depth/image_raw/compressedDepth
/ur5_camera/depth/image_raw/theora
/ur5_camera/image_raw
/ur5_camera/image_raw/compressed
/ur5_camera/image_raw/compressedDepth
/ur5_camera/image_raw/theora
/ur5_camera/imu
/ur5_camera/points
/ur5_joint_state_broadcaster/transition_event
/ur5_joint_trajectory_controller/controller_state
/ur5_joint_trajectory_controller/joint_trajectory
/ur5_joint_trajectory_controller/state
/ur5_joint_trajectory_controller/transition_event
```

You have the topics for each of the robots, where you can see the status of each of the joints of each arm, or the image from the camera that the ur5 has. Additionally, you will be able to see all the transforms in /tf or /tf_static

</details>

<details>
<summary>Actions</summary>

Once the simulation is launched, you can use the `ros2 action list` command to obtain the following result:

```bash
/ur10_joint_trajectory_controller/follow_joint_trajectory
/ur5_joint_trajectory_controller/follow_joint_trajectory
```

These are the actions that moveit2 will use to be able to move the arms

</details>

## Usage
In order to move the ur5 robot, you will have to use the following command from terminal:

```bash
ros2 topic pub /ur5_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header: 
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint', 'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']
points: [{positions: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2, nanosec: 0}}]"
```

In order to move the ur10 robot, you will have to use the following command from terminal:

```bash
ros2 topic pub /ur10_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header: 
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint', 'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint', 'ur10_rail_joint']
points: [{positions: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0, 0.6],
      time_from_start: {sec: 2, nanosec: 0}}]" 
```

If what you want is to see the image or the pointcloud generated by the camera, or the transform system, you should go to Rviz and follow the following steps:
- Transforms: `Add > By display type > TF`
- RGB image: `Add > By topic > /ur5_camera > /image_raw > Image`
- Depth image: `Add > By topic > /ur5_camera > /depth > /image_raw > Image`
- PointCloud: `Add > By topic > /ur5_camera > /points > PointCloud2`

## Demos
In this demo the commands given above are being executed, only modifying the positions of the joints:

[Demo 01: ur5 arm movement with camera and ur10 arm publishing in topics](https://github.com/IntelligentRoboticsLabs/GNC_orbital/assets/44479765/b95b3e50-8d5f-4296-9769-043cdaa1f76b)

## About
This is a project made by the [Intelligent Robotics Lab], a research group from the [Universidad Rey Juan Carlos].
Copyright &copy; 2024.

Maintainers:

* [Juan Carlos Manzanares Serrano]
* [Juan S. Cely]

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[Intelligent Robotics Lab]: https://intelligentroboticslab.gsyc.urjc.es/
[Juan Carlos Manzanares Serrano]: https://github.com/Juancams
[Juan S. Cely]: https://github.com/juanscelyg
