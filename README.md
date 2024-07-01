# GNC orbital

![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![humble](https://github.com/IntelligentRoboticsLabs/GNC_orbital/actions/workflows/humble.yaml/badge.svg)](https://github.com/IntelligentRoboticsLabs/GNC_orbital/actions/workflows/humble.yaml)

Simulator for guidance, navigation and control of proximity orbital operations.

#### Table of Contents

- [Instalation](#instalation)
- [Launch the simulation environment](#launch-the-simulation-environment)
- [Interfaces](#interfaces)
- [Configuration](#configuration)
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

![simulation](https://github.com/IntelligentRoboticsLabs/GNC_orbital/assets/44479765/451b97c9-ce72-4c17-8f82-575c2d83de9f)

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
/ur10_camera/camera_info
/ur10_camera/depth/camera_info
/ur10_camera/depth/image_raw
/ur10_camera/depth/image_raw/compressed
/ur10_camera/depth/image_raw/compressedDepth
/ur10_camera/depth/image_raw/theora
/ur10_camera/image_raw
/ur10_camera/image_raw/compressed
/ur10_camera/image_raw/compressedDepth
/ur10_camera/image_raw/theora
/ur10_camera/points
/ur10_joint_state_broadcaster/transition_event
/ur10_joint_trajectory_controller/controller_state
/ur10_joint_trajectory_controller/joint_trajectory
/ur10_joint_trajectory_controller/state
/ur10_joint_trajectory_controller/transition_event
/ur3_joint_state_broadcaster/transition_event
/ur3_joint_trajectory_controller/controller_state
/ur3_joint_trajectory_controller/joint_trajectory
/ur3_joint_trajectory_controller/state
/ur3_joint_trajectory_controller/transition_event
```

You have the topics for each of the robots, where you can see the status of each of the joints of each arm, or the image from the camera that the ur10 has. Additionally, you will be able to see all the transforms in /tf or /tf_static

</details>

<details>
<summary>Actions</summary>

Once the simulation is launched, you can use the `ros2 action list` command to obtain the following result:

```bash
/ur10_joint_trajectory_controller/follow_joint_trajectory
/ur3_joint_trajectory_controller/follow_joint_trajectory
```

These are the actions that moveit2 will use to be able to move the arms

</details>

## Configuration
### Spotlights
In the file `params/world_params.yaml` you will find the world configurations, including the spotlights.

<details>
<summary>world_params.yaml</summary>
  
```yaml
# Spotlight 1
## Pose
spot1_x_light: 0.0
spot1_y_light: 0.0
spot1_z_light: 1.2
spot1_roll_light: 0.0
spot1_pitch_light: 1.56
spot1_yaw_light: 0.78

## Diffuse
spot1_R_light: 1
spot1_G_light: 1
spot1_B_light: 1
spot1_opacity_light: 1.0

## Specular
spot1_specular_R: 1
spot1_specular_G: 1
spot1_specular_B: 1
spot1_specular_opacity: 1.0

## Attenuation
spot1_attenuation_range: 30
spot1_attenuation_linear: 1
spot1_quadratic: 0.001

## Spot
spot1_inner_angle: 0.1
spot1_outer_angle: 0.2
spot1_fall_off: 0.5


# Spotlight 2
## Pose
spot2_x_light: -2.0
spot2_y_light: 0.0
spot2_z_light: 1.2
spot2_roll_light: 0.0
spot2_pitch_light: 1.56
spot2_yaw_light: 2.53

## Diffuse
spot2_R_light: 1
spot2_G_light: 1
spot2_B_light: 1
spot2_opacity_light: 1.0

## Specular
spot2_specular_R: 1
spot2_specular_G: 1
spot2_specular_B: 1
spot2_specular_opacity: 1.0

## Attenuation
spot2_attenuation_range: 30
spot2_attenuation_linear: 1
spot2_quadratic: 0.001

## Spot
spot2_inner_angle: 0.1
spot2_outer_angle: 0.2
spot2_fall_off: 0.5
```
</details>

Parameters to take into account:
- To change the color of the spotlights, you have to modify the diffuse and specular RGB values, which range from 0 to 1.
- To modify the light range, you have to modify the spotlight's attenuation_range value.
- To modify the angle by which the light extends, you must touch the outer_angle value. This value cannot be less than the inner, so if a smaller angle is necessary, both will have to be modified.
- You can also modify the position of the spotlights and the angle at which the light goes, by touching the xyz rpy parameters.
- The parameters attenuation_linear, quadratic and fall_of are used to configure how the light decreases linearly with distance.
  
### Arms
The configuration of the arms is located in params/arms_params.yaml, and this allows you to modify its position and initial orientation, as well as the name given to it, type of arm and the prefix that will be given to the tf of the arm to differentiate them from another.

<details>
<summary>arms_params.yaml</summary>
  
```yaml
gnc_orbital:
  arm1:
    name: ur3
    type: ur3
    tf_prefix: ur3_
    position: [-2.4, -1.6, 0.73]
    orientation: [0, 0, 0]
  arm2:
    name: ur10
    type: ur10
    tf_prefix: ur10_
    position: [0, -2.89, 1]
    orientation: [-1.56, 0, 0]
```
</details>

It must be taken into account that if the table arm is going to be moved, the position of this table must also be modified. You can also do this from the params/world_params.yaml file

<details>
<summary>world_params.yaml</summary>
  
```yaml
# UR 5 Table
## Pose
x_table: -2.6
y_table: -1.1
z_table: 0.35
roll_table: 0.0
pitch_table: 0.0
yaw_table: 1.56

## Size
length_table: 1.2
width_table: 0.6
height_table: 0.73
```
</details>

## Usage
In order to move the ur3 robot, you will have to use the following command from terminal:

```bash
ros2 topic pub /ur3_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header: 
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['ur3_shoulder_pan_joint', 'ur3_shoulder_lift_joint', 'ur3_elbow_joint', 'ur3_wrist_1_joint', 'ur3_wrist_2_joint', 'ur3_wrist_3_joint']
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
points: [{positions: [0.0, -1.57, -1.57, 0.7, 1.5, -1.2, -1.1],
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

[Demo 02: ur3 and ur10 arm movement with camera. Totally dark world with spotlights](https://github.com/IntelligentRoboticsLabs/GNC_orbital/assets/44479765/e13574ec-540a-47cf-b334-20a12cc766e9)


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


## Acknowledgment
<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="50" align="left" >  

Grant TED2021-132099B-C32 funded by MCIN/AEI/10.13039/501100011033   
and by "European Union NextGenerationEU/PRTR".
