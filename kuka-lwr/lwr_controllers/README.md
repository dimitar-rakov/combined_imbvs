# KUKA LWR CONTROLLERS

ROS/indigo package with various controllers implemented for the LWR 4+

## Overview

This package contains the implementation of different control strategies for the KUKA LWR 4+. 


## Usage

The desired controller can be ran with the aid of the launch files from the _lwr_launch_ package. 
The following example loads the OneTaskInverseKinematics controller:  
```roslaunch lwr_launch lwr_launch.launch controller:=OneTaskInverseKinematics```  
Each of the controllers listed here can be loaded this way, either for use on a simulated robot or on a real one, depending on the value of the _use_lwr_sim_ parameter.  

Also more than one controller can be loaded at once:  
```roslaunch lwr_launch lwr_launch.launch controller:="joint_position_controller joint_trajectory_controller"```  
(please note the quotes surrounding the list of controllers to be loaded).  

Sometimes it is desired to load multiple controllers, but start only one of them, with the aim of switch between them at runtime with controller_manager.  
This can be achieved with the _stopped_controllers_ parameter:  
```roslaunch lwr_launch lwr_launch.launch controller:="OneTaskInverseKinematics"  stopped_controllers:="AdaptiveTorqueController"```

Already started controllers as for example one_task_inverse_kinematics can be stoped and swicth to another controller can be done by following service call:

```
rosservice call /lwr/controller_manager/switch_controller
"start_controllers:
- ’joint_trajectory_controller’
stop_controllers:
- ’one_task_inverse_kinematics’
strictness: 2"
```

Controllers tested on ICS setup:
- JointTrajectoryController

`rostopic pub /lwr/joint_trajectory_controller/command trajectory_msgs/JointTrajectory ’{joint_names: [’lwr_a1_joint’, ’lwr_a2_joint’, ’lwr_e1_joint’,’lwr_a3_joint’, ’lwr_a4_joint’, ’lwr_a5_joint’,’lwr_a6_joint’], points:[{positions: [0.0, 1.570796, 0.0, 0, 0.0, 0.0, -0.0],velocities: [], accelerations: [], effort: [],time_from_start: { secs: 10, nsecs: 0}}]}’ -1`

- OneTaskInverseKinematics

  - With default robot base and end effector

  `rostopic pub -1 /lwr/one_task_inverse_kinematics/command
  lwr_controllers/PoseWithBaseAndTool "base_name: ’robot_base’
  tool_name: ’flange’
  position:
    x: 0.0
    y: 0.6
    z: 0.5
  orientation:
    roll: 1.2
    pitch: 0.3
    yaw: 0.3"
  `
  - With world base and end effector

  `rostopic pub -1 /lwr/one_task_inverse_kinematics/command
  lwr_controllers/PoseWithBaseAndTool "base_name: ’world’
  tool_name: ’flange’
  position:
    x: 0.0
    y: 0.6
    z: 0.8
  orientation:
    roll: 0.8
    pitch: 0.3
    yaw: 0.5"
  `

- AdaptiveTorqueController
`rostopic pub -1 /lwr/adaptive_torque_controller/command std_msgs/Float64MultiArray ’{data: [0.0, 1.57, 0.0, 0.5, 0.0, 0.0, 0.0]}’`


Controler from original package are not tested on ICS setup.

- Impedance Controller:
`rostopic pub -1  /lwr/JointImpedanceControl/command std_msgs/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`

- Inverse Dynamics Controller
`rostopic pub -1  /lwr/InverseDynamicsControl/ext_wrench geometry_msgs/WrenchStamped '{wrench: {force:  {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0,y: 0.0,z: 0.0}}}' `

- Computed Torque Controller:
`rostopic pub -1  /lwr/ComputedTorqueControl/command std_msgs/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`

  - PID Gains setting
`rosrun rqt_reconfigure rqt_reconfigure`

- Multi Task Priority Inverse Kinematics Controller:
  
  - Tasks description (Notes: Link index -1 is EndEffector and Task parameters are [x,y,x,roll,pitch,yaw])
`rostopic pub -1  /lwr/MultiTaskPriorityInverseKinematics/command lwr_controllers/MultiPriorityTask '{links: [-1,3], tasks: [-0.4,0.3,1.5,0,0,0,-0.02,0.2,1.311,-1.568,0.291,0.1]}'`

  - PID Gains setting
`rosrun rqt_reconfigure rqt_reconfigure`

- Multi Task Priority Inverse Dynamics Controller:
  
  - Tasks description (Notes: Link index -1 is EndEffector and Task parameters are [x,y,x,roll,pitch,yaw])
`rostopic pub -1  /lwr/MultiTaskPriorityInverseDynamics/command lwr_controllers/MultiPriorityTask '{links: [-1,3], tasks: [-0.4,0.3,1.5,0,0,0,-0.02,0.2,1.311,-1.568,0.291,0.1]}'`

- Dynamic Sliding PID Control (Task Space)
`rostopic pub -1 /lwr/DynamicSlidingModeControllerTaskSpace/command std_msgs/Float64MultiArray '{data:[-0.4,0.3,1.5,0,0,0]}'`

- Minimum Effort Inverse Dynamics Controller:
just run the controller (no command support)

- Backstepping Controller:
just run the controller (no command support)






