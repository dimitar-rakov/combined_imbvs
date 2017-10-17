# IMBVS with obstacles avoidance

ROS/indigo package for image moment based visual servoing with obstacles avoidance

## Overview

This package contains the implementation IMBVS, extended whit an obstacles avoidance reactive algorithm.

## Usage

The node from this package can be run with the aid of the launch files from the _launch_ directory.
The following example start an imbvs_with_obstacles_avoidance_node:
```roslaunch imbvs_with_obstacles_avoidance imbvs_with_obstacles_avoidance.launch```


Ofline API documentation based on doxygen can be generated whit rosdoc_lite. For generation start ```rosdoc_lite .``` whitin the package directory. The generated documentation can be found in created doc/html/index.html


## Parameters

The package consists of multiple parameters, which can be modified from the launch file. Here is given the list of them:
```
  Node base name                                                base_name                             default="imbvs_with_obstacles_avoidance"
  Robot root name                                               root_name                             default="lwr_base_link"
  Robot tip name                                                tip_name                              default="lwr_a6_link"
  Robot description path                                        robot_description                     default="$(find ics_lwr_setup)/enviroment/ics_lwr_enviroment.urdf.xacro"
  Used topic for features data                                  features_data_topic                   default="/visual_features_extractor/visual_features_data"
  Used topic for joint states                                   joint_states_topic                    default="/lwr/joint_states"
  Used topic for objects aproximating obstacles                 obstacles_objects_topic               default="/obstacle_detector/obstacle_objects"
  Used topic for objects aproximating robot                     robot_objects_topic                   default="/obstacle_detector/robot_objects"
  Used topic for joint_trajectory_controller                    robot_command_topic                   default="/lwr/joint_trajectory_controller/command"/>
  Used topic for joint_trajectory_controller action             robot_trajectory_action_topic         default="/lwr/joint_trajectory_controller/follow_joint_trajectory"
  Enable usage of combined interaction matrix                   using_combined_matrices               default="true"
  Enable usage of transposed jacobian for repulsive vector      using_transpose_jacobian              default="false"
  Enable usage ofobstacle avoidance                             enb_obstacle_avoidance                default="true"
  Enable testing of  avoidance algorithm only                   test_only_obstacle_avoidance          default="false"
  Enable using of multiple collision points on robot body       using_multiple_colision_points        default="true"
  Maximum admissible speed                                      v_max                                 default="0.10"/>
  Smoothing factor                                              alpha                                 default="6.0"/>
  First interaction distance ro_1                               ro_1                                  default="1.0"
  Second interaction distance ro_2                              ro_2                                  default="0.2"
  Visual servoing gain                                          gama                                  default="[5.0, 5.0, 40.0, 20.0, 20.0, 1.0]"/>






