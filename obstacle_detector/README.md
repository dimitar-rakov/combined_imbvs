# Obstacle detector

ROS/indigo package for subtraction of known background and robot body from pointcloud as well as for approximation of obstacles with octree voxels.

## Overview

This package contains the implementation of subtraction of background and robot from scene, based on diffferent geometric primitives as boxes and spheres. Different enviromental setups as well as robot can be used after proper adjusting of congiration parameters.


## Usage

The node from this package can be run with the aid of the launch files from the _launch_ directory. 
The following example start a obstacle_detector_node configurated for ICS setup:
```roslaunch obstacle_detector obstacle_detector.launch```

Ofline API documentation based on doxygen can be generated whit rosdoc_lite. For generation start``` rosdoc_lite .``` whitin the package directory. The generated documentation can be found in created doc/html/index.html

## Parameters

The package consists of multiple parameters, which can be modified from the launch file. Here is given the list of them:
```
  Node base name                                              base_name                     default="obstacle_detector"
  Used points topic                                           points_topic                  default="/kinect_fusion/points"
  Size of used aruco marker in meters                         aruco_marker_size             default="0.265"
  Fixed frame in which obtacles are defined                   fixed_frame                   default="world"
  Robot links names used to get all needed TFs                tf_names                      default="[lwr_base_link, lwr_a1_link, lwr_a2_link, lwr_e1_link, lwr_a3_link, lwr_a4_link, lwr_a5_link, lwr_a6_link]"
  Resolution of octre used for the obstacles approximation    octree_resolution             default="0.14"
  Threshold for filter out all noisy voxels                   min_voxel_points              default="40"


```
In the launch file has to be defined the path to enviromental setup configuration file in (/config). In such file (see examples) can be defined geometric primitive parameters for the static background for subtraction, as well as the geometric primitive parameters used for subtracting of robot body. The property
'text' defines if points whitin the geometric primitive are kept or removed.





