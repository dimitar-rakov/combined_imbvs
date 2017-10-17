# Kinect fusion

ROS/indigo package for pointclouds fusion from up to 6 kinect sensors

## Overview

This package contains the implementation of synchronized fusion of pointclouds. There is additional option for extrinsic calibration based on aruco marker. The package is originaly developed for kinect xbox sensors.However, another RGBD sensors could be used as well after proper parametrization.


## Dependancies
The package depends of aruco marker library. For intalation of the aruco library can be used following tutorial: [original link](http://miloq.blogspot.de/2012/12/install-aruco-ubuntu-linux.html)

## Usage

The node from this package can be run with the aid of the launch files from the _launch_ directory. 
The following example start a kinect_fusion_node configurated for ICS setup:
```roslaunch kinect_fusion ics_kinect_fusion.launch```

Another example configurated for HRI setup, can be started as:
```roslaunch kinect_fusion hri_kinect_fusion.launch```

Ofline API documentation based on doxygen can be generated whit rosdoc_lite. For generation start ```rosdoc_lite .``` whitin the package directory. The generated documentation can be found in created doc/html/index.html

## Parameters

The package consists of multiple parameters, which can be modified from the launch file. Here is given the list of them:
```
  Node base name                                          base_name                     default="kinect_fusion"
  Used image topics                                       raw_images_topics             default="[/kinect2_k1/hd/image_color, /kinect2_k2/hd/image_color]"
  Used camera info topics                                 cameras_info_topics           default="[/kinect2_k1/hd/camera_info, /kinect2_k2/hd/camera_info]
  Used points topics                                      points_topics                 default="[/kinect2_k1/sd/points, /kinect2_k2/sd/points]"
  Size of used aruco marker in meters                     aruco_marker_size             default="0.265"

```
In the launch file can be found also static trasform broadcasters related to aruco markers frame and sensors frames. Transformation between kinect sensors rgb and ir frames and between aruco and world have to be adjusted before the calibration. Transformation between ir and world frames are calculated during the calibration, therefore they have to be reparametrized after the calbration process.





