# Visual Features Extraction

ROS/indigo package for visual features extraction

## Overview

This package contains the implementation of 2 different features extraction method. First one is based on coloured blobs. The second one is based on aruco markers. The extracted features are used for building features vectors and interaction matrices used for IMBVS algorithm.


## Dependencies
The package depends on aruco marker library. For intalation of the aruco library can be used following tutorial: [original link](http://miloq.blogspot.de/2012/12/install-aruco-ubuntu-linux.html)

## Usage

The node from this package can be run with the aid of the launch files from the _launch_ directory. Coresponding parameters in the launch file can be adjusted before the start (see Parameters list bellow)
The following example start a visual_extraction_node:
```roslaunch visual_features_extractor visual_features_extractor.launch```

Ofline API documentation based on doxygen can be generated whit rosdoc_lite. For generation start ```rosdoc_lite .``` whitin the package directory. The generated documentation can be found in created doc/html/index.html

## Parameters

The package consists of multiple parameters, which can be modified from the launch file. Here is given the list of them:
```
  Node base name                                    base_name                     default="visual_fetures_extractor"
  Used topic for raw image                          raw_images_topic              default="/usb_cam/image_raw"
  Camera name                                       camera_name                   default="eye_in_hand"
  Enable simulated features                         using_sim_features            default="false"
  Enable image moment based features                using_extended_features       default="true"
  Working with symmetrical features                 using_symmetrical_features    default="true"
  Variant for used extended features                extended_features_var         default="1.2"
  Working with coloured blobs                       using_colored_blobs           default="false"
  Minimum contour threshold for coloured features   contour_area_threshold        default="25.0"
  HSV color ranges for coloured blobs               blobs_color_ranges            default="[ {h_min: 170, h_max: 10}, {h_min: 40, h_max: 80}, {h_min: 100, h_max: 125}, {h_min: 130, h_max: 165} ]"
  Parameters of aruco markers                       arucos_param                  default="[ {id: 110, size: 0.04}, {id: 120, size: 0.04}, {id: 130, size: 0.04}, {id: 140, size: 0.04} ]"
  Path to camera calibration file                   camera_param_file             default="$(find usb_cam)/eye_in_hand_640x480.yaml"

```



