# combined_imbvs

ROS/indigo packages for image moment based visual servoing with obstacles avoidance developed for KUKA LWR4 arm

## Usage

Every single package has launch files from the _launch_ directory, which are used for parametrization and start of corresponding node.

First has to be started the node for real/sim robot enviroment. The following example start simulation:

```roslaunch ics_lwr_launch ics_lwr.launch use_lwr_sim:=true lwr_powered:=false```

For the real robot start first start the ros_control script on robot side after that use:

```roslaunch ics_lwr_launch ics_lwr.launch use_lwr_sim:=false lwr_powered:=true```

Next package, whic has to be to started is the visual_features_extraction. There is option for simulation started as:

```roslaunch visual_features_extractor visual_features_extractor.launch using_sim_features:=true```

or for working with a real camera and real features:

```roslaunch visual_features_extractor visual_features_extractor.launch using_sim_features:=false```

The next step is to start kinect sensors and kinect fusion package. For this framework are used two kinect xbox sensors, which configurations are defined in coresponding launch files. If more sensors are going to be used, additional launch files have to be prepared. Both kinects, used for the setup, can be started with following sequence:

```roslaunch kinect2_bridge_k1.launch kinect2_bridge_k1.launch```
```roslaunch kinect2_bridge_k2.launch kinect2_bridge_k2.launch```

For starting the fusion package has to prepared a proper configuration in launch file. The following example start a kinect_fusion_node configurated for ICS setup:

```roslaunch kinect_fusion ics_kinect_fusion.launch```

Another example, configurated for HRI setup, can be started as:
```roslaunch kinect_fusion hri_kinect_fusion.launch```

Next package is obstacle _avoidance. The enviromental setup and robot related parameters have to be adjusted first. The following example start an obstacle_detector_node:
```roslaunch obstacle_detector obstacle_detector.launch```


Last package is imbvs_with_obstacles_avoidance:

```roslaunch imbvs_with_obstacles_avoidance imbvs_with_obstacles_avoidance.launch```
