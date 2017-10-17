# ics_lwr_example

This package is a guide-trough-example to show how to use the [kuka_lwr](https://github.com/dimitar-rakov/packages/tree/master/kuka-lwr) packages.

## 1. Set your scenario

Create you robot and environment using the lwr model. 

For instance, in the package [__ics_lwr_setup__](./ics_lwr_setup/), the [robot](https://github.com/dimitar-rakov/packages/blob/master/kuka-lwr/ics_lwr/ics_lwr_setup/enviroment/ics_lwr_enviroment.urdf.xacro) is a lwr mounted on a mobile platform, and the [environment](https://github.com/dimitar-rakov/packages/blob/master/kuka-lwr/hri_lwr/hri_lwr_setup/worlds/simple_environment.world) is just a ground plane with a sun.

[Controllers](https://github.com/dimitar-rakov/packages/tree/master/kuka-lwr/ics_lwr/ics_lwr_setup/config/controllers.yaml#L2) are in the [config](https://github.com/dimitar-rakov/packages/tree/master/kuka-lwr/ics_lwr/ics_lwr_setup/config) folder. Note that the name space of controllers and the joint names contain the word `lwr`, which is the name you gave to the arm when creating the robot [here](https://github.com/dimitar-rakov/packages/blob/master/kuka-lwr/ics_lwr/ics_lwr_setup/enviroment/ics_lwr_enviroment.urdf.xacro#L36).

__NOTE__: Other than standard ros-controllers, there are custom controllers that can be found in the [lwr_controllers](https://github.com/dimitar-rakov/packages/tree/master/kuka-lwr/lwr_controllers) package.


## 2. Set your launch control panel

__NOTE__: recall that to run a real LWR 4+, you must follow instructions in [lwr_hw](https://github.com/dimitar-rakov/packages/tree/master/kuka-lwr/lwr_hw).

We encourage to have a separated package with your launch file to configure your different uses of your robot, including the network parameters.

We have an example in the package [__ics_lwr_launch__](./ics_lwr_launch). To check which arguments are available use:

`roslaunch --ros-args ics_lwr_launch ics_lwr.launch`

Argument autocomplete was available in Groovy, but somehow it skipped Hydro, hence Indigo, but it is comming, see [this issue](https://github.com/ros/ros_comm/issues/575).

## 3. Test in simulation

`roslaunch ics_lwr_launch ics_lwr.launch use_lwr_sim:=true lwr_powered:=false`

## 4. Test in real

Once everything went well in simulation, you can procede to use the real robot by using:

`roslaunch ics_lwr_launch ics_lwr.launch use_lwr_sim:=false lwr_powered:=true`


