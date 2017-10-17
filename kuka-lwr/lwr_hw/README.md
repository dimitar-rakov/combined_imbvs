# lwr_hw

This package acts much like a driver. It creates a hardware interface to the real/sim robot, and it requires a URDF description in the `robot_description` parameter or configuration file defining the joint names that this hardware interface refers to.

## What to do to run a real LWR 4+

1. Load the script [`lwr_hw/krl/ros_control.src`](https://github.com/dimitar-rakov/packages/tree/master/kuka-lwr/lwr_hw/krl/ros_control.src) and the corresponding [`.dat`](https://github.com/dimitar-rakov/packages/tree/master/kuka-lwr/lwr_hw/krl/ros_control.dat) on the robot.
2. Place the robot in a position where the joints 1 and 3 are as bent as possible (at least 45 degrees) to avoid the "__FRI interpolation error__". A good way to check this is to go to gravity compensation, and see if the robot enters succesfully in that mode. 
3. Set the robot in __Position__ control.
4. Start the script in semi-automatic mode and use step forward button until you see  WAIT FOR ($FriQuality == ...
5. Start the ROS node, it will wait until the robot goes in Monitor mode to avoid loosing UDP packages. ROS controllers will not start until the handshake is done.
6. From this point on, you can manage/start/stop/run/switch controllers from ROS, and depending on which interface they use, the switch in the KRC unit is done automatically. Everytime there is a switch of interface, it might take a while to get the controllers running again. If the controllers use the same interface, the switch is done only in ROS, which is faster.
