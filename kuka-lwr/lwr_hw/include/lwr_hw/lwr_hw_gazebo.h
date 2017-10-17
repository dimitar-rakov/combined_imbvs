#ifndef LWR_HW_GAZEBO_H
#define LWR_HW_GAZEBO_H

// ROS
#include <angles/angles.h>
// #include <pluginlib/class_list_macros.h>

// Gazebo hook
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

namespace lwr_hw {


class LWRHWGazebo : public LWRHW
{
public:

  LWRHWGazebo();
  ~LWRHWGazebo();

  void setParentModel(gazebo::physics::ModelPtr parent_model);

  // Init, read, and write, with Gazebo hooks
  bool init(ros::NodeHandle &nh);
  void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);

private:
  ros::NodeHandle nh_;

  // workaround: simple P controller to overcome the ofsets caused by physical properties of the gazebo model
  // ToDo for improvement to move this simple controllers in a separate thread with fixed sampling
  std::vector<double> joint_cmd_position_add_;
  std::vector<double> joint_cmd_effort_add_;    //Because of specific of gazebo 2 does not work, therefore not used. ToDo test in gazebo > 4

  // Gazebo stuff
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::ModelPtr parent_model_;
  bool parent_set_ = false;


};


} // namespace


#endif // LWR_HW_GAZEBO_H
