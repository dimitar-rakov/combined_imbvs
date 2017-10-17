#include "lwr_hw/lwr_hw_gazebo.h"

namespace lwr_hw {

LWRHWGazebo::LWRHWGazebo() : LWRHW() {}
LWRHWGazebo::~LWRHWGazebo() {}

void LWRHWGazebo::setParentModel(gazebo::physics::ModelPtr parent_model){parent_model_ = parent_model; parent_set_ = true;}

// Init, read, and write, with Gazebo hooks
bool LWRHWGazebo::init(ros::NodeHandle &nh)
{
  nh_ = nh;
  joint_cmd_position_add_.resize(n_joints_, 0.0);
  joint_cmd_effort_add_.resize(n_joints_, 0.0);
  if( !(parent_set_) )
  {
    std::cout << "Did you forget to set the parent model?" <<
                 std::endl << "You must do that before init()"
              << std::endl << "Exiting..." << std::endl;
    return false;
  }

  gazebo::physics::JointPtr joint;
  for(int j=0; j < n_joints_; j++)
  {
    joint = parent_model_->GetJoint(joint_names_[j]);
    if (!joint)
    {
      std::cout << "This robot has a joint named \"" << joint_names_[j]
        << "\" which is not in the gazebo model." << std::endl;
      return false;
    }
    sim_joints_.push_back(joint);

      /////////////////////d.r. New features///////////////////////

      #if GAZEBO_MAJOR_VERSION >= 4
                sim_joints_[j]->SetPosition(0, joint_cmd_position_[j]);
      #else
                sim_joints_[j]->SetAngle(0, joint_cmd_position_[j]);
      #endif

      ////////////////////////////////////////////////////////////

  }

  return true;
}

void LWRHWGazebo::read(ros::Time time, ros::Duration period)
{
  for(int j=0; j < n_joints_; ++j)
  {
    joint_msr_position_prev_[j] = joint_msr_position_[j];   
    joint_msr_velocity_prev_[j] = joint_msr_velocity_[j];
    joint_msr_position_[j] = angles::normalize_angle(sim_joints_[j]->GetAngle(0).Radian());
    joint_position_kdl_(j) = joint_msr_position_[j];
    // derivate velocity as in the real hardware instead of reading it from simulation
    joint_msr_velocity_[j] = (joint_msr_position_[j] - joint_msr_position_prev_[j])/period.toSec();
    joint_msr_velocity_[j] = filters::exponentialSmoothing(joint_msr_velocity_[j], joint_msr_velocity_prev_[j], 0.9);
    joint_msr_effort_[j] = sim_joints_[j]->GetForce((int)(0));
    joint_msr_stiffness_[j] = joint_cmd_stiffness_[j];
  }
}

void LWRHWGazebo::write(ros::Time time, ros::Duration period)
{
  enforceLimits(period);

  switch (getControlStrategy())
  {

    case JOINT_POSITION:

      for(int j=0; j < n_joints_; j++)
      {
          // according to the gazebo_ros_control plugin, this must *not* be called if SetForce is going to be called
          // but should be called when SetPostion is going to be called
          // so enable this when I find the SetMaxForce reset.
          // sim_joints_[j]->SetMaxForce(0, joint_effort_limits_[j]);


#if GAZEBO_MAJOR_VERSION >= 4
        sim_joints_[j]->SetPosition(0, joint_cmd_position_[j] + joint_cmd_position_add_[j]);
#else
        sim_joints_[j]->SetAngle(0, joint_cmd_position_[j] + joint_cmd_position_add_[j]);
#endif
        // d.r. workaround to fix ofsets caused by physical properties of the gazebo mode
        joint_cmd_position_add_[j] += -0.05 * (joint_msr_position_[j] -joint_cmd_position_[j]);
      }
      break;

    case CARTESIAN_IMPEDANCE:
      ROS_WARN("CARTESIAN IMPEDANCE NOT IMPLEMENTED");
      break;

    case JOINT_IMPEDANCE:
      // compute the gracity term
      f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

      for(int j=0; j < n_joints_; j++)
      {
        // replicate the joint impedance control strategy
        // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
        const double stiffness_effort = 0.0;//10.0*( joint_position_command_[j] - joint_position_[j] ); // joint_stiffness_command_[j]*( joint_position_command_[j] - joint_position_[j] );
        //double damping_effort = joint_damping_command_[j]*( joint_velocity_[j] );
        const double effort = stiffness_effort + joint_cmd_effort_[j] + gravity_effort_(j);
        sim_joints_[j]->SetForce(0, effort);
      }
      break;

    case JOINT_EFFORT:
      // compute the gracity term
      f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

      for(int j=0; j < n_joints_; j++)
      {
          // replicate the joint impedance control strategy without stiffness and damping
          // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
          const double effort = joint_cmd_effort_[j] + gravity_effort_(j);
          sim_joints_[j]->SetForce(0, effort);
      }
      break;
  }
}

} // namespace
