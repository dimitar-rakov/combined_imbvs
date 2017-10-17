#ifndef LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H

#include "KinematicChainControllerBase.h"
#include "lwr_controllers/PoseRPY.h"
#include "lwr_controllers/PoseWithBaseAndTool.h"

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <utils/trajectrory_splines.h>


namespace lwr_controllers
{


class OneTaskInverseKinematics: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
{
public:
  OneTaskInverseKinematics();
  ~OneTaskInverseKinematics();

  bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void command(const lwr_controllers::PoseWithBaseAndTool::ConstPtr &msg);

private:
  ros::Subscriber sub_command_;
  ros::Subscriber sub_gains_;
  ros::Publisher  pub_all_data_;
  ros::Publisher pub_on_target_;

  std_msgs::Float64MultiArray all_data_msg;

  KDL::Frame Fbe_;        //current pose wrt to robot base
  KDL::Frame F_des_wt_;   //desired pose wrt to work base
  KDL::Frame Fwb_;        //end base wrt to work base
  KDL::Frame Fwe_;        //end efffector wrt work base
  KDL::Frame Fet_;        //tool wrt to end effector
  KDL::Frame Fwt_;        //tool wrt to work base
  KDL::Frame F_des_wt_traj_;

  std::vector<std::string> bases_names_, tools_names_;
  std::vector<KDL::Frame> frames_w_b_;
  std::vector<KDL::Frame> frames_e_t_;

  KDL::Twist x_err_;
  KDL::Jacobian Jbe_, Jbt_, Jwe_, Jwt_;	//Jacobians
  Eigen::MatrixXd J_pinv_;

  double record_data_interval_;
  int cmd_flag_;
  bool enb_record_all_data_;

  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
  boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;

  std::vector<TrajectrorySpline> cart_traj_;
  ros::Time traj_start_t;


  void recordAllData();
  void getFramesFromParamServer(XmlRpc::XmlRpcValue &obj, std::vector<std::string> &name, std::vector<KDL::Frame> &frames);
};

}

#endif
