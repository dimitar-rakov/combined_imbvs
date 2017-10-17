#ifndef IIMBVS_WITH_OBSTACLES_AVOIDANCE
#define IIMBVS_WITH_OBSTACLES_AVOIDANCE


#include "imbvs_with_obstacles_avoidance/VisualServoing.h"
#include "imbvs_with_obstacles_avoidance/PoseRPY.h"
#include "imbvs_with_obstacles_avoidance/SetGama.h"
#include "imbvs_with_obstacles_avoidance/VisFeature.h"

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>

#include <Eigen/Dense>



/// SAFETY maximum difference between two desire joint position
const static double MAX_JOINT_DES_STEP = 0.0025;
/// using of trajectory client instead of oj just publisher
const static bool USE_TRAJ_CLIENT = true;
/**
 * @brief The ImbvsWithObstaclesAvoidance class
 */
class ImbvsWithObstaclesAvoidance
{

public:

  /**
   * @brief ImbvsWithObstaclesAvoidance Default constructor
   */
  ImbvsWithObstaclesAvoidance();
  /**
   * @brief ~ImbvsWithObstaclesAvoidance Default destructor
   */
  ~ImbvsWithObstaclesAvoidance();

  /**
     * @brief init Initilializing of KinectFusion. It has to be called only once before update().
     * @param nh Input node handle
     * @return true if everithing pass normaly, false otherwise
     */
  bool init(ros::NodeHandle &n);

  /**
   * @brief starting Have to be caled once just before first update()
   * @param time Input time from start
   */
  void starting(const ros::Time& time);

  /**
     * @brief update Periodicaly called. All calculations related to the class are done whitin.
     * @param time Input time from start
     * @param period Input last update period
     */
  void update(const ros::Time& time, const ros::Duration& period);
  /**
   * @brief command Calback for external setpoint for desired features (not used)
   * @param msg Incomming message
   */
  void command(const  imbvs_with_obstacles_avoidance::VisualServoing &msg);

  /**
     * @brief publish All publishers are whitin. Data for publishing has to be prepared previously in update ()
     */
  void publish();


private:
  /// Node handle
  ros::NodeHandle nh_;

  /// KDL Chainf of the robot
  KDL::Chain kdl_chain_;

  /// Joint measured states
  KDL::JntArrayAcc joint_msr_states_;

  /// Joint desired states
  KDL::JntArrayAcc joint_des_states_;

  /// Joint previus desired states used for trapezoidal integration
  KDL::JntArrayAcc joint_des_states_prev_;


  /// Stored joint states at initial obstacles interaction
  KDL::JntArrayAcc joint_init_interaction_states_;

  /// Robot limits
  struct limits_
  {
    KDL::JntArray min;
    KDL::JntArray max;
    KDL::JntArray center;
    KDL::JntArray velocity;
    KDL::JntArray acceleration;
    KDL::JntArray jerk;
    KDL::JntArray effort;
  } joint_limits_;

  /// Subcriber to command
  ros::Subscriber sub_command_;

  /// Subcriber to visual data
  ros::Subscriber sub_visual_features_data_;

  /// Subcriber to obstacles objects
  ros::Subscriber sub_markers_obstacles_objects_;

  /// Subcriber to robot body objects
  ros::Subscriber sub_markers_robot_objects_;

  /// Subcriber to robot joint states
  ros::Subscriber sub_joint_state_;

  /// Publisher for all data
  ros::Publisher  pub_all_data_;

  /// Publisher for calculated joint trajectory
  ros::Publisher  pub_joint_traj_;

  /// Publisher for visualization of repulsive vector
  ros::Publisher  pub_rep_vec_marker_;

  /// Pointer for trajectory client
  boost::shared_ptr <actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > > traj_client_ptr_;

  /// Transform listener
  tf::TransformListener lr_;


  /// Message for all data publisher
  std_msgs::Float64MultiArray all_data_msg;

  /// Trajectory points for all joints
  trajectory_msgs::JointTrajectoryPoint traj_points_;

  /// Trajectory message
  trajectory_msgs::JointTrajectory traj_msg_;

  /// Container for robot links names
  std::vector<std::string> tf_names_;

  /// Markers related to robot body
  visualization_msgs::MarkerArray markers_robot_objects_;

  /// Markers related to obstacles
  visualization_msgs::MarkerArray markers_obstacles_objects_;

  /// Current pose end efffector wrt robot base
  KDL::Frame Fbe_;

  /// End efffector wrt robot world
  KDL::Frame Fwe_;

  /// Camera wrt to end effector
  KDL::Frame Fec_;

  /// Base wrt to world
  KDL::Frame Fwb_;

  /// Poses of obstacles wrt base
  std::vector< KDL::Frame> Fbo_;

  /// Poses of robot objects wrt base
  std::vector< KDL::Frame> Fbs_;

  /// Points on obstacles object surface, where distance to robot object surface is min
  std::vector<KDL::Vector> Pb_os_min_;

  /// Points on robot object surface, where distance to obstacles surface is min
  std::vector<KDL::Vector> Pb_rs_min_;

  /// Jacobians to end effector wrt to robot base
  KDL::Jacobian Jbe_;

  /// Jacobians to end effector wrt to end effector
  KDL::Jacobian Jee_;

  /// Jacobians to end effector wrt to camera
  KDL::Jacobian Jce_;

  /// Jacobian solver for @par Jbe_
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  /// Forward kinematic solver for @par Fbe_
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

  /// Inverse Jacobian of @par Jce_
  Eigen::MatrixXd J_pinv_ce_;

  /// Inverse of Jacobian to robot surface point  wrt to robot base
  Eigen::MatrixXd J_pinv_bs_ ;


  /// Repulsive joint vector
  Eigen::VectorXd joint_rep_field_;

  /// Previus repulsive joint vector used for exponential filtration
  Eigen::VectorXd joint_rep_field_prev_;


  /// Measured features vector
  Eigen::VectorXd s_msr_;

  /// Desired features vector
  Eigen::VectorXd s_des_;

  /// Desired features vector
  Eigen::VectorXd s_cmd_obs_dot_;

  /// Command features derivate vector
  Eigen::VectorXd s_cmd_dot_;

  /// First part of command features derivate vector
  Eigen::VectorXd s_cmd1_dot_;

  /// Second part of command features derivate vector
  Eigen::VectorXd s_cmd2_dot_;

  /// Second part of command features derivate vector during interaction whit obstacles
  Eigen::VectorXd s_cmd_obs2_dot_ ;

  /// Interaction matrix for measured features
  Eigen::MatrixXd Lhat_msr_;

  /// Interaction matrix for desired features
  Eigen::MatrixXd Lhat_des_;

  /// Approximated interaction matrix
  Eigen::MatrixXd Lhat_;

  /// First part of approximated interaction matrix
  Eigen::MatrixXd L1hat_;

  /// Second part of approximated interaction matrix
  Eigen::MatrixXd L2hat_;

  /// Gain for visual servoing
  Eigen::VectorXd gama_;

  /// Scaled gain used for simple gain scheduling (not used any more)
  Eigen::VectorXd scaled_gama_;

  /// Pseudo-Inverse of Lhat*Jce
  Eigen::MatrixXd Lhat_Jce_pinv_;

  /// Pseudo-Inverse of L1hat*Jce
  Eigen::MatrixXd L1hat_Jce_pinv_;

  /// number of released features during interaction with obstacles
  int num_released_features_;

  /// Calculated minimum distance between robot and obstacles
  double min_dist_;

  /// Calculated maximum smoothing factor (not used)
  double max_ni1_;


  /**
   * Flags for topics data. status -1 - not not received, status 0 - delayed,
   * status 1 - receive in time and data ok, status 2 - receive in time and not valid data,
  */
  int obstacles_obj_status_;
  int robot_obj_status_;
  int msr_features_status_;
  int des_features_status_;
  int joints_state_status_;


  /// Safety timers robot objects
  ros::Time safety_ton_robot_obj_;

  /// Safety timers obstacles objects
  ros::Time safety_ton_obstacles_obj_;

  /// Safety timers measured features objects
  ros::Time safety_ton_msr_features_;

  /// Safety timers desired features objects
  ros::Time safety_ton_des_features_;

  /// Safety timers joint states
  ros::Time safety_ton_joints_state;

  /// Robot joint number
  unsigned int joint_number_;

  /// Command active
  int cmd_flag_;

  /// Recor interval for all data publisher
  double record_interval_;

  /// Obstacle interaction activated
  bool interaction_active_ ;

  /// Go to initial position
  bool restoring_initial_ ;


  /// Enable usage of combined interaction matrices (from parameter server)
  bool using_combined_matrices_;

  /// Enable usage of external set point (from parameter server)
  bool using_external_set_point_;

  /// Enable usage of transpose jacobian for repulsive vector calculation (from parameter server)
  bool using_transpose_jacobian_;

  /// Enable usage of multiple collision points on robot (from parameter server)
  bool using_multiple_colision_points_;

  /// Enable obstacle avoidance part of combined algorithm  (from parameter server)
  bool enb_obstacle_avoidance_;

  /// Name for the node (from parameter server)
  std::string base_name_;

  /// Robot description path
  std::string robot_description_;

  /// Robot root name
  std::string root_name_;

  /// Robot tip name
  std::string tip_name_;

  /// Features topic name
  std::string features_data_topic_;

  /// Obstacles objects topic name
  std::string obstacles_objects_topic_;

  /// Robot objects topic name
  std::string robot_objects_topic_;

  /// Joint states name
  std::string joint_states_topic_;

  /// Robot command topic
  std::string robot_command_topic_;

  /// Robot trajectory action topic
  std::string robot_trajectory_action_topic_;

  /// Enable testing of obstacles avoidance
  bool test_only_obst_avoidance_;

  /// First distance for interaction
  double ro1_;

  /// Second distance for interaction
  double ro2_;

  /// Maximum admisable velocity
  double Vmax_;

  /// Shaping factor
  double alpha_;

  /// Mutex for feaures callback
  boost::mutex features_mutex_;

  /// Mutex for obstacles objects callback
  boost::mutex obst_mutex_;

  /// Mutex for commmand callback
  boost::mutex cmd_mutex_;

  /// Mutex for joint state callback
  boost::mutex joint_mutex_;

  /// Mutex for robot objects callback
  boost::mutex robot_mutex_;

  /// Desired end effector pose stored at begging of interaction
  KDL::Frame Fbe_des_;


  /**
   * @brief jointPositionCB Calback for joint position
   * @param msg Incoming msg
   */
  void jointPositionCB(const sensor_msgs::JointState &msg);

  /**
   * @brief featureExtractorCB Calback for features data
   * @param msg Incoming msg
   */
  void featureExtractorCB(const imbvs_with_obstacles_avoidance::VisFeature &msg);

  /**
   * @brief obstaclesObjectsCB Calback for obstacles objects
   * @param msg Incoming msg
   */
  void obstaclesObjectsCB(const visualization_msgs::MarkerArray &msg);

  /**
   * @brief robotObjectsCB Calback for robot objects
   * @param msg Incoming msg
   */
  void robotObjectsCB(const visualization_msgs::MarkerArray &msg);

  /**
   * @brief recordAllData Record imbvs data
   */
  void recordAllData();

  /**
   * @brief obstaclesProcesing
   * @param dst_min_dist Output destination for calculated minimum distance
   * @param dst_max_ni1 Output destination for calculated smoothing factor
   */
  void obstaclesProcesing(double &dst_min_dist, double &dst_max_ni1);

  /**
   * @brief calcObjectsFrames Calculate robot and obstacles frames
   * @param joint_to_base_indeces Indeces how every robot objects is related to links tf to base
   */
  void calcObjectsFrames(std::vector<size_t> &link_to_base_indeces);

  /**
   * @brief calcMinDistances Calculates minimal distances betweeen robot objects and obstacles objects
   */
  void calcMinDistances();

  /**
   * @brief getTFs Get @par Fec_ and @par Fwb_
   * @return
   */
  bool getTFs();

  /**
   * @brief testAvoidance Function for testing only avoidance part
   * @param period
   */
  void testAvoidance(ros::Duration period);

};



#endif
