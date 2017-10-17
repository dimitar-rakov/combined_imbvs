
#include <imbvs_with_obstacles_avoidance/imbvs_with_obstacles_avoidance.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <urdf/model.h>
#include <control_toolbox/filters.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <tf_conversions/tf_kdl.h>
#include <angles/angles.h>


ImbvsWithObstaclesAvoidance::ImbvsWithObstaclesAvoidance() {}
ImbvsWithObstaclesAvoidance::~ImbvsWithObstaclesAvoidance() {}


bool ImbvsWithObstaclesAvoidance::init(ros::NodeHandle &n)
{
  nh_ = n;
  urdf::Model model;


  safety_ton_obstacles_obj_.fromSec(-10.0);
  safety_ton_robot_obj_.fromSec(-10.0);
  safety_ton_msr_features_.fromSec(-10.0);
  safety_ton_des_features_.fromSec(-10.0);
  safety_ton_joints_state.fromSec(-10.0);;

  obstacles_obj_status_ = -1;
  robot_obj_status_ = -1;
  joints_state_status_ = -1;
  msr_features_status_ = -1;
  des_features_status_ = -1;

  cmd_flag_ = 0;
  interaction_active_ =false;
  restoring_initial_= false;
  num_released_features_ = 3;


  // Get base_name from parameter server
  if (!nh_.getParam("base_name", base_name_)){
    nh_.param("base_name", base_name_, std::string ("imbvs_with_obstacles_avoidance"));
    ROS_WARN("Parameter base_name was not found. Default name is used: %s ", base_name_.c_str());
  }


  // Get base_name from parameter server
  if (!nh_.getParam("base_name", base_name_)){
    nh_.param("base_name", base_name_, std::string ("imbvs_with_obstacles_avoidance"));
    ROS_WARN("Parameter base_name was not found. Default name is used: %s ", base_name_.c_str());
  }


  // get URDF and name of root and tip from the parameter server
  if (!nh_.getParam("root_name", root_name_)){
    ROS_ERROR_STREAM("ImbvsWithObstaclesAvoidance: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
    return false;
  }

  if (!nh_.getParam("tip_name", tip_name_)){
    ROS_ERROR_STREAM("ImbvsWithObstaclesAvoidance: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
    return false;
  }


  if (!ros::param::search(nh_.getNamespace()+"/robot_description", robot_description_)){
    ROS_ERROR_STREAM("ImbvsWithObstaclesAvoidance: No robot description (URDF) found on parameter server (/robot_description)");
    return false;
  }


  // Construct an URDF model from the xml string
  std::string xml_string;

  if (nh_.hasParam(robot_description_))
    nh_.getParam(robot_description_.c_str(), xml_string);
  else
  {
    ROS_ERROR("Parameter %s not set, shutting down node...", robot_description_.c_str());
    nh_.shutdown();
    return false;
  }

  if (xml_string.size() == 0)
  {
    ROS_ERROR("Unable to load robot model from parameter %s",robot_description_.c_str());
    nh_.shutdown();
    return false;
  }

  ROS_DEBUG("%s content\n%s", robot_description_.c_str(), xml_string.c_str());

  // Get urdf model out of robot_description

  if (!model.initString(xml_string))
  {
    ROS_ERROR("Failed to parse urdf file");
    n.shutdown();
    return false;
  }
  ROS_INFO("Successfully parsed urdf file in ");

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    nh_.shutdown();
    return false;
  }

  // Populate the KDL chain
  if(!kdl_tree.getChain(root_name_, tip_name_, kdl_chain_))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
    ROS_ERROR_STREAM("  "<<root_name_<<" --> "<<tip_name_);
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
    ROS_ERROR_STREAM("  The segments are:");

    KDL::SegmentMap segment_map = kdl_tree.getSegments();
    KDL::SegmentMap::iterator it;

    for( it=segment_map.begin(); it != segment_map.end(); it++ )
      ROS_ERROR_STREAM( "    "<<(*it).first);

    return false;
  }

  ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
  ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
  joint_number_  = kdl_chain_.getNrOfSegments();

  // Parsing joint limits from urdf model along kdl chain
  boost::shared_ptr<const urdf::Link> link = model.getLink(tip_name_);
  boost::shared_ptr<const urdf::Joint> joint;
  joint_limits_.min.resize(joint_number_);
  joint_limits_.max.resize(joint_number_);
  joint_limits_.center.resize(joint_number_);
  joint_limits_.velocity.resize(joint_number_);
  joint_limits_.acceleration.resize(joint_number_);
  joint_limits_.jerk.resize(joint_number_);
  joint_limits_.effort.resize(joint_number_);

  unsigned int index;
  for (unsigned int i = 0; i < joint_number_ && link; i++)
  {
    joint = model.getJoint(link->parent_joint->name);
    ROS_INFO("Getting limits for joint: %s", joint->name.c_str());
    traj_msg_.joint_names.insert(traj_msg_.joint_names.begin(),joint->name);
    tf_names_.insert(tf_names_.begin(),link->name);

    index = joint_number_ - i - 1;
    joint_limits_.min(index) = joint->limits->lower;
    joint_limits_.max(index) = joint->limits->upper;
    joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;
    joint_limits_.velocity(index) = joint->limits->velocity;
    joint_limits_.acceleration(index) = joint->limits->velocity *2.0; // maximum velocity can be achieved in 0.5 seconds
    joint_limits_.effort(index) = joint->limits->effort;
    link = model.getLink(link->getParent()->name);

  }
  tf_names_.insert(tf_names_.begin(),root_name_);


  // Get features_data_topic  from parameter server
  if (!nh_.getParam("features_data_topic", features_data_topic_)){
    nh_.param("features_data_topic", features_data_topic_, std::string ("/visual_features_extractor/visual_features_data"));
    ROS_WARN("Parameter features_data_topic was not found. Default topic's name is used: %s ", features_data_topic_.c_str());
  }

  // Get lwr_states_topic  from parameter server
  if (!nh_.getParam("joint_states_topic", joint_states_topic_)){
    nh_.param("joint_states_topic", joint_states_topic_, std::string ("/joint_states"));
    ROS_WARN("Parameter joint_states_topic was not found. Default topic's name is used: %s ", joint_states_topic_.c_str());
  }

  // Get obstacles_objects_topic  from parameter server
  if (!nh_.getParam("obstacles_objects_topic", obstacles_objects_topic_)){
    nh_.param("obstacles_objects_topic", obstacles_objects_topic_, std::string ("/obstacle_detector/obstacle_objects"));
    ROS_WARN("Parameter obstacles_objects_topic was not found. Default topic's name is used: %s ", obstacles_objects_topic_.c_str());
  }

  // Get robot_objects_topic  from parameter server
  if (!nh_.getParam("robot_objects_topic", robot_objects_topic_)){
    nh_.param("robot_objects_topic", robot_objects_topic_, std::string ("/obstacle_detector/robot_objects"));
    ROS_WARN("Parameter robot_objects_topic was not found. Default topic's name is used: %s ", robot_objects_topic_.c_str());
  }

  // Get robot_command_topic  from parameter server
  if (!nh_.getParam("robot_command_topic", robot_command_topic_)){
    nh_.param("robot_command_topic", robot_command_topic_, std::string ("/lwr/joint_trajectory_controller/command"));
    ROS_WARN("Parameter robot_command_topic was not found. Default topic's name is used: %s ", robot_command_topic_.c_str());
  }

  // Get robot_trajectory_action_topic  from parameter server
  if (!nh_.getParam("robot_trajectory_action_topic", robot_trajectory_action_topic_)){
    nh_.param("robot_trajectory_action_topic", robot_trajectory_action_topic_, std::string ("lwr/joint_trajectory_controller/follow_joint_trajectory"));
    ROS_WARN("Parameter robot_trajectory_action_topic was not found. Default topic's name is used: %s ", robot_trajectory_action_topic_.c_str());
  }

  // Get using_external_set_point  from parameter server
  if (!nh_.getParam("using_external_set_point", using_external_set_point_)){
    nh_.param("using_external_set_point", using_external_set_point_, false);
    ROS_WARN("Parameter using_external_set_point was not found. Default value is used: false");
  }

  // Get using_combined_matrices  from parameter server
  if (!nh_.getParam("using_combined_matrices", using_combined_matrices_)){
    nh_.param("using_combined_matrices", using_combined_matrices_, false);
    ROS_WARN("Parameter using_combined_matrices was not found. Default value is used: false");
  }

  // Get using_transpose_jacobian  from parameter server
  if (!nh_.getParam("using_transpose_jacobian", using_transpose_jacobian_)){
    nh_.param("using_transpose_jacobian", using_transpose_jacobian_, false);
    ROS_WARN("Parameter using_transpose_jacobian was not found. Default value is used: false");
  }

  // Get enb_obstacle_avoidance  from parameter server
  if (!nh_.getParam("enb_obstacle_avoidance", enb_obstacle_avoidance_)){
    nh_.param("enb_obstacle_avoidance", enb_obstacle_avoidance_, false);
    ROS_WARN("Parameter enb_obstacle_avoidance was not found. Default value is used: false");
  }

  // Get using_multiple_colision_points  from parameter server
  if (!nh_.getParam("using_multiple_colision_points", using_multiple_colision_points_)){
    nh_.param("using_multiple_colision_points", using_multiple_colision_points_, false);
    ROS_WARN("Parameter using_multiple_colision_points was not found. Default value is used: false");
  }

  // Get test_only_obstacle_avoidance  from parameter server
  if (!nh_.getParam("test_only_obstacle_avoidance", test_only_obst_avoidance_)){
    nh_.param("test_only_obstacle_avoidance", test_only_obst_avoidance_, false);
    ROS_WARN("Parameter test_only_obstacle_avoidance was not found. Default value is used: false");
  }

  // Get ro_1  from parameter server
  if (!nh_.getParam("ro_1", ro1_)){
    nh_.param("ro_1", ro1_, 1.0);
    ROS_WARN("Parameter ro_1 was not found. Default value is used: %lf", ro1_);
  }

  // Get ro_1  from parameter server
  if (!nh_.getParam("ro_2", ro2_)){
    nh_.param("ro_2", ro2_, ro1_- 0.2*ro1_);
    ROS_WARN("Parameter ro_2 was not found. Default value is used: %lf", ro2_);
  }
  else if (ro2_>ro1_){
    ro2_ = ro1_- 0.2*ro1_;
    ROS_WARN("Parameter ro_2 muss be less than ro1. Default value is used: %lf", ro2_);
  }

  // Get v_max from parameter server
  if (!nh_.getParam("v_max", Vmax_)){
    nh_.param("v_max", Vmax_, 0.5);
    ROS_WARN("Parameter v_max was not found. Default value is used: %lf", Vmax_);
  }

  // Get Vmax_  from parameter server
  if (!nh_.getParam("alpha", alpha_)){
    nh_.param("alpha", alpha_, 7.0);
    ROS_WARN("Parameter alpha was not found. Default value is used: %lf", alpha_);
  }


  gama_.resize(8);
  // Get gama from parameter server
  std::vector<double> gama;
  if (!nh_.getParam("gama", gama)){
    nh_.param("gama", gama, std::vector<double> {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    ROS_WARN("Parameter gama was not found. Default gain is used");
  }
  else if (gama.size() != 6 && gama.size() != 8){
    ROS_WARN("Parameter gama has wrong size: %ld. Default gain is used",gama.size());
    gama = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  }


  for ( unsigned int  i = 0; i < gama.size(); i++)
    gama_(i) = gama[i];
  scaled_gama_ = gama_;


  joint_msr_states_.resize(joint_number_);
  joint_des_states_.resize(joint_number_);
  joint_des_states_prev_.resize(joint_number_);
  joint_init_interaction_states_.resize(joint_number_);
  traj_points_.positions.resize(joint_number_);
  traj_points_.velocities.resize(joint_number_);

  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  Jbe_.resize(joint_number_);
  Jce_.resize(joint_number_);
  Jee_.resize(joint_number_);


  //Repulsive field init
  joint_rep_field_ = Eigen::VectorXd::Zero(7);
  joint_rep_field_prev_ = Eigen::VectorXd::Zero(7);


  sub_command_ = nh_.subscribe("command", 1, &ImbvsWithObstaclesAvoidance::command, this);
  sub_visual_features_data_ = nh_.subscribe(features_data_topic_, 1, &ImbvsWithObstaclesAvoidance::featureExtractorCB, this);
  sub_joint_state_= nh_.subscribe(joint_states_topic_, 1, &ImbvsWithObstaclesAvoidance::jointPositionCB, this);
  sub_markers_obstacles_objects_ = nh_.subscribe(obstacles_objects_topic_, 1, &ImbvsWithObstaclesAvoidance::obstaclesObjectsCB, this);
  sub_markers_robot_objects_ = nh_.subscribe(robot_objects_topic_, 1, &ImbvsWithObstaclesAvoidance::robotObjectsCB, this);
  pub_all_data_ = nh_.advertise<std_msgs::Float64MultiArray>(nh_.getNamespace()+"/all_data", 5);
  pub_joint_traj_ = nh_.advertise<trajectory_msgs::JointTrajectory>( robot_command_topic_, 5 );
  pub_rep_vec_marker_ = nh_.advertise<visualization_msgs::Marker>(nh_.getNamespace()+"/repulsive_vector", 5 );


  if (USE_TRAJ_CLIENT){
    // Initialize traj_client
    traj_client_ptr_.reset(new actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction >(robot_trajectory_action_topic_, true));

    // wait for action server to come up
    while(!traj_client_ptr_->waitForServer(ros::Duration(5.0)))
      ROS_INFO_THROTTLE(1, "Waiting for the joint_trajectory_action server");
  }

  ROS_INFO ("ImbvsWithObstaclesAvoidance with a name %s is initialized", base_name_.c_str());
  return true;
}

void ImbvsWithObstaclesAvoidance::starting(const ros::Time& time) {


  cmd_flag_ = 1;
  ROS_INFO_STREAM("joint_msr_states.q: "<<joint_msr_states_.q.data.transpose());
  ROS_INFO_STREAM("joint_des_states.q: "<<joint_des_states_.q.data.transpose());
  traj_points_.time_from_start.fromSec(10.0);
  traj_points_.velocities.clear();
  traj_points_.accelerations.clear();
  traj_points_.effort.clear();
  fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbe_des_);
  for ( unsigned int  i = 0; i < joint_number_; i++)
    traj_points_.positions[i] = joint_des_states_.q(i);
}

void ImbvsWithObstaclesAvoidance::update(const ros::Time& time, const ros::Duration& period)
{

  // Safety timers
  if ((ros::Time::now()- safety_ton_obstacles_obj_).toSec()< 2.0 && obstacles_obj_status_> -1){ obstacles_obj_status_= 1; }
  else if ((ros::Time::now()- safety_ton_obstacles_obj_).toSec()> 2.0 && obstacles_obj_status_> -1){ obstacles_obj_status_= 0; }
  if (obstacles_obj_status_ == 0) ROS_WARN("Obstacle objects's topic is not longer available");
  else if (obstacles_obj_status_ == -1  && enb_obstacle_avoidance_) ROS_WARN_THROTTLE(5, "Waiting for obstacle objects's topic");

  if ((ros::Time::now()- safety_ton_robot_obj_).toSec()< 2.0 && robot_obj_status_> -1){ robot_obj_status_= 1; }
  else if ((ros::Time::now()- safety_ton_robot_obj_).toSec()> 2.0 && robot_obj_status_> -1){ robot_obj_status_= 0; }
  if (robot_obj_status_ == 0) ROS_WARN("Robot objects's topic is not longer available");
  else if (robot_obj_status_ == -1 && enb_obstacle_avoidance_) ROS_WARN_THROTTLE(5, "Waiting for robot objects's topic");


  if ((ros::Time::now()- safety_ton_msr_features_).toSec()> 2.0 && msr_features_status_> -1){ msr_features_status_= 0; }
  if (msr_features_status_ == 0) ROS_WARN("Measured features's topic is not longer available");
  else if (msr_features_status_ == -1) ROS_WARN_THROTTLE(5, "Waiting for measured features's topic");
  else if (msr_features_status_ == 1) ROS_WARN_THROTTLE(2, "Measured features are out of FOV");

  if ((ros::Time::now()- safety_ton_des_features_).toSec()> 2.0 && des_features_status_> -1){ des_features_status_= 0; }
  if (des_features_status_ == 0) ROS_WARN("Desired features's topic is not longer available");
  else if (des_features_status_ == -1) ROS_WARN_THROTTLE(5, "Waiting for desired features's topic");
  else if (des_features_status_ == 1) ROS_WARN_THROTTLE(2, "Desired features are out of FOV");

  // use   msr joint positions  to
  if (joints_state_status_ == 2 && getTFs())
  {
    // computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbe_);

    // computing Jacobian
    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jbe_);

    // Jacobian camera with respect to the camera Jcc
    /* ref "Comparison of Basic Visual Servoing Methods" F. Janabi-Sharifi and L. Deng and W. J. Wilson


         KDL::Jacobian Jee (joint_number_), Jce (joint_number_);
         Eigen::MatrixXd Xeb = Eigen::MatrixXd::Identity(6,6);
         Eigen::MatrixXd Xce = Eigen::MatrixXd::Identity(6,6);
         Map<Matrix<double, 3, 3, RowMajor> > Reb(Fbe_.Inverse().M.data,3,3);
         Xeb.topLeftCorner(3,3) = Reb;
         Xeb.bottomRightCorner(3,3) = Reb;
         Eigen::Matrix<double,3,3> p_hat;
         KDL::Frame Fce =Fec_.Inverse();
         Map<Matrix<double, 3, 3, RowMajor> > Rce(Fce.M.data,3,3);
         skew_symmetric (Fce.p , p_hat);
        Xce.topLeftCorner(3,3) = Rce;
         Xce.topRightCorner(3,3) = p_hat*Rce;
         Xce.bottomRightCorner(3,3) = Rce;
         Jee.data = Xeb*Jbe_.data;
         Jce.data = Xce*Jee.data;
    */
    KDL::changeBase(Jbe_, Fbe_.Inverse().M, Jee_);
    KDL::changeRefFrame(Jee_, Fec_.Inverse(), Jce_);

    // computing J_pinv_ce_
    pseudo_inverse(Jce_.data, J_pinv_ce_);
    joint_des_states_prev_.q.data = joint_des_states_.q.data;
    joint_des_states_prev_.qdot.data = joint_des_states_.qdot.data;
    joint_des_states_.qdot.data =  Eigen::VectorXd::Zero(joint_number_);  // do not move if there is no setpoint

    // Safety common part
    if (enb_obstacle_avoidance_ && obstacles_obj_status_ == 1 && robot_obj_status_ == 1){
      obstaclesProcesing(min_dist_ , max_ni1_);
      // start control if obstacle appears
      if (min_dist_< ro1_ && cmd_flag_ ==  false){
        cmd_flag_ =  true;
        ROS_WARN_ONCE("Command started, because of obstacle avoidance");
      }
    }

    // imbvs with obstacles avoidance
    if (msr_features_status_ ==2 && des_features_status_ ==2 && cmd_flag_ ==1 && !test_only_obst_avoidance_ && !restoring_initial_)
    {
      //      // Simple Gain Scheduling up to 3 times
      //      if ( (s_des_- s_msr_).squaredNorm() >  0.000002 && (s_des_- s_msr_).squaredNorm() <  0.002)
      //        scaled_gama_ =  gama_ * (1 + 1 * 0.000002 / (s_des_- s_msr_).squaredNorm());
      //      else if ((s_des_- s_msr_).squaredNorm() <=  0.000002)
      //        scaled_gama_ =  gama_ ;
      //      else if ((s_des_- s_msr_).squaredNorm() >=  0.002)
      //        scaled_gama_ =  gama_;

      //      ROS_INFO_STREAM("Norm :"<<(s_des_- s_msr_).squaredNorm()<< "scaled_gama_(0,0): "<<scaled_gama_(0,0));

      // if it used external setpoint Lhat_msr_ = Lhat_des_
      if (using_combined_matrices_)
        Lhat_ = 0.5 * (Lhat_msr_ + Lhat_des_);
      else
        Lhat_ = Lhat_msr_;
      s_cmd_dot_.resize(s_msr_.size());
      for (int i = 0 ; i < s_msr_.size(); i++){
        s_cmd_dot_(i) = -scaled_gama_(i) *(s_msr_(i) - s_des_(i));
        // Moment based VS => s(5) is alpha
        if (i == 5 && s_msr_.size() == 6 )
          s_cmd_dot_(i) = -scaled_gama_(i) * angles::shortest_angular_distance(s_msr_(i), s_des_(i));
      }

      // For test every single features controllability
      //      s_err_dot_ << 0, 0, 0, s_err_dot_(3), 0, 0;

      pseudo_inverse(Lhat_*Jce_.data, Lhat_Jce_pinv_);
      if (enb_obstacle_avoidance_ && obstacles_obj_status_ == 1 && robot_obj_status_ == 1){
        s_cmd1_dot_ = s_cmd_dot_.head(s_cmd_dot_.size() - num_released_features_);
        s_cmd2_dot_ = s_cmd_dot_.tail(num_released_features_);
        L1hat_ = Lhat_.topRows(s_msr_.size() - num_released_features_);
        L2hat_ = Lhat_.bottomRows(num_released_features_);
        pseudo_inverse(L1hat_*Jce_.data, L1hat_Jce_pinv_);
        // Safety part
        double sec_dist =  std::abs(min_dist_) -ro2_;
        sec_dist =(sec_dist < 0)? 0 : sec_dist;
        //sec_dist =(sec_dist > ro1_ -ro2_)? ro1_ -ro2_ : sec_dist;

        double ni2 = 1/(1 + std::exp((sec_dist *2/(ro1_ -ro2_) - 1) * alpha_ ));
        //double ni2 = 1/(1 + std::exp(((std::abs(min_dist_) - ro2_) *2/ro1_ - 1) * alpha_));
        double ni1 = 1/(1 + std::exp((std::abs(min_dist_) *2/ro1_ - 1) * alpha_));

        // By every new interaction with obstacles store the last stable configuration
        if ( !interaction_active_ && min_dist_ <= ro1_ && msr_features_status_ ==2 && des_features_status_ ==2){
          joint_init_interaction_states_ = joint_msr_states_;
          interaction_active_ = true;
        }
        ROS_WARN_COND(min_dist_ < ro1_,"OBSTACLES AVOIDANCE ACTIVE");
        s_cmd_obs2_dot_ = ((1 - ni2) * s_cmd2_dot_) + ni2 * (L2hat_* Jce_.data * L1hat_Jce_pinv_) * s_cmd1_dot_ ;
        Eigen::MatrixXd N = (Eigen::MatrixXd::Identity(joint_number_,joint_number_) - L1hat_Jce_pinv_* (L1hat_*Jce_.data));
        s_cmd_obs_dot_ << s_cmd1_dot_, s_cmd_obs2_dot_;
        joint_des_states_.qdot.data =  (Lhat_Jce_pinv_ * s_cmd_obs_dot_) + (ni2 * N * joint_rep_field_);
        ROS_INFO("ni1 %lf ni2 %lf min_dist_ %lf",ni1, ni2, min_dist_ );
      }
      else
        joint_des_states_.qdot.data =  Lhat_Jce_pinv_ * (s_cmd_dot_);

      // Diffferent on target criteria depending of type of s
      Eigen::MatrixXd err = s_des_- s_msr_;
      Eigen::MatrixXd ranges = 0.0001 * Eigen::VectorXd::Ones(s_des_.size());
      if (s_msr_.size() ==6 ){
        ranges << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001;
        bool err_in_range = true;
        for(size_t i = 0; i < s_msr_.size(); i++)
          err_in_range = (err_in_range && err(i)<ranges(i) && err(i)>-ranges(i))? true : false;
        cmd_flag_ =  (err_in_range)? 0: cmd_flag_;
        ROS_INFO_STREAM_COND(err_in_range, "On target err: "<<err.transpose());
      }

      if (s_msr_.size() ==8){
        ranges << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
        bool err_in_range = true;
        for(size_t i = 0; i < s_msr_.size(); i++)
          err_in_range = (err_in_range && err(i)<ranges(i) && err(i)>-ranges(i))? true : false;
        cmd_flag_ =  (err_in_range)? 0: cmd_flag_;
        ROS_INFO_STREAM_COND(err_in_range, "On target err: "<<err.transpose());
      }

      // Record data at interval 10 ms (always in this case)
      if (record_interval_>= 0.010){
        recordAllData();
        record_interval_ = 0.0;
      }
      else
        record_interval_ += period.toSec();
    }

    // Go to the position when the obstacle interaction first occurs if it is free
    else if ((msr_features_status_ ==1 && des_features_status_ ==2 && cmd_flag_ ==1 && !test_only_obst_avoidance_) || restoring_initial_){

      if(interaction_active_  && min_dist_ > ro1_){

        ROS_WARN("GO TO INITIAL OBSTACLE INTERACTION POSITION ACTIVE");
        Eigen::MatrixXd err = joint_msr_states_.q.data - joint_init_interaction_states_.q.data;

        //Check error range (Eigen (a>0).all() example does not work here. Probably the Eigen version in Ubuntu 14.04 is to old )
        bool err_in_range = true;
        for(size_t i = 0; i < joint_number_; i++)
          err_in_range = (err_in_range && err(i)<0.001 && err(i)>-0.001)? true : false;

        // Very simple controller in order to go till stored joint states
        if (err_in_range){
          interaction_active_ = false;
          restoring_initial_ = false;
        }
        else{
          restoring_initial_ = true;
          joint_des_states_.qdot.data =  -0.3 * err;
        }
      }
      //Otherwise do obstacle avoidane if it is enabled
      else if (enb_obstacle_avoidance_ && min_dist_ <= ro1_){
        joint_des_states_.qdot.data =  joint_rep_field_;

      }
    }

    // Test only obstacle avoidance
    else if (test_only_obst_avoidance_){
      cmd_flag_ = 1;
      testAvoidance(period);
    }

    // joint limits saturation
    for (unsigned int  i = 0;  i < joint_number_; i++)
    {
        // integrating q_dot -> getting q (Trapezoidal method)
        joint_des_states_.q(i) += period.toSec()*0.5*(joint_des_states_prev_.qdot(i)+joint_des_states_.qdot(i));
        // Limit maximmum desired joint step send to robot and max joint position
        double err_dist = 0.0;
        angles::shortest_angular_distance_with_limits(joint_des_states_prev_.q(i), joint_des_states_.q(i), joint_limits_.min(i), joint_limits_.max(i), err_dist);
        if(err_dist >  MAX_JOINT_DES_STEP)
           joint_des_states_.q(i) = joint_des_states_prev_.q(i) + MAX_JOINT_DES_STEP;
        if(err_dist < -MAX_JOINT_DES_STEP)
           joint_des_states_.q(i) = joint_des_states_prev_.q(i) - MAX_JOINT_DES_STEP;

        if (joint_des_states_.q(i) < joint_limits_.min(i) + 0.01){
            joint_des_states_.q(i) = joint_limits_.min(i) + 0.01;
            ROS_WARN_DELAYED_THROTTLE(1,"Activated min position saturation for joint %u", i);
        }
        if (joint_des_states_.q(i) > joint_limits_.max(i) - 0.01){
            joint_des_states_.q(i) = joint_limits_.max(i) - 0.01;
            ROS_WARN_DELAYED_THROTTLE(1,"Activated max position saturation for joint %u", i);
        }
    }


    // set controls for joints
    for (unsigned int  i = 0; i < joint_number_; i++){
      traj_points_.positions[i] = joint_des_states_.q(i);
      traj_points_.velocities[i] = joint_des_states_.qdot(i);
    }
    ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

    traj_points_.time_from_start = period;
    traj_msg_.points.clear();
    traj_msg_.points.push_back(traj_points_);
  }
}

void ImbvsWithObstaclesAvoidance::command(const imbvs_with_obstacles_avoidance::VisualServoing &msg)
{
  //cmd_mutex_.lock();
  for (size_t i = 0; i < msg.s_des.size(); i++)
  {
    if (using_external_set_point_){
      // ToDo consider removing
      //            s_des_[i]= msg.s_des[i];
      //            des_features_status_ = 2 ;
      //            safety_ton_des_features_ = ros::Time::now();
    }
  }
  //cmd_mutex_.unlock();
  ROS_INFO_STREAM("New setpoint for s is: "<<s_des_.transpose());
  cmd_flag_ = 1;
}

void ImbvsWithObstaclesAvoidance::publish(){
  if (!traj_msg_.points.empty()){
    if (USE_TRAJ_CLIENT){
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory = traj_msg_;
      //goal.trajectory .header.stamp = ros::Time::now();
      traj_client_ptr_->sendGoal(goal);
    }
    else
      pub_joint_traj_.publish(traj_msg_);
  }
  if (!all_data_msg.data.empty())
    pub_all_data_.publish (all_data_msg);
}

void ImbvsWithObstaclesAvoidance::featureExtractorCB(const imbvs_with_obstacles_avoidance::VisFeature &msg){

  if (msg.is_valid_msr_feature ){

    // First time received data initializes the size of the used matrices and the gain
    if (msr_features_status_ ==-1 || s_msr_.size() < 6 )
    {
      s_msr_ = VectorXd::Zero(msg.size_s_msr);
      s_cmd_obs_dot_  = VectorXd::Zero(msg.size_s_msr);
      Lhat_msr_ = MatrixXd::Zero(msg.rowsLmsr, msg.colsLmsr);
      ROS_INFO("First msr features data is received");
    }

    // Exponential smoothing filter. Values closer to 0 weight the last smoothed value more heavily
    for (size_t i = 0; i< msg.size_s_msr; i++){
      if (msr_features_status_ ==-1)
        s_msr_(i) = msg.s_msr[i];
      else
        s_msr_(i) = filters::exponentialSmoothing(msg.s_msr[i], s_msr_(i), 0.25);
    }
    for (size_t r = 0; r< msg.rowsLmsr; r++){
      for (size_t c = 0; c< msg.colsLmsr; c++)
        Lhat_msr_(r, c) = msg.L_data_msr[r*msg.colsLmsr + c];
    }

    // for initialization at start set des = msr otherwise is overlapped
    if (des_features_status_ == -1  || s_des_.size() < 6 ) {
      s_des_ = s_msr_;
      Lhat_des_ = Lhat_msr_;
    }

    if (msg.is_valid_des_feature){
      for (size_t i = 0; i< msg.size_s_des; i++)
        s_des_(i) = msg.s_des[i];

      for (size_t r = 0; r< msg.rowsLmsr; r++){
        for (size_t c = 0; c< msg.colsLmsr; c++){
          if (msg.is_valid_des_feature)
            Lhat_des_(r, c) = msg.L_data_des[r*msg.colsLdes + c];
        }
      }
    }
    //        std::cout << std::fixed << std::setprecision(4);
    //        std::cout << "\nData in s_msr:"<<"\n" <<s_msr_.transpose();
    //        std::cout << "\nData in s_des"<<"\n" <<s_des_.transpose();
    //        std::cout << "\nData in L_msr:"<<"\n" <<Lhat_msr_;
    //        std::cout << "\nData in L_des:"<<"\n" <<Lhat_des_<< "\n";
  }
  msr_features_status_ = (msg.is_valid_msr_feature)? 2 : 1;
  safety_ton_msr_features_ = ros::Time::now();
  des_features_status_ = (msg.is_valid_des_feature)? 2 : 1;
  safety_ton_des_features_ = ros::Time::now();

}

void ImbvsWithObstaclesAvoidance::obstaclesObjectsCB(const visualization_msgs::MarkerArray &msg){
  //obs_mutex_.lock();
  markers_obstacles_objects_ = msg;
  obstacles_obj_status_ = 1;
  safety_ton_obstacles_obj_ = ros::Time::now();
  //obs_mutex_.unlock();
}

void ImbvsWithObstaclesAvoidance::robotObjectsCB(const visualization_msgs::MarkerArray &msg){
  //robot_mutex_.lock();
  markers_robot_objects_ = msg;
  robot_obj_status_ = 1;
  safety_ton_robot_obj_ = ros::Time::now();
  // Resize once
  if (Pb_os_min_.size()==0){
    Pb_os_min_.resize(markers_robot_objects_.markers.size());
    Pb_rs_min_.resize(markers_robot_objects_.markers.size());
  }
  //robot_mutex_.unlock();
}

void ImbvsWithObstaclesAvoidance::recordAllData(){

  // All data for rosbag
  all_data_msg.data.clear();
  //measured angles 3-9
  for (unsigned int i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(joint_msr_states_.q(i));

  //desired angles 10-16
  for (unsigned int i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(joint_des_states_.q(i));

  //measured velocity 17-23
  for (unsigned int i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(joint_msr_states_.qdot(i));

  //desired velocity 24-30
  for (unsigned int i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(joint_des_states_.qdot(i));

  //repulsive field velocity(simple 31-37 )
  for (unsigned int i = 0; i < joint_rep_field_.size(); i++)
    all_data_msg.data.push_back(joint_rep_field_(i));


  // Fbe_ 38-43
  double roll, pitch, yaw;
  KDL::Frame Fwc = Fwb_ * Fbe_ * Fec_;
  all_data_msg.data.push_back(Fwc.p.x());
  all_data_msg.data.push_back(Fwc.p.y());
  all_data_msg.data.push_back(Fwc.p.z());
  Fwc.M.GetRPY(roll, pitch, yaw);
  all_data_msg.data.push_back(roll);
  all_data_msg.data.push_back(pitch);
  all_data_msg.data.push_back(yaw);

  if (msr_features_status_ ==2 && des_features_status_ ==2 && !test_only_obst_avoidance_ ){

    //measured features vector (simple 44-49 )
    for (unsigned int i = 0; i < s_msr_.size(); i++)
      all_data_msg.data.push_back(s_msr_(i));

    //desired features vector (simple 50-55 )
    for (unsigned int i = 0; i < s_des_.size(); i++)
      all_data_msg.data.push_back(s_des_(i));


    //error features vector_dot (simple 56-61 )
    for (unsigned int i = 0; i < s_cmd_dot_.size(); i++)
      all_data_msg.data.push_back(s_cmd_dot_(i));

    // Calculate svd of the Jbc_ (simple 62-68 )
    JacobiSVD<MatrixXd> svd1(Jce_.data, ComputeThinU | ComputeThinV);



    Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]");
    std::cout << std::fixed << std::setprecision(4);
    ROS_WARN_STREAM_COND(svd1.singularValues()(5) < 0.001,"Jce_ sigmas "<<
                         std::fixed << std::setprecision(4)<<(svd1.singularValues()).transpose().format(CleanFmt));
    for (long i = 0; i < (svd1.singularValues()).size(); i++)
      all_data_msg.data.push_back((svd1.singularValues())(i));

    // Calculate svd of the  Lhat_*Jce_ (simple 69-75 )
    JacobiSVD<MatrixXd> svd2((Lhat_*Jce_.data), ComputeFullU | ComputeFullV);
    ROS_WARN_STREAM_COND(svd2.singularValues()(5) < 0.001,"Lhat_*Jce_ sigmas: "<<
                         std::fixed << std::setprecision(4)<<(svd2.singularValues()).transpose().format(CleanFmt));
    for (long i = 0; i < (svd2.singularValues()).size(); i++)
      all_data_msg.data.push_back((svd2.singularValues())(i));

    // Calculate svd of the  Lhat_
    JacobiSVD<MatrixXd> svd3((Lhat_), ComputeFullU | ComputeFullV);
    ROS_WARN_STREAM_COND(svd3.singularValues()(5) < 0.001,"Lhat_ sigmas: "<<
                         std::fixed << std::setprecision(4)<<(svd3.singularValues()).transpose().format(CleanFmt));


  }
}

void ImbvsWithObstaclesAvoidance::jointPositionCB(const sensor_msgs::JointState &msg){
  // joint order is ['lwr_a1_joint', 'lwr_a2_joint', 'lwr_a3_joint', 'lwr_a4_joint', 'lwr_a5_joint', 'lwr_a6_joint', 'lwr_e1_joint']
  joint_mutex_.lock();
  int cnt_founded_joints = 0;
  for (unsigned int i = 0; i < traj_msg_.joint_names.size(); i++){
    for (unsigned int j = 0; j < msg.name.size(); j++){
      if ( traj_msg_.joint_names[i] == msg.name[j]){
        joint_msr_states_.q(i) =  msg.position[j];
        joint_msr_states_.qdot(i) =  msg.velocity[j];

        // fill desired joint position and velocities at the starting
        if (joints_state_status_ == -1){
          joint_des_states_.q(i) = joint_msr_states_.q(i);
          joint_des_states_.qdot(i) = 0.0;
          joint_des_states_prev_.q(i) = joint_des_states_.q(i);
          joint_des_states_prev_.qdot(i) = 0.0;
        }
        cnt_founded_joints++;
      }
    }
  }
  if (joints_state_status_ == -1)
    starting(ros::Time::now());
  joints_state_status_ = 1;
  joint_mutex_.unlock();
  if (cnt_founded_joints != static_cast<int>(joint_number_))
    ROS_WARN ("Joint names in joint_states topic do not corespond to joint names in robot model");
  else
    joints_state_status_ = 2;
}



void ImbvsWithObstaclesAvoidance::calcObjectsFrames(std::vector< size_t > &link_to_base_indeces){

  KDL::Frame Fwo, Fbj, Fjs;
  link_to_base_indeces.clear();
  Fbs_.clear();
  Fbo_.clear();

  for (size_t i = 0; i < markers_obstacles_objects_.markers.size(); i++){
    tf::poseMsgToKDL(markers_obstacles_objects_.markers[i].pose, Fwo);
    Fbo_.push_back(Fwb_.Inverse() * Fwo);
  }


  for(size_t i = 0; i< markers_robot_objects_.markers.size(); i++){
    for(size_t j = 0; j< tf_names_.size(); j++){
      if (tf_names_[j] == markers_robot_objects_.markers[i].header.frame_id){
        fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbj, static_cast<int>(j));
        link_to_base_indeces.push_back(j);
        tf::poseMsgToKDL(markers_robot_objects_.markers[i].pose, Fjs);
        Fbs_.push_back(Fbj * Fjs);
      }
    }
  }

}

void ImbvsWithObstaclesAvoidance::calcMinDistances(){
  KDL::Vector min_distance_vector;
  KDL::Vector Pb_os_min, Pb_rs_min;
  double min_dist = 10000; //initialize with 10000 meters away

  for (size_t j = 0; j< Fbs_.size(); j++){
    // radius robot sphere always approximate to sphere with largest axis
    geometry_msgs::Vector3 os_s = markers_robot_objects_.markers[j].scale;
    double rs2 = 0.5 *std::max(std::max(os_s.x, os_s.y),  os_s.z);
    min_dist = 10000;   //initialize with 10000 meters away

    for (size_t i = 0; i< markers_obstacles_objects_.markers.size(); i++){
      // radius obstacle object, always approximate to sphere with largest axis
      geometry_msgs::Vector3 ro_s = markers_obstacles_objects_.markers[i].scale;
      double ro2 = 0.5 * std::max(std::max(ro_s.x, ro_s.y),  ro_s.z);
      double dist_centers = std::abs((Fbo_[i].p - Fbs_[j].p).Norm());

      KDL::Vector Pb_ss, Pb_os;
      //Check if objects does not overlapp
      if (dist_centers > ro2 +rs2){
        // point on body sphere where the line between two objects centers cross the body sphere surface
        Pb_ss =  Fbs_[j].p + rs2/dist_centers * (Fbo_[i].p - Fbs_[j].p );

        // point on obstacle sphere where the line between two objects centers cross the obstacle sphere surface
        Pb_os =  Fbo_[i].p + ro2/dist_centers * (Fbs_[j].p - Fbo_[i].p);
      }
      // Measurments are not correct, just take centers of objects
      else{
         Pb_ss =  Fbs_[j].p;
         Pb_os =  Fbo_[i].p;
      }
      KDL::Vector dist_vec_surfaces = Pb_os - Pb_ss;
      double dist_surfaces = std::abs((dist_vec_surfaces).Norm());

      // find closest obstacle to current robot object
      if (min_dist > dist_surfaces){
        min_dist = dist_surfaces;
        min_distance_vector = dist_vec_surfaces;
        Pb_os_min = Pb_os;
        Pb_rs_min = Pb_ss;
      }
    }

    // Smooth heavily distances to avoid jerk during avoidance
    if( Pb_os_min_[j] != KDL::Vector::Zero() && Pb_rs_min_[j] !=  KDL::Vector::Zero()){
      Pb_os_min_[j] = Pb_os_min;
      Pb_rs_min_[j] = Pb_rs_min;
    }
    else{
      Pb_os_min_[j](0) = filters::exponentialSmoothing(Pb_os_min(0), Pb_os_min_[j](0), 0.8);
          Pb_os_min_[j](1) = filters::exponentialSmoothing(Pb_os_min(1), Pb_os_min_[j](1), 0.8);
          Pb_os_min_[j](2) = filters::exponentialSmoothing(Pb_os_min(2), Pb_os_min_[j](2), 0.8);
          Pb_rs_min_[j](0) = filters::exponentialSmoothing(Pb_rs_min(0), Pb_rs_min_[j](0), 0.8);
          Pb_rs_min_[j](1) = filters::exponentialSmoothing(Pb_rs_min(1), Pb_rs_min_[j](1), 0.8);
          Pb_rs_min_[j](2) = filters::exponentialSmoothing(Pb_rs_min(2), Pb_rs_min_[j](2), 0.8);
    }

  }
}


void ImbvsWithObstaclesAvoidance::obstaclesProcesing(double &dst_min_dist, double &dst_max_ni1){


  std::vector< size_t > joint_to_base_indeces;
  calcObjectsFrames(joint_to_base_indeces);
  calcMinDistances();

  KDL::Frame  Fbj;
  KDL::Jacobian Jbj(joint_number_), Jb_ss(joint_number_);
  Eigen::VectorXd Vrep = Eigen::VectorXd::Zero(6);
  joint_rep_field_prev_ = joint_rep_field_;
  joint_rep_field_ = Eigen::VectorXd::Zero(7);

  double min_dist = 10000; //initialize with 10000 meters away
  double max_ni1 = 0.0;
  size_t min_dist_idx = 0;

  for (size_t j = 0; j< Fbs_.size(); j++){
    KDL::Vector dist_vec_surfaces = Pb_os_min_[j] - Pb_rs_min_[j];
    double dist_surfaces = std::abs((dist_vec_surfaces).Norm());
    double ni1 = 1/(1 + std::exp((std::abs(dist_surfaces) *2/ro1_ - 1) * alpha_));
    if (min_dist > dist_surfaces){
      min_dist = dist_surfaces;
      max_ni1 = ni1;
      min_dist_idx= j;
    }

    if (using_multiple_colision_points_){
      fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbj, static_cast<int>(joint_to_base_indeces[j]));
      jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jbj, static_cast<int>(joint_to_base_indeces[j]));
      KDL::Vector Pj_ss = Fbj.Inverse() * Pb_rs_min_[j];
      KDL::changeRefPoint(Jbj, Pj_ss, Jb_ss);

      // use distance vector as linear velocity and set the angular velocity to zero
      Eigen::VectorXd V = Eigen::VectorXd::Zero(6);
      V(0) = dist_vec_surfaces(0),
          V(1) = dist_vec_surfaces(1),
          V(2) = dist_vec_surfaces(2),
          Vrep = (Vmax_*ni1 / dist_surfaces) * V;
      if (using_transpose_jacobian_)
        joint_rep_field_ += - Jb_ss.data.transpose() * Vrep;
      else {
        pseudo_inverse(Jb_ss.data, J_pinv_bs_);
        joint_rep_field_ += - J_pinv_bs_ * Vrep;      // in original paper is with -J_pinv_bs_ * Vrep
      }
    }
    else if (!using_multiple_colision_points_ && j == Fbs_.size()-1){
      // reduce calculations if only one point is used
      fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbj, static_cast<int>(joint_to_base_indeces[min_dist_idx]));
      jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jbj, static_cast<int>(joint_to_base_indeces[min_dist_idx]));
      KDL::Vector Pj_ss = Fbj.Inverse() * Pb_rs_min_[min_dist_idx];
      KDL::changeRefPoint(Jbj, Pj_ss, Jb_ss);

      // use distance vector as linear velocity and set the angular velocity to zero
      Eigen::VectorXd V = Eigen::VectorXd::Zero(6);
      KDL::Vector min_dist_vec_surfaces = Pb_os_min_[min_dist_idx] - Pb_rs_min_[min_dist_idx];
      V(0) = min_dist_vec_surfaces(0),
          V(1) = min_dist_vec_surfaces(1),
          V(2) = min_dist_vec_surfaces(2),
          Vrep = (max_ni1 / min_dist) * V;
      if (using_transpose_jacobian_)
        joint_rep_field_ = -Jb_ss.data.transpose() * Vrep;
      else {
        pseudo_inverse(Jb_ss.data, J_pinv_bs_);
        joint_rep_field_ = -J_pinv_bs_ * Vrep;      // in original paper is with -J_pinv_bs_ * Vrep
      }
    }
  }

  // Exponential smoothing filter for joint_rep_field_. Values closer to 0 weight the last smoothed value more heavily
  for (long i = 0; i< joint_rep_field_.size(); i++)
    joint_rep_field_(i) = filters::exponentialSmoothing(joint_rep_field_(i), joint_rep_field_prev_(i), 0.6);
  dst_min_dist = min_dist;
  dst_max_ni1 = max_ni1;

  if (markers_robot_objects_.markers.size()){
    // Publish the vector which repesents the closest to robot obstacle
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;
    marker.header.frame_id = "lwr_base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "repulsive vector";
    marker.id = 348;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x =  0.02;
    marker.scale.y =  0.03;
    marker.scale.z =  0.05;
    marker.color.a = 0.75; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(1);
    tf::pointKDLToMsg(Pb_os_min_[min_dist_idx], p);
    marker.points.clear();
    marker.points.push_back(p);
    tf::pointKDLToMsg(Pb_rs_min_[min_dist_idx], p);
    marker.points.push_back(p);
    pub_rep_vec_marker_.publish(marker);
  }

}

bool ImbvsWithObstaclesAvoidance::getTFs(){
  tf::StampedTransform transform;
  try{
    lr_.lookupTransform("lwr_a6_link", "eye_in_hand", ros::Time(0), transform);
    tf::transformTFToKDL(transform, Fec_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }

  try{
    lr_.lookupTransform("world", "lwr_base_link", ros::Time(0), transform);
    tf::transformTFToKDL(transform, Fwb_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
  return  true;

}

void ImbvsWithObstaclesAvoidance::testAvoidance(ros::Duration period){

  if (cmd_flag_ ==1)
  {

    Eigen:: MatrixXd Jbe_pin, Jbe1, Jbe2,  Jbe1_pin, Jbe2_pin;
    Eigen::VectorXd x_err_dot, x_err1_dot, x_err2_dot, x_err_obs_dot, x_err_obs2_dot;
    pseudo_inverse(Jbe_.data, Jbe_pin);
    joint_des_states_prev_.q.data = joint_des_states_.q.data;
    joint_des_states_prev_.qdot.data = joint_des_states_.qdot.data;
    KDL::Twist x_err_ = diff(Fbe_, Fbe_des_);
    x_err_dot.resize(6);
    x_err_obs_dot.resize(6);
    x_err_dot <<  x_err_.vel.x(), x_err_.vel.y(), x_err_.vel.z(), x_err_.rot.x(), x_err_.rot.y(), x_err_.rot.z();

    if (enb_obstacle_avoidance_ && obstacles_obj_status_ == 1 && robot_obj_status_ == 1){
      x_err1_dot = x_err_dot.head(x_err_dot.size() - num_released_features_);
      x_err2_dot = x_err_dot.tail(num_released_features_);
      Jbe1 = Jbe_.data.topRows(x_err1_dot.size());
      Jbe2 = Jbe_.data.bottomRows(x_err2_dot.size());
      pseudo_inverse(Jbe1, Jbe1_pin);

      // Safety part
      double ni2 = 1/(1 + std::exp(((std::abs(min_dist_) -ro2_) *2/ro1_ - 1) * alpha_));

      ROS_WARN_COND(min_dist_ < ro1_,"OBSTACLES AVOIDANCE ACTIVE");

      x_err_obs2_dot = ((1 - ni2) * x_err2_dot) + ni2 * (Jbe2 * Jbe1_pin) * x_err1_dot ;
      Eigen::MatrixXd N = (Eigen::MatrixXd::Identity(joint_number_,joint_number_) - Jbe1_pin* (Jbe1));
      x_err_obs_dot << x_err1_dot, x_err_obs2_dot;
      //joint_des_states_.qdot.data =  (Jbe_pin * x_err_obs_dot) + (max_ni * N * joint_rep_field_);
      joint_des_states_.qdot.data =  ( Jbe_pin * x_err_dot) + joint_rep_field_;
    }

    if (Equal(Fbe_, Fbe_des_, 0.005))
    {
      ROS_INFO("On target");
    }

  }
}
