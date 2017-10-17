#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <lwr_controllers/adaptive_torque_controller.h>

namespace lwr_controllers 
{
AdaptiveTorqueController::AdaptiveTorqueController() {}
AdaptiveTorqueController::~AdaptiveTorqueController() {}

bool AdaptiveTorqueController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

  nh_.getParam("/lwr/adaptive_torque_controller/enb_record_all_data" ,enb_record_all_data_);
  if (enb_record_all_data_) {ROS_INFO("The data recording for AdaptiveTorqueController controller is enabled");}
  else{enb_record_all_data_= false;}

  nh_.getParam("/lwr/adaptive_torque_controller/enb_record_all_data" ,enb_adapting_);
  if (enb_adapting_) {ROS_INFO("The adaptation for AdaptiveTorqueController is enabled");}
  else{enb_adapting_= false;}


  cmd_states_.resize(kdl_chain_.getNrOfJoints());
  init_states_.resize(kdl_chain_.getNrOfJoints());
  tau_cmd_.resize(kdl_chain_.getNrOfJoints());
  Kp_.resize(kdl_chain_.getNrOfJoints());
  Ki_.resize(kdl_chain_.getNrOfJoints());
  Kd_.resize(kdl_chain_.getNrOfJoints());


  joint_ref_states_.resize(kdl_chain_.getNrOfJoints());
  Sq_.resize(kdl_chain_.getNrOfJoints());
  delta_.resize(kdl_chain_.getNrOfJoints());
  delta_int_.resize(kdl_chain_.getNrOfJoints());
  delta_prev_.resize(kdl_chain_.getNrOfJoints());
  Yr_ = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(),7) ;
  theta_= Eigen::VectorXd::Zero(7);
  theta_dot_ = Eigen::VectorXd::Zero(7);
  theta_dot_prev_ = Eigen::VectorXd::Zero(7);

  sub_posture_ = nh_.subscribe("command", 1, &AdaptiveTorqueController::command, this);
  sub_gains_ = nh_.subscribe("set_gains", 1, &AdaptiveTorqueController::set_gains, this);
  pub_all_data_= nh_.advertise<std_msgs::Float64MultiArray>("/lwr/adaptive_torque_controller/all_data", 10);


  // for real robot PID default
  Kp_(0) = 11.00;
  Kp_(1) = 11.00;
  Kp_(2) = 9.00;
  Kp_(3) = 11.00;
  Kp_(4) = 8.00;
  Kp_(5) = 9.00;
  Kp_(6) = 9.00;

  Ki_(0) = 0.2*Kp_(0)*Kp_(0);
  Ki_(1) = 0.2*Kp_(1)*Kp_(1);
  Ki_(2) = 0.2*Kp_(2)*Kp_(2);
  Ki_(3) = 0.2*Kp_(3)*Kp_(3);
  Ki_(4) = 0.2*Kp_(4)*Kp_(4);
  Ki_(5) = 0.2*Kp_(5)*Kp_(5);
  Ki_(6) = 0.2*Kp_(6)*Kp_(6);

  Kd_(0) = 10.00;
  Kd_(1) = 10.00;
  Kd_(2) = 10.00;
  Kd_(3) = 10.00;
  Kd_(4) = 10.00;
  Kd_(5) = 10.00;
  Kd_(6) = 10.00;

  theta_<< 0.026451,   0.079353,   0.105804,   0.105804,   0.057207,   0.057207,   0.062513;

  XmlRpc::XmlRpcValue gains;
  std::string a = std::string(n.getNamespace())+"/Kp";
  std::cout << "Namespace is "<<a<<"\n";

  // Kp gain from yaml
  if ( !nh_.getParam(std::string(n.getNamespace())+"/Kp", gains ) || gains.getType() != XmlRpc::XmlRpcValue::TypeArray){
    ROS_WARN("Proportional gain is not properly set in yaml file");
    std::cout <<"Using: "<<Kp_.data.transpose()<<"\n";
  }
  else{
    for (int i = 0; i < joint_handles_.size(); ++i){
      if ( gains[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        ROS_WARN("Proportional gain Kp(%d) is not properly set in yaml file, Using %f", i, Kp_(i));
      else
        Kp_(i) = static_cast <double>(gains[i]);
    }
    std::cout <<"Proportional gain: "<<Kp_.data.transpose()<<"\n";
  }

  // Ki gain from yaml

  if ( !nh_.getParam("Ki", gains ) || gains.getType() != XmlRpc::XmlRpcValue::TypeArray){
    ROS_WARN("Integral gain is not properly set in yaml file");
    std::cout <<"Using: "<<Ki_.data.transpose()<<"\n";
  }
  else{
    for (int i = 0; i < joint_handles_.size(); ++i){
      if ( gains[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        ROS_WARN("Integral gain Ki(%d) is not properly set in yaml file, Using %f", i, Ki_(i));
      else
        Ki_(i) = static_cast <double>(gains[i]);
    }
    std::cout <<"Integral gain: "<<Ki_.data.transpose()<<"\n";
  }

  // Kd gain from yaml
  if ( !nh_.getParam("Kd", gains ) || gains.getType() != XmlRpc::XmlRpcValue::TypeArray){
    ROS_WARN("Damping gain is not properly set in yaml file");
    std::cout <<"Using: "<<Kd_.data.transpose()<<"\n";
  }
  else{
    for (int i = 0; i < joint_handles_.size(); ++i){
      if ( gains[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        ROS_WARN("Damping gain Kd(%d) is not properly set in yaml file, Using %f", i, Kd_(i));
      else
        Kd_(i) = static_cast <double>(gains[i]);
    }
    std::cout <<"Damping gain: "<<Kd_.data.transpose()<<"\n";
  }

  // theta from yaml
  if ( !nh_.getParam("theta", gains ) || gains.getType() != XmlRpc::XmlRpcValue::TypeArray){
    ROS_WARN("theta is not properly set in yaml file");
    std::cout <<"Using: "<<theta_.transpose()<<"\n";
  }
  else{
    for (int i = 0; i < theta_.size(); ++i){
      if ( gains[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        ROS_WARN("theta(%d) is not properly set in yaml file, Using %f", i, theta_(i));
      else
        theta_(i) = static_cast <double>(gains[i]);
    }
    std::cout <<"theta: "<<theta_.transpose()<<"\n";
  }


  // get joint positions
  for(size_t i=0; i<joint_handles_.size(); i++)
  {
    joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    joint_msr_states_.qdotdot(i) = 0.0;
    joint_des_states_.q(i) = joint_msr_states_.q(i);
  }

  calcYr();
  Yr_theta_.data = Yr_*theta_;

  for(size_t i=0; i<joint_handles_.size(); i++)
  {
    tau_cmd_(i) = Yr_theta_(i);
    joint_handles_[i].setCommand(tau_cmd_(i));
    cmd_states_(i) = joint_msr_states_.q(i);
    init_states_(i)= joint_msr_states_.q(i);
    delta_(i)= joint_msr_states_.q(i) - joint_des_states_.q(i);
    delta_int_(i)= 0.0;
  }

  ROS_INFO("adaptive_torque_controller is initilized.");
  return true;
}

void AdaptiveTorqueController::starting(const ros::Time& time)
{

  // get joint positions
  std::cout << "Initial joints position is: ";
  for(size_t i=0; i<joint_handles_.size(); i++)
  {
    joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    joint_msr_states_.qdotdot(i) = 0.0;
    joint_des_states_.q(i) = joint_msr_states_.q(i);
    std::cout << init_states_(i)<<"  ";
  }
  std::cout <<std::endl;

  calcYr();
  Yr_theta_.data = Yr_*theta_;

  for(size_t i=0; i<joint_handles_.size(); i++)
  {
    tau_cmd_(i) = Yr_theta_(i);
    joint_handles_[i].setCommand(tau_cmd_(i));
    cmd_states_(i) = joint_msr_states_.q(i);
    init_states_(i)= joint_msr_states_.q(i);
    delta_(i)= joint_msr_states_.q(i) - joint_des_states_.q(i);
    delta_int_(i)= 0.0;
  }

  spline_time_= 25.0; //spline time  in sec for the full range error (-pi, pi)
  lambda = 2*M_PI/spline_time_;	// lower values: flatter.
  cmd_flag_ = 0;
  timer_temp_= 0.0;
  timer02ms_ =0.0;
  timer10ms_ = 0.0;


  ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );

}

void AdaptiveTorqueController::update(const ros::Time& time, const ros::Duration& period)
{
  // get joint positions
  for(size_t i=0; i<joint_handles_.size(); i++)
  {
    joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    joint_msr_states_.qdotdot(i) = 0.0;
  }

  if(cmd_flag_)
  {
    // reaching desired joint position using a hyperbolic tangent function
    for(size_t i=0; i<joint_handles_.size(); i++)
    {
      joint_des_states_.q(i) = init_states_(i) + (cmd_states_(i) - init_states_(i))*0.5*(1.0+ tanh(lambda*timer_temp_-M_PI));
      joint_des_states_.qdot(i) = (cmd_states_(i) - init_states_(i))*0.5*lambda*(1.0/(cosh(M_PI-lambda*timer_temp_)*cosh(M_PI-lambda*timer_temp_))); // 1/(cosh^2) = sech^2
      joint_des_states_.qdotdot(i) = (cmd_states_(i) - init_states_(i))*lambda*lambda*(1.0/(cosh(M_PI-timer_temp_)*cosh(M_PI-timer_temp_)))*tanh(M_PI-timer_temp_);

      if( std::abs(joint_msr_states_.q(i) - cmd_states_(i))<0.0005)
      {
        //cmd_flag_ = 0;	//reset command flag
      }
      else
      {
        cmd_flag_ = 1;	//still not on the goal
      }

    }
    if(!cmd_flag_)
    {
      timer_temp_=0.0;
      ROS_INFO("Posture OK");
    }
  }

  for(size_t i=0; i<joint_handles_.size(); i++) {
    delta_prev_(i)= delta_(i);
    delta_(i)= joint_msr_states_.q(i) - joint_des_states_.q(i);
    delta_int_(i) += period.toSec()*0.5*(delta_prev_(i) + delta_(i));
    // PD like controler
    joint_ref_states_.qdot(i) = joint_des_states_.qdot(i) - Kp_(i)*(delta_(i));
    // PID like controler
    joint_ref_states_.qdot(i) = joint_des_states_.qdot(i) - Kp_(i)*delta_(i) - Ki_(i)*delta_int_(i);
    Sq_(i) = joint_msr_states_.qdot(i) - joint_ref_states_.qdot(i);
  }

  calcYr();
  if (enb_adapting_)
    calcTheta(period, 0.001);

  Yr_theta_.data = Yr_*theta_;

  for(size_t i=0; i<joint_handles_.size(); i++)
  {
    // control law
    tau_cmd_(i) = -Kd_(i)*Sq_(i) +Yr_theta_(i);
    // SENTINEL

    if (tau_cmd_(i) < -joint_limits_.effort(i)){
      ROS_WARN ("Torque lower bound joint %lu . The desired value was %.4lf", i, tau_cmd_(i) );
      tau_cmd_(i) = -joint_limits_.effort(i);

    }
    if (tau_cmd_(i) > joint_limits_.effort(i)) {
      ROS_WARN ("Torque upper bound joint %lu . The desired value was %.4lf", i, tau_cmd_(i) );
      tau_cmd_(i) = joint_limits_.effort(i);
    }
    joint_handles_[i].setCommand(tau_cmd_(i));
  }

  timer02ms_ += period.toSec();
  timer10ms_ += period.toSec();
  // pulse timer at 2ms
  if (timer02ms_>= 0.002){
    //record and publish all data at every 2 ms interval
    if (enb_record_all_data_){
      recordAllData(time, period);
      pub_all_data_.publish (all_data_msg);
    }
    timer02ms_= 0.0;
  }

  // pulse timer at 10ms used for the hyperbolic tangent function
  if (timer10ms_>= 0.010){
    timer_temp_+=0.010;
    timer10ms_= 0.0;
  }
}

void AdaptiveTorqueController::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if(msg->data.size() == 0)
    ROS_INFO("Desired configuration must be of dimension %lu", joint_handles_.size());
  else if(msg->data.size() != joint_handles_.size())
  {
    ROS_ERROR("Posture message had the wrong size: %u", (unsigned int)msg->data.size());
    return;
  }
  else
  {
    double max_error = 0.0;
    for (unsigned int i = 0; i<joint_handles_.size(); i++){
      cmd_states_(i) = msg->data[i];
      init_states_(i) = joint_des_states_.q(i);
      max_error = std::max(max_error, std::abs(cmd_states_(i)-init_states_(i)));
    }

    cmd_flag_ = 1;
    timer_temp_= 0.0;
    lambda = 4*M_PI/(spline_time_ *(max_error) + 0.001);	// scaling lambda up to the spline_time_ for the max position error
  }

}

void AdaptiveTorqueController::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if(msg->data.size() == joint_handles_.size())
  {
    for(unsigned int i = 0; i < joint_handles_.size(); i++)
    {
      Kp_(i) = msg->data[i];
      Ki_(i) = 0.2*Kp_(i)*Kp_(i);
    }
  }
  else if (msg->data.size() == 2*joint_handles_.size()){
    for(unsigned int i = 0; i < joint_handles_.size(); i++)
    {
      Kp_(i) = msg->data[i];
      Ki_(i) = msg->data[i + joint_handles_.size()];
    }

  }
  else
    ROS_INFO("Wrong gains size. Number of Joint handles muss be %lu, or %lu" , joint_handles_.size(), 2*joint_handles_.size());

  ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

  ROS_INFO("New gains Kp: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5), Kp_(6));
  ROS_INFO("New gains Ki: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Ki_(0), Ki_(1), Ki_(2), Ki_(3), Ki_(4), Ki_(5), Ki_(6));

}

void AdaptiveTorqueController::recordAllData(const ros::Time &time, const ros::Duration &period){
  // All data for rosbag
  all_data_msg.data.clear();
  //measured angles 3-9
  for (int i = 0; i < joint_handles_.size(); i++)
    all_data_msg.data.push_back(joint_msr_states_.q(i));

  //desired angles 10-16
  for (int i = 0; i < joint_handles_.size(); i++)
    all_data_msg.data.push_back(joint_des_states_.q(i));

  //measured velocity 17-23
  for (int i = 0; i < joint_handles_.size(); i++)
    all_data_msg.data.push_back(joint_msr_states_.qdot(i));

  //desired velocity 24-30
  for (int i = 0; i < joint_handles_.size(); i++)
    all_data_msg.data.push_back(joint_des_states_.qdot(i));


  // measured torques 31-37
  for (int i = 0; i < joint_handles_.size(); i++)
    all_data_msg.data.push_back(joint_handles_[i].getEffort());


  // desired torques 38-44
  for (int i = 0; i < joint_handles_.size(); i++)
    all_data_msg.data.push_back(tau_cmd_(i));

  // Sq 45-51
  for (int i = 0; i < joint_handles_.size(); i++)
    all_data_msg.data.push_back(Sq_(i));


  // Theta  52-58
  for (int i = 0; i < theta_.size(); i++)
    all_data_msg.data.push_back(theta_(i));
}

void AdaptiveTorqueController::calcYr(){

  double s1 = sin(joint_msr_states_.q(0));
  double s2 = sin(joint_msr_states_.q(1));
  double s3 = sin(joint_msr_states_.q(2));
  double s4 = sin(joint_msr_states_.q(3));
  double s5 = sin(joint_msr_states_.q(4));
  double s6 = sin(joint_msr_states_.q(5));
  double s7 = sin(joint_msr_states_.q(6));


  double c1 = cos(joint_msr_states_.q(0));
  double c2 = cos(joint_msr_states_.q(1));
  double c3 = cos(joint_msr_states_.q(2));
  double c4 = cos(joint_msr_states_.q(3));
  double c5 = cos(joint_msr_states_.q(4));
  double c6 = cos(joint_msr_states_.q(5));
  double c7 = cos(joint_msr_states_.q(6));
  double gx = gravity_(0);
  double gy = gravity_(1);
  double gz = gravity_(2);

  Yr_(0,0) = 0.25*gx*s1*s2 - 0.25*c1*gy*s2 ;
  Yr_(0,1) = 0.75*gx*s1*s2 - 0.75*c1*gy*s2 ;
  Yr_(0,2) = gx*s1*s2 - c1*gy*s2 ;
  Yr_(0,3) = gx*s1*s2 - c1*gy*s2 ;
  Yr_(0,4) = c4*gx*s1*s2 - c1*gx*s3*s4 - gy*s1*s3*s4 - c1*c4*gy*s2 + c1*c2*c3*gy*s4 - c2*c3*gx*s1*s4 ;
  Yr_(0,5) = c4*gx*s1*s2 - c1*gx*s3*s4 - gy*s1*s3*s4 - c1*c4*gy*s2 + c1*c2*c3*gy*s4 - c2*c3*gx*s1*s4 ;
  Yr_(0,6) = c4*c6*gx*s1*s2 - c1*c4*c6*gy*s2 - c1*c6*gx*s3*s4 + c1*c3*gx*s5*s6 - c6*gy*s1*s3*s4 + c3*gy*s1*s5*s6 + c1*c2*c3*c6*gy*s4 - c2*c3*c6*gx*s1*s4 + c1*c4*c5*gx*s3*s6 + c1*c2*gy*s3*s5*s6 - c1*c5*gy*s2*s4*s6 + c4*c5*gy*s1*s3*s6 - c2*gx*s1*s3*s5*s6 + c5*gx*s1*s2*s4*s6 - c1*c2*c3*c4*c5*gy*s6 + c2*c3*c4*c5*gx*s1*s6 ;
  Yr_(1,0) = - 0.25*gz*s2 - 0.25*c1*c2*gx - 0.25*c2*gy*s1 ;
  Yr_(1,1) = - 0.75*gz*s2 - 0.75*c1*c2*gx - 0.75*c2*gy*s1 ;
  Yr_(1,2) = - gz*s2 - c1*c2*gx - c2*gy*s1 ;
  Yr_(1,3) = - gz*s2 - c1*c2*gx - c2*gy*s1 ;
  Yr_(1,4) = c2*c3*gz*s4 - c1*c2*c4*gx - c2*c4*gy*s1 - c4*gz*s2 - c1*c3*gx*s2*s4 - c3*gy*s1*s2*s4 ;
  Yr_(1,5) = c2*c3*gz*s4 - c1*c2*c4*gx - c2*c4*gy*s1 - c4*gz*s2 - c1*c3*gx*s2*s4 - c3*gy*s1*s2*s4 ;
  Yr_(1,6) = c2*c3*c6*gz*s4 - c1*c2*c4*c6*gx - c2*c4*c6*gy*s1 - c4*c6*gz*s2 + c2*gz*s3*s5*s6 - c5*gz*s2*s4*s6 - gy*s1*s2*s3*s5*s6 - c2*c3*c4*c5*gz*s6 - c1*c3*c6*gx*s2*s4 - c1*c2*c5*gx*s4*s6 - c3*c6*gy*s1*s2*s4 - c2*c5*gy*s1*s4*s6 - c1*gx*s2*s3*s5*s6 + c1*c3*c4*c5*gx*s2*s6 + c3*c4*c5*gy*s1*s2*s6 ;
  Yr_(2,4) = c1*c3*gy*s4 - gz*s2*s3*s4 - c3*gx*s1*s4 - c1*c2*gx*s3*s4 - c2*gy*s1*s3*s4 ;
  Yr_(2,5) = c1*c3*gy*s4 - gz*s2*s3*s4 - c3*gx*s1*s4 - c1*c2*gx*s3*s4 - c2*gy*s1*s3*s4 ;
  Yr_(2,6) = c1*c3*c6*gy*s4 - c3*c6*gx*s1*s4 + c1*gy*s3*s5*s6 - c6*gz*s2*s3*s4 + c3*gz*s2*s5*s6 - gx*s1*s3*s5*s6 - c1*c3*c4*c5*gy*s6 - c1*c2*c6*gx*s3*s4 + c1*c2*c3*gx*s5*s6 + c3*c4*c5*gx*s1*s6 - c2*c6*gy*s1*s3*s4 + c2*c3*gy*s1*s5*s6 + c4*c5*gz*s2*s3*s6 + c1*c2*c4*c5*gx*s3*s6 + c2*c4*c5*gy*s1*s3*s6 ;
  Yr_(3,4) = c1*gx*s2*s4 - c2*gz*s4 - c4*gx*s1*s3 + gy*s1*s2*s4 + c1*c4*gy*s3 + c3*c4*gz*s2 + c1*c2*c3*c4*gx + c2*c3*c4*gy*s1 ;
  Yr_(3,5) = c1*gx*s2*s4 - c2*gz*s4 - c4*gx*s1*s3 + gy*s1*s2*s4 + c1*c4*gy*s3 + c3*c4*gz*s2 + c1*c2*c3*c4*gx + c2*c3*c4*gy*s1 ;
  Yr_(3,6) = c1*c4*c6*gy*s3 - c2*c6*gz*s4 + c3*c4*c6*gz*s2 + c2*c4*c5*gz*s6 + c1*c6*gx*s2*s4 - c4*c6*gx*s1*s3 + c6*gy*s1*s2*s4 + c1*c2*c3*c4*c6*gx + c2*c3*c4*c6*gy*s1 - c1*c4*c5*gx*s2*s6 - c4*c5*gy*s1*s2*s6 + c1*c5*gy*s3*s4*s6 + c3*c5*gz*s2*s4*s6 - c5*gx*s1*s3*s4*s6 + c1*c2*c3*c5*gx*s4*s6 + c2*c3*c5*gy*s1*s4*s6 ;
  Yr_(4,6) = s6*(c5*gz*s2*s3 - c2*gz*s4*s5 - c1*c3*c5*gy + c3*c5*gx*s1 + c1*c2*c5*gx*s3 + c2*c5*gy*s1*s3 + c1*c4*gy*s3*s5 + c3*c4*gz*s2*s5 + c1*gx*s2*s4*s5 - c4*gx*s1*s3*s5 + gy*s1*s2*s4*s5 + c1*c2*c3*c4*gx*s5 + c2*c3*c4*gy*s1*s5) ;
  Yr_(5,6) = c2*c5*c6*gz*s4 - c1*c3*c6*gy*s5 - c2*c4*gz*s6 + c1*c4*gx*s2*s6 + c3*c6*gx*s1*s5 + c4*gy*s1*s2*s6 - c1*gy*s3*s4*s6 - c3*gz*s2*s4*s6 + c6*gz*s2*s3*s5 + gx*s1*s3*s4*s6 - c1*c4*c5*c6*gy*s3 - c3*c4*c5*c6*gz*s2 - c1*c2*c3*gx*s4*s6 + c1*c2*c6*gx*s3*s5 - c1*c5*c6*gx*s2*s4 + c4*c5*c6*gx*s1*s3 - c2*c3*gy*s1*s4*s6 + c2*c6*gy*s1*s3*s5 - c5*c6*gy*s1*s2*s4 - c1*c2*c3*c4*c5*c6*gx - c2*c3*c4*c5*c6*gy*s1 ;



}

void AdaptiveTorqueController::calcTheta(const ros::Duration &period, double inv_Gama){
  //calculate the derivate of theta vector
  Eigen::MatrixXd transpose_Yr = Yr_.transpose();
  if(Sq_.data.norm() < 0.01){
    theta_dot_.setZero();
  }
  else{
    theta_dot_ = -inv_Gama*transpose_Yr*Sq_.data;
  }

  // integrate theta_dot using Trapezoidal method
  theta_ += 0.5 * period.toSec() * (theta_dot_prev_ + theta_dot_);
  theta_dot_prev_ = theta_dot_;

}



}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::AdaptiveTorqueController, controller_interface::ControllerBase)
