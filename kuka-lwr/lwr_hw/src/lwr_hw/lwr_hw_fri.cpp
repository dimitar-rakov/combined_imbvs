#include "lwr_hw/lwr_hw_fri.h"

// tools
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <chrono>
#include <mutex>

// ToDo: add timeouts to all sync-while's to KRL since the UDP connection might be lost and we will know

namespace lwr_hw
{

LWRHWFRI::LWRHWFRI():
  LWRHW(),
  port_set_(false),
  ip_set_ (false),
  stopKRCComm_ (false),
  controlStarted_(false)
{ }

LWRHWFRI::~LWRHWFRI() { stopKRCComm_ = true; KRCCommThread_.get()->join(); }

void LWRHWFRI::setPort(int port){port_ = port; port_set_ = true; }

void LWRHWFRI::setIP(std::string hintToRemoteHost){ hintToRemoteHost_ = hintToRemoteHost; ip_set_ = true; }

// Init, read, and write, with FRI hooks
bool LWRHWFRI::init(ros::NodeHandle &nh)
{
  nh_ = nh;
  if( !(port_set_) || !(ip_set_) ){
    std::cout << "Did you forget to set the port/ip?" << std::endl << "You must do that before init()" << std::endl << "Exiting..." << std::endl;
    return false;
  }

  // construct a low-level lwr
  device_.reset( new friRemote( port_, const_cast<char*>(hintToRemoteHost_.c_str()) ) );

  // initialize FRI values
  lastQuality_ = FRI_QUALITY_BAD;
  lastCtrlScheme_ = FRI_CTRL_OTHER;

  std::cout << "Opening FRI Version "
            << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR
            << " Interface for LWR ROS server" << std::endl;

  std::cout << "Checking if the robot is Stopped..." << std::endl;
  device_->doReceiveData();
  if( device_->getState() == FRI_STATE_OFF )
    std::cout << "Please, start the KRL script now." << std::endl;

  ///d.r
  fri_last_timestamp = device_->getTimestamp();
  send_comm_data_.data.resize(n_joints_+2);
  return true;
}

void LWRHWFRI::read(ros::Time time, ros::Duration period)
{

  float joint_msr_position[n_joints_];
  float joint_msr_effort[n_joints_];
  device_->getMsrMsrJntPosition(joint_msr_position);
  device_->getMsrJntTrq(joint_msr_effort);

  for (size_t j = 0; j < n_joints_; j++){
    joint_msr_position_prev_[j] = joint_msr_position_[j];
    joint_msr_velocity_prev_[j] = joint_msr_velocity_[j];
    joint_msr_position_[j] = joint_msr_position[j];
    joint_position_kdl_(j) = joint_msr_position_[j];
    joint_msr_effort_[j] = joint_msr_effort[j];
    if (fri_last_timestamp != device_->getTimestamp()){
      joint_msr_velocity_[j] = (joint_msr_position_[j]-joint_msr_position_prev_[j])/std::abs(device_->getTimestamp() -fri_last_timestamp);
      joint_msr_velocity_[j] = filters::exponentialSmoothing(joint_msr_velocity_[j], joint_msr_velocity_prev_[j], 0.8);
    }

    joint_msr_stiffness_[j] = joint_cmd_stiffness_[j];
    joint_msr_damping_[j] = joint_cmd_damping_[j];
  }
  fri_last_timestamp = device_->getTimestamp();
  return;
}

void LWRHWFRI::write(ros::Time time, ros::Duration period)
{
  enforceLimits(period);

  float newJntPosition[n_joints_];
  float newJntStiff[n_joints_];
  float newJntDamp[n_joints_];
  float newJntAddTorque[n_joints_];

  switch (getControlStrategy())
  {
  case JOINT_POSITION:

    for (size_t j = 0; j < n_joints_; j++)
      newJntPosition[j] = joint_cmd_position_[j];
    device_->doPositionControl(newJntPosition, false);
    break;

  case CARTESIAN_IMPEDANCE:
    /// TODO
    break;

  case JOINT_IMPEDANCE:
    for(size_t j=0; j < n_joints_; j++) {
      newJntPosition[j] = joint_cmd_position_[j];
      newJntAddTorque[j] = joint_cmd_effort_[j];
      newJntStiff[j] = joint_cmd_stiffness_[j];
      newJntDamp[j] = joint_cmd_damping_[j];
    }
    device_->doJntImpedanceControl(newJntPosition, newJntStiff, newJntDamp, newJntAddTorque, false);
    break;

  case JOINT_EFFORT:
    // mirror the position
    device_->getMsrMsrJntPosition(newJntPosition);
    for(size_t j=0; j < n_joints_; j++){
      newJntAddTorque[j] = joint_cmd_effort_[j];
      newJntStiff[j] = 0.0;
      newJntDamp [j]= 0.0;
    }
    device_->doJntImpedanceControl(newJntPosition, newJntStiff, newJntDamp, newJntAddTorque, false);

    break;
  }
  return;
}

void LWRHWFRI::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
{
  // at this point, we now that there is only one controller that ones to command joints
  ControlStrategy desired_strategy = JOINT_POSITION; // default

  // If any of the controllers in the start list works on a velocity interface, the switch can't be done.
  for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
  {
    if( it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
    {
      std::cout << "Request to switch to hardware_interface::PositionJointInterface (JOINT_POSITION)" << std::endl;
      desired_strategy = JOINT_POSITION;
      break;
    }
    else if( it->hardware_interface.compare( std::string("hardware_interface::EffortJointInterface") ) == 0 )
    {
      std::cout << "Request to switch to hardware_interface::EffortJointInterface (PREDEFINED JOINT_IMPEDANCE)" << std::endl;
      desired_strategy = JOINT_EFFORT;
      break;
    }
    else if( it->hardware_interface.compare( std::string("hardware_interface::ImpedanceJointInterface") ) == 0 )
    {
      std::cout << "Request to switch to hardware_interface::ImpedanceJointInterface (JOINT_IMPEDANCE)" << std::endl;
      desired_strategy = JOINT_IMPEDANCE;
      break;
    }
  }


  float joint_msr_position[n_joints_];
  device_->getMsrMsrJntPosition(joint_msr_position);
  for (size_t j = 0; j < n_joints_; ++j){

    ///semantic Zero
    joint_cmd_position_[j] = joint_msr_position[j];
    joint_cmd_effort_[j] = 0.0; // ToDo try to findproper initial values

    ///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
    try{  position_interface_.getHandle(joint_names_[j]).setCommand(joint_cmd_position_[j]);  }
    catch(const hardware_interface::HardwareInterfaceException&){}
    try{  effort_interface_.getHandle(joint_names_[j]).setCommand(joint_cmd_effort_[j]);  }
    catch(const hardware_interface::HardwareInterfaceException&){}
    try{  impedance_interface_.getHandle(joint_names_[j]).setPositionCommand(joint_cmd_position_[j]);  }
    catch(const hardware_interface::HardwareInterfaceException&){}
    try{  impedance_interface_.getHandle(joint_names_[j]).setStiffnessCommand(joint_cmd_stiffness_[j]);  }
    catch(const hardware_interface::HardwareInterfaceException&){}
    try{  impedance_interface_.getHandle(joint_names_[j]).setDampingCommand(joint_cmd_damping_[j]);  }
    catch(const hardware_interface::HardwareInterfaceException&){}
    try{  impedance_interface_.getHandle(joint_names_[j]).setEffortCommand(joint_cmd_effort_[j]);  }
    catch(const hardware_interface::HardwareInterfaceException&){}

    ///reset joint_limit_interfaces
    pj_sat_interface_.reset();
    pj_limits_interface_.reset();
  }

  if(desired_strategy == getControlStrategy())
    std::cout << "The ControlStrategy didn't changed, it is already: " << getControlStrategy() << std::endl;
  else
  {
    stopFRI();

    // send to KRL the new strategy
    if( desired_strategy == JOINT_POSITION )
      device_->setToKRLInt(0, JOINT_POSITION);
    else if( desired_strategy == JOINT_EFFORT)
      device_->setToKRLInt(0, JOINT_IMPEDANCE);
    else if( desired_strategy == JOINT_IMPEDANCE)
      device_->setToKRLInt(0, JOINT_IMPEDANCE);

    startFRI();
    setControlStrategy(desired_strategy);
    std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
  }
}

void LWRHWFRI::KRCCommThreadCallback()
{
  sched_param sch;
  int policy;
  pthread_getschedparam(pthread_self(), &policy, &sch);
  sch.sched_priority = 95;
  if (pthread_setschedparam(KRCCommThread_.get()->native_handle(), SCHED_FIFO, &sch))
    std::cout << "Failed to setschedparam: " << std::strerror(errno) << " for the communication thread\n";

  std::thread::id thread_id = KRCCommThread_.get()->get_id();
  pthread_getschedparam(pthread_self(), &policy, &sch);
  iomutex.lock();
  std::cout << "Communication thread " << thread_id << " is executing at priority: "<< sch.sched_priority << '\n';
  iomutex.unlock();
  while(!stopKRCComm_)
  {
    device_->doReceiveData();
    if (controlStarted_)
      device_->doSendData();
    send_comm_data_.data[0]= (ros::Time::now().toSec() - last_comm_cycle_.toSec());
    pub_comm_data_.publish(send_comm_data_);
    last_comm_cycle_ = ros::Time::now();

  }
  return;
}

void LWRHWFRI::startFRI()
{
  // wait until FRI enters in command mode
  // std::cout << "Waiting for good communication quality..." << std::endl;
  // while( device_->getQuality() != FRI_QUALITY_OK ){};
  device_->setToKRLInt(1, 1);
  // std::cout << "Waiting for command mode..." << std::endl;
  // while ( device_->getFrmKRLInt(1) != 1 )
  // {
  // std::cout << "device_->getState(): " << device_->getState() << std::endl;
  // device_->setToKRLInt(1, 1);
  // usleep(1000000);
  // }
  return;
}

void LWRHWFRI::stopFRI()
{
  // wait until FRI enters in command mode
  device_->setToKRLInt(1, 0);
  std::cout << "Waiting for monitor mode..." << std::endl;
  bool stopped = false;
  while ( !stopped )
  {
    stopped = device_->getFrmKRLInt(1) == 0;
    usleep(100000);
  }
  // {
  // std::cout << "device_->getState(): " << device_->getState() << std::endl;
  // std::cout << "Waiting for monitor mode..." << std::endl;
  // device_->setToKRLInt(1, 0);
  // usleep(1000000);
  // }
  return;
}

bool LWRHWFRI::KRCCommStart(){

  last_comm_cycle_ = ros::Time::now();
  pub_comm_data_= nh_.advertise<std_msgs::Float64MultiArray>("/lwr/lwr_hw_fri/comm_data", 10);
  KRCCommThread_.reset( new std::thread( &LWRHWFRI::KRCCommThreadCallback,this ));
  std::this_thread::sleep_for(std::chrono::seconds(1));
  startFRI();

  std::cout <<  "Ready, FRI has been started!" << std::endl;
  std::cout <<  "FRI Status:\n" <<
                "Timestamp: "<<device_->getTimestamp() << "\n" <<
                "State: "<<device_->getState() << "\n" <<
                "Quality: "<<device_->getQuality() << "\n" <<
                "desiredMsrSampleTime: "<<device_->getMsrBuf().intf.desiredMsrSampleTime << "\n" <<
                "desiredCmdSampleTime: "<<device_->getMsrBuf().intf.desiredCmdSampleTime << "\n" <<
                "safetyLimits: "<<device_->getMsrBuf().intf.safetyLimits << "\n" <<
                "latency: "<<device_->getMsrBuf().intf.stat.latency << "\n" <<
                "jitter: "<<device_->getMsrBuf().intf.stat.jitter << "\n" <<
                "missRate: "<<device_->getMsrBuf().intf.stat.missRate << "\n" <<
                "missCounter: "<<device_->getMsrBuf().intf.stat.missCounter << "\n";
  return true;
}

void LWRHWFRI::setControlReady(){
  controlStarted_ = true;
}
} // end namespace lwr_hw
