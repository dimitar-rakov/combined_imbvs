#ifndef LWR_HW_FRI_H
#define LWR_HW_FRI_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRI remote hooks
#include <limits.h>
#include <thread>
#include "fri/friudp.h"
#include "fri/friremote.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <pthread.h>
#include <mutex>

// ToDo: add timeouts to all sync-while's to KRL since the UDP connection might be lost and we will know

namespace lwr_hw
{

class LWRHWFRI : public LWRHW
{

public:

  LWRHWFRI();
  ~LWRHWFRI();

  void setPort(int port);

  void setIP(std::string hintToRemoteHost);

  // Init, read, and write, with FRI hooks
  bool init(ros::NodeHandle &nh);

  void read(ros::Time time, ros::Duration period);

  void write(ros::Time time, ros::Duration period);

  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);

  bool KRCCommStart();

  void setControlReady();

private:

  ros::NodeHandle nh_;

  ros::Publisher  pub_comm_data_;
  std_msgs::Float64MultiArray send_comm_data_;
  ros::Time last_comm_cycle_;


  // Parameters
  int port_;
  bool port_set_;
  bool ip_set_;

  bool stopKRCComm_ ;
  bool controlStarted_;
  bool communicationIsAlive;
  std::string hintToRemoteHost_;
  float fri_last_timestamp;

  // low-level interface
  boost::shared_ptr<friRemote> device_;

  // FRI values
  FRI_QUALITY lastQuality_;
  FRI_CTRL lastCtrlScheme_;

  boost::shared_ptr<std::thread> KRCCommThread_;
  std::mutex iomutex;

  void KRCCommThreadCallback();

  void startFRI();

  void stopFRI();

};

}
#endif // LWR_HW_FRI_H
