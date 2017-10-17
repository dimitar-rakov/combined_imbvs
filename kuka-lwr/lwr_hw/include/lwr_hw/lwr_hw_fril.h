#ifndef LWR_HW_FRIL_H
#define LWR_HW_FRIL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRIL remote hooks
#include <FastResearchInterface.h>

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK   2000
#define EOK 0

namespace lwr_hw
{

class LWRHWFRIL : public LWRHW
{

public:

  LWRHWFRIL();
  ~LWRHWFRIL();

  void stop();
  void set_mode();
  void setInitFile(std::string init_file);

  // Init, read, and write, with FRI hooks
  bool init(ros::NodeHandle &nh);
  void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);
  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);

private:
  ros::NodeHandle nh_;
  // Parameters
  std::string init_file_;
  bool file_set_;// = false;

  // low-level interface
  boost::shared_ptr<FastResearchInterface> device_;
  int ResultValue;// = 0;
};

}


#endif // LWR_HW_FRIL_H
