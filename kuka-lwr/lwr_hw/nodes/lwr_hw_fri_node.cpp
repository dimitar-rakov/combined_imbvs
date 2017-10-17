// SYS
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Bool.h>

// the lwr hw fri interface
#include "lwr_hw/lwr_hw_fri.h"


bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

bool isStopPressed = false;
bool wasStopHandled = true;
void eStopCB(const std_msgs::BoolConstPtr& e_stop_msg)
{
  isStopPressed = e_stop_msg->data;
}

// Get the URDF XML from the parameter server
std::string getURDF(ros::NodeHandle &model_nh_, std::string param_name)
{
  std::string urdf_string;
  std::string robot_description = "/robot_description";

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("LWRHWFRI", "LWRHWFRI node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("LWRHWFRI", "LWRHWFRI node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("LWRHWFRI", "Received URDF from param server, parsing...");

  return urdf_string;
}

int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "lwr_hw_interface", ros::init_options::NoSigintHandler);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // create a node
  ros::NodeHandle lwr_nh;

  // get params or give default values
  int port;
  std::string hintToRemoteHost;
  std::string name;
  lwr_nh.param("port", port, 49939);
  lwr_nh.param("ip", hintToRemoteHost, std::string("192.168.0.40") );
  lwr_nh.param("name", name, std::string("lwr"));

  // advertise the e-stop topic
  ros::Subscriber estop_sub = lwr_nh.subscribe(lwr_nh.resolveName("emergency_stop"), 1, eStopCB);

  // get the general robot description, the lwr class will take care of parsing what's useful to itself
  std::string urdf_string = getURDF(lwr_nh, "/robot_description");

  // construct and start the real lwr
  lwr_hw::LWRHWFRI lwr_robot;
  lwr_robot.create(name, urdf_string);
  lwr_robot.setPort(port);
  lwr_robot.setIP(hintToRemoteHost);


  if(!lwr_robot.init(lwr_nh))
  {
    ROS_FATAL_NAMED("lwr_hw_fri","Could not initialize robot real interface");
    return -1;
  }

  if(!lwr_robot.KRCCommStart())
  {
    ROS_FATAL_NAMED("lwr_hw_fri","Could not start KRC communication");
    return -1;
  }


  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(0.002),des_period(0.002);


  //the controller manager
  controller_manager::ControllerManager manager(&lwr_robot, lwr_nh);
  lwr_robot.setControlReady();

  last = ros::Time::now();
  // run as fast as possible
  while( !g_quit )
  {
    // get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts))
    {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
    }
    else
    {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    }

    if(lwr_robot.getControlStrategy() == lwr_hw::LWRHW::ControlStrategy::JOINT_POSITION){
        //default value for position control algorithms default 0.0001
        //smaller as possible in order to have good interpolation
        des_period.fromSec(0.0001);
    }
    else {
        //default value for torque  based control algorithms default 0.002
        des_period.fromSec(0.002);
    }


    if (period.toSec()>=des_period.toSec()){

        //  checks if the controll loop takes too much time (just for information)
        if (period.toSec()>= 2*des_period.toSec())
        {
            // ROS_WARN("Last control period: %f",period.toSec());
        }

        // read the state from the lwr
        lwr_robot.read(now, period);

        // Compute the controller commands
        bool resetControllers;
        if(!wasStopHandled && !resetControllers)
        {
            ROS_WARN("E-STOP HAS BEEN PRESSED: Controllers will be restarted, but the robot won't move until you release the E-Stop");
            ROS_WARN("HOW TO RELEASE E-STOP: rostopic pub -r 10 /NAMESPACE/emergency_stop std_msgs/Bool 'data: false'");
            resetControllers = true;
            wasStopHandled = true;
        }

        if( isStopPressed )
        {
            wasStopHandled = false;
        }
        else
        {
            resetControllers = false;
            wasStopHandled = true;
        }

        // update the controllers
        manager.update(now, period, resetControllers);

        // write the command to the lwr
        lwr_robot.write(now, period);
        last = now;
    }
  }



  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();

  //std::cerr<<"Stopping LWR..."<<std::endl;
  //lwr_robot.stopFRI();

  std::cerr<<"Bye!"<<std::endl;

  return 0;
}
