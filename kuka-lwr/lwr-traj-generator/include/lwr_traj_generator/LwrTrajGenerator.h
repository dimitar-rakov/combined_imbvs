#ifndef LWR_TRAJ_GENERATOR_H
#define LWR_TRAJ_GENERATOR_H

//Standard Headers
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <pthread.h>
#include <eigen3/Eigen/Eigen>

//ROS headers
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <cv_bridge/cv_bridge.h>             //interface between ROS and OpenCV
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

class LwrTrajGenerator
{
public:
    LwrTrajGenerator();
    void init (ros::NodeHandle &nh);
    void update(const ros::Time& time, const ros::Duration& period);
    void command(const std_msgs::String &msg);


    // variables
    pthread_mutex_t count_mutex;


private:
    void startTrajectroies();
    void stopTrajectroies();
    void goToPosition(Eigen::VectorXd goal_pos);
    void getJointPosition(const sensor_msgs::JointState &msg);


    std::vector<Eigen::VectorXd> all_traj;
    Eigen::VectorXd joint_msr_pos_, joint_des_pos_, home_pos_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_command_, sub_joint_state_;
    ros::Publisher  pub_posture_;
    std_msgs::Float64MultiArray cmd_msg_;
    bool flag_work_;
    double stuck_time_;

};

#endif // LWR_TRAJ_GENERATOR_H
