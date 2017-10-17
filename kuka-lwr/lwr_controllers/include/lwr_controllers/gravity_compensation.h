#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace lwr_controllers
{
    class GravityCompensation: public controller_interface::KinematicChainControllerBase<hardware_interface::ImpedanceJointInterface>
    {
    public:
        
        GravityCompensation();
        ~GravityCompensation();
        
        bool init(hardware_interface::ImpedanceJointInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);
        
    private:
        KDL::JntArray  Yr_theta_;
        Eigen::MatrixXd Yr_;
        Eigen::VectorXd theta_,des_stiffness_ ;
        ros::Publisher  pub_all_data_;
        std_msgs::Float64MultiArray all_data_msg;
        bool enb_record_all_data_;
        double timer02ms_, timer10ms_;

        void calcYr();
        void recordAllData(const ros::Time &time, const ros::Duration &period);

    };
}

#endif
