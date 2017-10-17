#ifndef LWR_CONTROLLERS__ADAPTIVE_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS__ADAPTIVE_TORQUE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>



namespace lwr_controllers
{
	class AdaptiveTorqueController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:

		AdaptiveTorqueController();
		~AdaptiveTorqueController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);

	private:

		ros::Subscriber sub_posture_;
		ros::Subscriber sub_gains_;
        ros::Publisher  pub_all_data_;
        std_msgs::Float64MultiArray all_data_msg;
        
        KDL::JntArray cmd_states_, init_states_;
		int cmd_flag_;	// discriminate if a user command arrived
		double lambda;	// flattening coefficient of tanh
		int step_;		// step used in tanh for reaching gradually the desired posture

        bool enb_record_all_data_, enb_adapting_;
        double timer_temp_, timer02ms_, timer10ms_;
        double spline_time_;

        KDL::JntArrayAcc joint_ref_states_;
        KDL::JntArray  Sq_, delta_, delta_prev_, delta_int_, Yr_theta_;
        Eigen::MatrixXd Yr_;
        Eigen::VectorXd theta_, theta_dot_, theta_dot_prev_; // regressor only for gravity term

		KDL::JntArray tau_cmd_;
        KDL::JntArray Kp_, Kd_, Ki_;	// gains

        void recordAllData(const ros::Time &time, const ros::Duration &period);
        void calcYr();
        void calcTheta(const ros::Duration &period, double inv_Gama);

	};
}

#endif
