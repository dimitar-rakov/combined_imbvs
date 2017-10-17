#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <lwr_controllers/computed_torque_controller.h>

namespace lwr_controllers 
{
	ComputedTorqueController::ComputedTorqueController() {}
	ComputedTorqueController::~ComputedTorqueController() {}

	bool ComputedTorqueController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

        nh_.getParam("/lwr/computed_torque_controller/enb_record_all_data" ,enbRecordAllData_);
        if (enbRecordAllData_) {ROS_INFO("The data recording for ComputedTorqueController controller is enabled");}
        else{enbRecordAllData_= false;}
        
		id_solver_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

		cmd_states_.resize(kdl_chain_.getNrOfJoints());
        init_states_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kv_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());

		sub_posture_ = nh_.subscribe("command", 1, &ComputedTorqueController::command, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &ComputedTorqueController::set_gains, this);
        pub_all_data_= nh_.advertise<std_msgs::Float64MultiArray>("/lwr/computed_torque_controller/all_data", 10);
        //ROS_INFO("computed_torque_controller is initilized.");

		return true;		
	}

	void ComputedTorqueController::starting(const ros::Time& time)
	{

        // computing Gravity matrices
        id_solver_->JntToGravity(joint_msr_states_.q, G_);
        // get joint positions
        std::cout << "Initial joints position is: ";
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
            // for sim robot PD
            Kp_(i) = 100.0;
            Kv_(i) = 0.7;
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    		joint_des_states_.q(i) = joint_msr_states_.q(i);

            tau_cmd_(i) = G_(i);
            joint_handles_[i].setCommand(tau_cmd_(i));
            cmd_states_(i) = joint_msr_states_.q(i);
            init_states_(i)= joint_msr_states_.q(i);
            std::cout << init_states_(i)<<"  ";
    	}
        std::cout <<std::endl;
        spline_time_= 25.0; //spline time  in sec for the full range error (-pi, pi)
        lambda = 2*M_PI/spline_time_;	// lower values: flatter.
    	cmd_flag_ = 0;	
        timer_temp_= 0.0;

    	ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );

    }

    void ComputedTorqueController::update(const ros::Time& time, const ros::Duration& period)
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
            }


            if(joint_msr_states_.q == cmd_states_)
    		{
    			cmd_flag_ = 0;	//reset command flag
                timer_temp_=0.0;
    			ROS_INFO("Posture OK");
    		}
    	}

    	// computing Inertia, Coriolis and Gravity matrices
        id_solver_->JntToMass(joint_msr_states_.q, M_);
        id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
        id_solver_->JntToGravity(joint_msr_states_.q, G_);

		for(size_t i=0; i<joint_handles_.size(); i++) 
		{
			// control law
            tau_cmd_(i) = M_(i,i)*(joint_des_states_.qdotdot(i) + Kv_(i)*(joint_des_states_.qdot(i) - joint_msr_states_.qdot(i)) + Kp_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i))) + C_(i)*joint_msr_states_.qdot(i) + G_(i);

            // SENTINEL
            if (tau_cmd_(i) < -210.0/((float)i+1)){
                ROS_WARN ("Torque lower bound joint %lu . The desired value was %.4lf", i, tau_cmd_(i) );
                tau_cmd_(i) = -210.0/((float)i+1);

            }
            if (tau_cmd_(i) > 210.0/((float)i+1)) {
                ROS_WARN ("Torque upper bound joint %lu . The desired value was %.4lf", i, tau_cmd_(i) );
                tau_cmd_(i) = 210.0/((float)i+1);
            }
            joint_handles_[i].setCommand(tau_cmd_(i));

        }

        timer02ms_ += period.toSec();
        timer10ms_ += period.toSec();

        // pulse timer at 2ms
        if (timer02ms_>= 0.002){
            //record and publish all data at every 2 ms interval
            if (enbRecordAllData_){
                recordAllData(time, period);
                pub_all_data_.publish (allDataMsg);
            }
            timer02ms_= 0.0;
        }

        // pulse timer at 10ms used for the hyperbolic tangent function
        if (timer10ms_>= 0.010){
            timer_temp_+=0.010;
            timer10ms_= 0.0;
        }

    }

    void ComputedTorqueController::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
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

	void ComputedTorqueController::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
        if (msg->data.size() == 2*joint_handles_.size()){
            for(unsigned int i = 0; i < joint_handles_.size(); i++)
            {
                Kp_(i) = msg->data[i];
                Kv_(i) = msg->data[i + joint_handles_.size()];
            }

        }
		else
			ROS_INFO("Number of Joint handles = %lu", joint_handles_.size());

		ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

		ROS_INFO("New gains Kp: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5), Kp_(6));
		ROS_INFO("New gains Kv: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kv_(0), Kv_(1), Kv_(2), Kv_(3), Kv_(4), Kv_(5), Kv_(6));

	}

    void ComputedTorqueController::recordAllData(const ros::Time &time, const ros::Duration &period){
        // All data for rosbag
        allDataMsg.data.clear();
        //measured angles 3-9
        for (int i = 0; i < joint_handles_.size(); i++)
            allDataMsg.data.push_back(joint_msr_states_.q(i));

        //desired angles 10-16
        for (int i = 0; i < joint_handles_.size(); i++)
            allDataMsg.data.push_back(joint_des_states_.q(i));

        //measured velocity 17-23
        for (int i = 0; i < joint_handles_.size(); i++)
            allDataMsg.data.push_back(joint_msr_states_.qdot(i));

        //desired velocity 24-30
        for (int i = 0; i < joint_handles_.size(); i++)
            allDataMsg.data.push_back(joint_des_states_.qdot(i));

        // measured torques 31-37
        for (int i = 0; i < joint_handles_.size(); i++)
            allDataMsg.data.push_back(joint_handles_[i].getEffort());        

        // desired torques 38-44
        for (int i = 0; i < joint_handles_.size(); i++)
            allDataMsg.data.push_back(tau_cmd_(i));

        // G 45-51
        for (int i = 0; i < joint_handles_.size(); i++)
            allDataMsg.data.push_back(G_(i));


   }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::ComputedTorqueController, controller_interface::ControllerBase)
