#include <pluginlib/class_list_macros.h>
#include <math.h>

#include <lwr_controllers/gravity_compensation.h>

namespace lwr_controllers 
{
    GravityCompensation::GravityCompensation() {}
    GravityCompensation::~GravityCompensation() {}
    
    bool GravityCompensation::init(hardware_interface::ImpedanceJointInterface *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<hardware_interface::ImpedanceJointInterface>::init(robot, n);

        nh_.getParam("/gravity_compensation_controller/enb_record_all_data" ,enb_record_all_data_);
        if (enb_record_all_data_) {ROS_INFO("The data recording for GravityCompensation controller is enabled");}
        else{enb_record_all_data_= false;}

        Yr_ = Eigen::MatrixXd::Zero(kdl_chain_.getNrOfJoints(),7) ;
        theta_= Eigen::VectorXd::Zero(7);
        des_stiffness_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints()) ;

        pub_all_data_= nh_.advertise<std_msgs::Float64MultiArray>("/lwr/gravity_compensation_controller/all_data", 10);

        theta_<< 0.026451,   0.079353,   0.105804,   0.105804,   0.057207,   0.057207,   0.062513;
        des_stiffness_<< 1000.0,   500.0,   100.0,   50.0,   20.0,  0.0,   0.0;

        XmlRpc::XmlRpcValue gains;

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

        // stiffness from yaml
        if ( !nh_.getParam("stiffness", gains ) || gains.getType() != XmlRpc::XmlRpcValue::TypeArray){
            ROS_WARN("stiffness is not properly set in yaml file");
            std::cout <<"Using: "<<des_stiffness_.transpose()<<"\n";
        }
        else{
            for (int i = 0; i < des_stiffness_.size(); ++i){
                if ( gains[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
                    ROS_WARN("stiffness(%d) is not properly set in yaml file, Using %f", i, des_stiffness_(i));
                else
                    des_stiffness_(i) = static_cast <double>(gains[i]);
            }
            std::cout <<"stiffness: "<<des_stiffness_.transpose()<<"\n";
        }

        ROS_INFO("gravity_compensation_controller is initilized.");
        return true;
    }
    
    void GravityCompensation::starting(const ros::Time& time)
    {

        // get joint positions
        for(size_t i=0; i<joint_handles_.size(); i++)
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();

        calcYr();
        Yr_theta_.data = Yr_*theta_;
        for(size_t i=0; i<joint_handles_.size(); i++){
            joint_handles_[i].setPositionCommand(joint_handles_[i].getPosition());
            joint_handles_[i].setStiffnessCommand(des_stiffness_(i));
            joint_handles_[i].setDampingCommand(0.0);
            joint_handles_[i].setEffortCommand(Yr_theta_(i));
        }
        timer02ms_ = 0.0;
        timer10ms_ = 0.0;

    }
    
    void GravityCompensation::update(const ros::Time& time, const ros::Duration& period)
    {

        // get joint positions
        for(size_t i=0; i<joint_handles_.size(); i++)
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        calcYr();
        Yr_theta_.data = Yr_*theta_;
        for(size_t i=0; i<joint_handles_.size(); i++){
            joint_handles_[i].setPositionCommand(joint_msr_states_.q(i));
            joint_handles_[i].setStiffnessCommand(0.1);
            joint_handles_[i].setDampingCommand(0.0);
            joint_handles_[i].setEffortCommand(Yr_theta_(i));
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
            timer10ms_= 0.0;
        }
    }
    
    void GravityCompensation::stopping(const ros::Time& time)
    {
    }

    void GravityCompensation::calcYr(){

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

    void GravityCompensation::recordAllData(const ros::Time &time, const ros::Duration &period){
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
            all_data_msg.data.push_back(Yr_theta_(i));

        // Theta   45-51
        for (int i = 0; i < theta_.size(); i++)
            all_data_msg.data.push_back(theta_(i));
   }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::GravityCompensation, controller_interface::ControllerBase)
