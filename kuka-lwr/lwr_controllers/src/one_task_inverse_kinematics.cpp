
#include <lwr_controllers/one_task_inverse_kinematics.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>


#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <math.h>
#include <angles/angles.h>


namespace lwr_controllers 
{
    OneTaskInverseKinematics::OneTaskInverseKinematics() {}
    OneTaskInverseKinematics::~OneTaskInverseKinematics() {}

    bool OneTaskInverseKinematics::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) ) {
            ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
            return false;
        }

        nh_.getParam("/lwr/one_task_inverse_kinematics/enb_record_all_data" ,enb_record_all_data_);
        if (enb_record_all_data_) {ROS_INFO("The data recording for OneTaskInverseKinematics controller is enabled");}
        else{enb_record_all_data_= false;}

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

        Jbe_.resize(kdl_chain_.getNrOfJoints());      
        Jbt_.resize(joint_handles_.size());
        Jwt_.resize(joint_handles_.size());
        cart_traj_.resize(6);

        // get joint positions
        for(size_t i=0; i < joint_handles_.size(); i++){
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
            joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);

            // states used by trapezoidal integration
            joint_msr_states_prev_.qdot(i) = joint_msr_states_.qdot(i);
            joint_des_states_prev_.qdot(i) = joint_des_states_.qdot(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbe_);


        // Robot default frame systems
        bases_names_.push_back("robot_base");
        tools_names_.push_back("flange");
        Fwb_ = KDL::Frame::Identity();
        frames_w_b_.push_back(Fwb_);
        Fet_ = KDL::Frame::Identity();
        frames_e_t_.push_back(Fet_);
        Fwt_ = Fwb_* Fbe_ * Fet_ ;


        //Get all bases from yaml
        XmlRpc::XmlRpcValue bases;
        std::string bases_param_name = std::string ("bases");
        if ( !nh_.getParam(nh_.getNamespace()+"/"+bases_param_name, bases ) || bases.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            ROS_WARN("%s is not properly set in yaml file",bases_param_name.c_str());
        else
            getFramesFromParamServer(bases, bases_names_, frames_w_b_);


         //Get all tools from yaml
        XmlRpc::XmlRpcValue tools;
        std::string tools_param_name = std::string ("tools");
        if ( !nh_.getParam(nh_.getNamespace()+"/"+tools_param_name, tools ) || tools.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            ROS_WARN("%s is not properly set in yaml file",tools_param_name.c_str());
        else
            getFramesFromParamServer(tools, tools_names_, frames_e_t_);

        //Desired posture is the current one base: robot_base, tool: flange
        F_des_wt_ = Fwt_;
        cmd_flag_ = 0;
        sub_command_ = nh_.subscribe("command", 1, &OneTaskInverseKinematics::command, this);
        pub_all_data_= nh_.advertise<std_msgs::Float64MultiArray>("all_data", 1);
        pub_on_target_ = nh_.advertise<std_msgs::Bool>("on_target", 1);
        ROS_INFO("one_task_inverse_kinematics is initilized.");
        return true;

    }

    void OneTaskInverseKinematics::starting(const ros::Time& time)
    {
        // get joint positions
        for(size_t i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
            joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);

            // states used by trapezoidal integration
            joint_msr_states_prev_.qdot(i) = joint_msr_states_.qdot(i);
            joint_des_states_prev_.qdot(i) = joint_des_states_.qdot(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbe_);
        Fwt_ = Fwb_* Fbe_ * Fet_ ;

        //Desired posture is the current one base: robot_base, tool: flange
        F_des_wt_ = Fwt_;

        record_data_interval_ = 0.0;
        double x, y, z, w;
        double roll, pitch, yaw;
        Fbe_.M.GetRPY(roll, pitch, yaw);
        Fbe_.M.GetQuaternion(x, y, z, w);
        std::cout << "Initial Position Fbe: " << Fbe_.p.x() <<"   "<<  Fbe_.p.y()<<"   "<< Fbe_.p.z()<<"\n";
        std::cout << "Initial Orientation Fbe Quaternion: " <<x <<"   "<< y<<"   "<<z<<"   "<<w<<"\n";
        std::cout << "Initial Orientation Fbe RPY: " <<roll <<"   "<< pitch<<"   "<<yaw<<"\n";

    }

    void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
    {
        // get joint positions and velocity
        for(size_t i=0; i < joint_handles_.size(); i++){
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        }
        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbe_);

        // computing Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jbe_);

        // Change references points to tool frame and reference frame to work_base
        KDL::changeRefPoint(Jbe_,Fet_.p, Jbt_);
        KDL::changeBase(Jbt_, Fwb_.M, Jwt_);
        pseudo_inverse(Jwt_.data, J_pinv_);
        Fwt_ = Fwb_* Fbe_ * Fet_ ;

        if (cmd_flag_)
        {


            /// Jacobian tool with respect to the base Jbt (Khatib lectures 4 Jacobian coresponding to KDL::changeRefPoint)
            ///     Eigen::MatrixXd Ste = Eigen::MatrixXd::Identity(6,6);
            ///     Eigen::Matrix<double,3,3> p_hat;
            ///     skew_symmetric (Fec_.p , p_hat);
            ///     Ste.topRightCorner(3,3) = p_hat;
            ///     Jbt_.data = Ste*Jbe_.data;


            // get current trajectory Pose
            double curr_time = (ros::Time::now()- traj_start_t).toSec();
            KDL::Vector t = KDL::Vector (cart_traj_[0].getPos(curr_time), cart_traj_[1].getPos(curr_time), cart_traj_[2].getPos(curr_time));
            KDL::Rotation R = KDL::Rotation::RPY(cart_traj_[3].getPos(curr_time), cart_traj_[4].getPos(curr_time), cart_traj_[5].getPos(curr_time));
            F_des_wt_traj_  = KDL::Frame (R, t);

            // end-effector pose error twist
            x_err_ = diff(Fwt_, F_des_wt_traj_);
            // x_err_ = diff(Fwt_, F_des_wt_);

            // computing q_dot
            for (size_t i = 0; i < J_pinv_.rows(); i++){
                joint_des_states_prev_.q(i) = joint_des_states_.q(i);
                joint_des_states_prev_.qdot(i) = joint_des_states_.qdot(i);
                joint_des_states_.qdot(i) = 0.0;
                for (size_t k = 0; k < J_pinv_.cols(); k++){
                    joint_des_states_.qdot(i) += J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
                }
            }

            // integrating q_dot -> getting q (Trapezoidal method)
            for (size_t i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*0.5*(joint_des_states_prev_.qdot(i)+joint_des_states_.qdot(i));

            // joint limits saturation
            for (size_t i =0;  i < joint_handles_.size(); i++)
            {
                // Limit maximmum desired joint step send to robot and max joint position
                double err_dist = 0.0;
                angles::shortest_angular_distance_with_limits(joint_des_states_prev_.q(i), joint_des_states_.q(i), joint_limits_.min(i), joint_limits_.max(i), err_dist);
                if(err_dist > controller_interface::MAX_JOINT_DES_STEP)
                   joint_des_states_.q(i) = joint_des_states_prev_.q(i) + controller_interface::MAX_JOINT_DES_STEP;
                if(err_dist <-controller_interface::MAX_JOINT_DES_STEP)
                   joint_des_states_.q(i) = joint_des_states_prev_.q(i) - controller_interface::MAX_JOINT_DES_STEP;

                if (joint_des_states_.q(i) < joint_limits_.min(i) + 0.01){
                    joint_des_states_.q(i) = joint_limits_.min(i) + 0.01;
                    ROS_WARN_DELAYED_THROTTLE(5,"Activated min position saturation for joint %zu", i);
                }
                if (joint_des_states_.q(i) > joint_limits_.max(i) - 0.01){
                    joint_des_states_.q(i) = joint_limits_.max(i) - 0.01;
                    ROS_WARN_DELAYED_THROTTLE(5,"Activated max position saturation for joint %zu", i);
                }
            }

            std_msgs::Bool on_target_msg;
            if (Equal(Fwt_, F_des_wt_, 0.0001))
            {
                ROS_INFO("On target");
                on_target_msg.data = true;
                cmd_flag_ = 0;
            }
            else
                on_target_msg.data = false;
            pub_on_target_.publish(on_target_msg);

            // set controls for joints
            for (size_t i = 0; i < joint_handles_.size(); i++)
              joint_handles_[i].setCommand(joint_des_states_.q(i));
        }

        //record and publish all data at every 2 ms interval
        if (enb_record_all_data_){
            if (record_data_interval_>= 0.002){
                recordAllData();
                pub_all_data_.publish (all_data_msg);
                record_data_interval_= 0.0;
            }
            else { record_data_interval_ += period.toSec();}
         }


    }

    void OneTaskInverseKinematics::command(const lwr_controllers::PoseWithBaseAndTool::ConstPtr &msg)
    {
        int base_name_idx = -1;
        int tool_name_idx = -1;
        for (size_t i = 0; i < bases_names_.size() ; i++)
            base_name_idx = (msg->base_name == bases_names_[i] )? i : base_name_idx;

        for (size_t i = 0; i < tools_names_.size() ; i++)
            tool_name_idx = (msg->tool_name == tools_names_[i] )? i : tool_name_idx;

        if (base_name_idx >-1 &&  tool_name_idx > -1){

            F_des_wt_.p = KDL::Vector(msg->position.x, msg->position.y, msg->position.z);
            F_des_wt_.M = KDL::Rotation::RPY(msg->orientation.roll, msg->orientation.pitch, msg->orientation.yaw);
            cmd_flag_ = 1;
            Fwb_ = frames_w_b_[base_name_idx];
            Fet_ = frames_e_t_[tool_name_idx];
            Fwt_ = Fwb_* Fbe_ * Fet_ ;

            traj_start_t =ros::Time::now();
            double tx = std::abs(F_des_wt_.p.x() - Fwt_.p.x())/controller_interface::MAX_V;
            double ty = std::abs(F_des_wt_.p.y() - Fwt_.p.y())/controller_interface::MAX_V;
            double tz = std::abs(F_des_wt_.p.z() - Fwt_.p.z())/controller_interface::MAX_V;

            double roll1, pitch1 , yaw1;
            double roll2, pitch2 , yaw2;


            Fwt_.M.GetRPY(roll1, pitch1 , yaw1 );
            F_des_wt_.M.GetRPY(roll2, pitch2 , yaw2 );

            double t_roll = std::abs(roll2 -roll1)/controller_interface::MAX_W;
            double t_pitch = std::abs(pitch2 - pitch1)/controller_interface::MAX_W;
            double t_yaw = std::abs(yaw2 -yaw1)/controller_interface::MAX_W;

            cart_traj_[0].calcSpline(Fwt_.p.x(), 0.0, F_des_wt_.p.x(), 0.0, tx);
            cart_traj_[1].calcSpline(Fwt_.p.y(), 0.0, F_des_wt_.p.y(), 0.0, ty);
            cart_traj_[2].calcSpline(Fwt_.p.z(), 0.0, F_des_wt_.p.z(), 0.0, tz);
            cart_traj_[3].calcSpline(roll1, 0.0, roll2, 0.0, t_roll);
            cart_traj_[4].calcSpline(pitch1, 0.0, pitch2, 0.0, t_pitch);
            cart_traj_[5].calcSpline(yaw1, 0.0, yaw2, 0.0, t_yaw);


        }
        else if (base_name_idx == -1)
            ROS_WARN ("base name is not among existed bases_names from parameter server");
        else if (tool_name_idx == -1)
            ROS_WARN ("tool_name is not among existed tools_names from parameter server");
    }

    void OneTaskInverseKinematics::recordAllData(){
           // All data for rosbag
           all_data_msg.data.clear();
           //measured angles 0-6
           for (int i = 0; i < joint_handles_.size(); i++){
               all_data_msg.data.push_back(joint_msr_states_.q(i));
           }
           //desired angles 7-13
           for (int i = 0; i < joint_handles_.size(); i++){
               all_data_msg.data.push_back(joint_des_states_.q(i));
           }
           //measured velocity 14-20
           for (int i = 0; i < joint_handles_.size(); i++){
               all_data_msg.data.push_back(joint_msr_states_.qdot(i));
           }

           //desired velocity 21-27
           for (int i = 0; i < joint_handles_.size(); i++){
               all_data_msg.data.push_back(joint_des_states_.qdot(i));
           }

           // measured frame 28-33
           double roll, pitch, yaw;

           all_data_msg.data.push_back(Fwt_.p.x());
           all_data_msg.data.push_back(Fwt_.p.y());
           all_data_msg.data.push_back(Fwt_.p.z());
           Fwt_.M.GetRPY(roll, pitch, yaw);
           all_data_msg.data.push_back(roll);
           all_data_msg.data.push_back(pitch);
           all_data_msg.data.push_back(yaw);

           //desired  frame 34-39
           all_data_msg.data.push_back(F_des_wt_.p.x());
           all_data_msg.data.push_back(F_des_wt_.p.y());
           all_data_msg.data.push_back(F_des_wt_.p.z());
           F_des_wt_.M.GetRPY(roll, pitch, yaw);
           all_data_msg.data.push_back(roll);
           all_data_msg.data.push_back(pitch);
           all_data_msg.data.push_back(yaw);

           //desired  frame  trjectory 40-45
           all_data_msg.data.push_back(F_des_wt_traj_.p.x());
           all_data_msg.data.push_back(F_des_wt_traj_.p.y());
           all_data_msg.data.push_back(F_des_wt_traj_.p.z());
           F_des_wt_traj_.M.GetRPY(roll, pitch, yaw);
           all_data_msg.data.push_back(roll);
           all_data_msg.data.push_back(pitch);
           all_data_msg.data.push_back(yaw);

   }

    void OneTaskInverseKinematics::getFramesFromParamServer(XmlRpc::XmlRpcValue &obj, std::vector<std::string> &name, std::vector<KDL::Frame> &frames){

        for (XmlRpc::XmlRpcValue::ValueStruct::iterator map_it=obj.begin(); map_it!=obj.end(); ++map_it) {
            if ( map_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) // map iterator
                ROS_WARN("[%s] is not properly set in yaml file", map_it->first.c_str());
            else{
                // Check fields
                if ( !map_it->second.hasMember(std::string("position")))
                    ROS_WARN("[%s] is not proceed, since key [position] is not found in yaml file.", map_it->first.c_str());
                else if ( !map_it->second.hasMember(std::string("rpy")))
                    ROS_WARN("[%s] is not proceed, since key [rpy] is not found in yaml file.", map_it->first.c_str());
                else{
                    if ( map_it->second[std::string("position")].getType() != XmlRpc::XmlRpcValue::TypeArray)
                        ROS_WARN("[%s] is not proceed, since key [position] is incorrect data type.", map_it->first.c_str());
                    else if ( map_it->second[std::string("rpy")].getType() != XmlRpc::XmlRpcValue::TypeArray)
                        ROS_WARN("[%s] is not proceed, since key [rpy] is incorrect data type.", map_it->first.c_str());
                    else {
                        XmlRpc::XmlRpcValue pos = map_it->second[std::string("position")];
                        XmlRpc::XmlRpcValue rpy = map_it->second[std::string("rpy")];
                        if (pos.size() !=3 && pos.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                            ROS_WARN("[%s] is not proceed, since key [position] has incorrect size or data.", map_it->first.c_str());
                        else if (rpy.size() !=3 && rpy.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                            ROS_WARN("[%s] is not proceed, since key [rpy] has incorrect size or data.", map_it->first.c_str());
                        else{
                            name.push_back((std::string)(map_it->first));
                            frames.push_back(KDL::Frame(KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]), KDL::Vector(pos[0], pos[1], pos[2])));
                        }
                    }
                }
            }
        }
    }




}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::OneTaskInverseKinematics, controller_interface::ControllerBase)
