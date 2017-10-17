#include "lwr_traj_generator/LwrTrajGenerator.h"


LwrTrajGenerator::LwrTrajGenerator(){}

void LwrTrajGenerator::init(ros::NodeHandle &nh){
    nh_=nh;
    cmd_msg_.data.resize(7);
    flag_work_=false;
    joint_msr_pos_ = Eigen::VectorXd::Zero(7);
    joint_des_pos_= Eigen::VectorXd::Zero(7);
    home_pos_=(Eigen::MatrixXd(7,1) << 0.00, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00).finished();

    all_traj.push_back((Eigen::MatrixXd(7,1) << 0.00, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << 0.00, 1.57, 0.00, 0.00, 0.00, 0.00, 0.00).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.50, 1.00, 0.00, 0.00, 0.00, 0.00, 00.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << 0.2, 1.0, 1.0, 0.5, 1.0, 0.5, 1.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << 0.3, 1.0, 1.0, 0.5, 1.0, 0.5, 1.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.25, 1.0, 1.0, 0.5, 1.0, 0.5, 1.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.0, 0.17, 1.0, 1.0, -1.0, -1.0, -1.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.3, 1.17, 1.0, 1.0, -1.0, -1.0, -0.05).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.2, -1.2, 1.0, 1.0, -0.8, -1.1, -0.8).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << 0.0, -1.2, 0.0, -1.0, -1.1, 0.0, -1.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.8, -0.5, -0.5, -1.0, -1.0, 0.0, -1.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.2, 0.17, 1.0, 1.0, -1.0, -1.0, -1.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.8, -0.5, -0.5, -1.0, -1.0, 0.0, -1.0).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -1.5, -0.0, 1.1, -0.6, 0.5, 0.6, -0.2).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.7, 1.0, 0.2, -0.6, 0.5, 0.6, -0.2).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << 0.0, -0.7, 0.3, 0.7, 0.2, 1.0, -1.2).finished());
    all_traj.push_back((Eigen::MatrixXd(7,1) << -0.2, -0.7, 0.5, -0.6, 0.2, 1.0, -1.2).finished());


    pub_posture_ =  nh_.advertise<std_msgs::Float64MultiArray>("/lwr/adaptive_torque_controller/command", 10);
    sub_command_= nh_.subscribe("/lwr/trajectory_generator_command", 1, &LwrTrajGenerator::command, this);
    sub_joint_state_= nh_.subscribe("/lwr/joint_states", 1, &LwrTrajGenerator::getJointPosition, this);

     ROS_INFO ("LwrTrajGenerator is initialized");


}

void LwrTrajGenerator::update(const ros::Time& time, const ros::Duration& period){


    if (flag_work_ && ((joint_des_pos_ - joint_msr_pos_).norm()<0.002 || stuck_time_>45.0)){
        int rand_traj_num = std::rand() % all_traj.size();
        joint_des_pos_= all_traj[rand_traj_num];
        for (int i = 0; i < joint_des_pos_.size(); i++){
            cmd_msg_.data[i] = joint_des_pos_(i);
        }
        pub_posture_.publish (cmd_msg_);
        std::cout<< "Measured last position: " <<joint_msr_pos_.transpose()<<"\n";
        std::cout<< "Desired new position: " <<joint_des_pos_.transpose()<<"\n";
        stuck_time_=0.0;
    }
    stuck_time_+=period.toSec();
}

void LwrTrajGenerator::command(const std_msgs::String &msg){
    std::string cmd = msg.data.c_str();
    if (cmd.compare("start") == 0) {
        ROS_INFO ("Starting...");
        startTrajectroies();
    }

    else if (cmd.compare("stop") == 0) {
        ROS_INFO ("Stoping...");
        stopTrajectroies();
    }

    else if (cmd.compare("home up") == 0) {
        ROS_INFO ("Homing to upper position...");
        goToPosition(home_pos_);
    }

    else if (cmd.compare("home down") == 0) {
        ROS_INFO ("Homing to lower position...");
        goToPosition((Eigen::MatrixXd(7,1) << 0.00, 1.57, 0.00, 0.00, 0.00, 0.00, 0.00).finished());
    }

    else{
        ROS_WARN("Wrong command! The choice is: start, stop, home");
    }



}

void LwrTrajGenerator::startTrajectroies(){
    joint_des_pos_= all_traj[std::rand() % all_traj.size()];
    for (int i = 0; i < joint_des_pos_.size(); i++){
        cmd_msg_.data[i] = joint_des_pos_(i);
    }
    pub_posture_.publish (cmd_msg_);
    std::cout<< "Desired initial position: " <<joint_des_pos_.transpose()<<"\n";
    stuck_time_ =0.0;
    flag_work_ = true;
}

void LwrTrajGenerator::stopTrajectroies(){
    joint_des_pos_= joint_msr_pos_;
    for (int i = 0; i < joint_des_pos_.size(); i++){
        cmd_msg_.data[i] = joint_des_pos_(i);
    }
    pub_posture_.publish (cmd_msg_);
    std::cout<< "Stop desired position: " <<joint_des_pos_.transpose()<<"\n";
    flag_work_ = false;
}

void LwrTrajGenerator::goToPosition(Eigen::VectorXd goal_pos){
    if(goal_pos.size()!=7){
        ROS_WARN ("Incorrect size for goal position");
    }
    else{
        joint_des_pos_= goal_pos;
        for (int i = 0; i < joint_des_pos_.size(); i++){
            cmd_msg_.data[i] = joint_des_pos_(i);
        }
        pub_posture_.publish (cmd_msg_);
        std::cout<< "Desired position: " <<joint_des_pos_.transpose()<<"\n";
        flag_work_ = false;
    }
}

void LwrTrajGenerator::getJointPosition(const sensor_msgs::JointState &msg){
    // the joint order is:
    // ['lwr_a1_joint', 'lwr_a2_joint', 'lwr_a3_joint', 'lwr_a4_joint', 'lwr_a5_joint', 'lwr_a6_joint', 'lwr_e1_joint']
    int joint_indexes[7] = {0, 1, 3, 4, 5, 6, 2};
    for (int i = 0; i < joint_msr_pos_.size(); i++){
        joint_msr_pos_(joint_indexes[i]) =  msg.position[i];
    }
}

