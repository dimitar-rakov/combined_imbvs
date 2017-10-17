#ifndef KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define KINEMATIC_CHAIN_CONTROLLER_BASE_H

#include <control_msgs/JointControllerState.h> // TODO: state message for all controllers?

#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <vector>
#include <lwr_hw_interfaces/joint_impedance_state_interface.h>
#include <lwr_hw_interfaces/joint_impedance_command_interface.h>
namespace controller_interface
{

const static double MAX_V = 0.05;
const static double MAX_W = 0.25;
const static double MAX_JOINT_DES_STEP = 0.00002;
template<typename JI>
class KinematicChainControllerBase: public Controller<JI>
{
public:
    KinematicChainControllerBase() {}
    ~KinematicChainControllerBase() {}

    bool init(JI *robot, ros::NodeHandle &n);

protected:
    ros::NodeHandle nh_;
    KDL::Chain kdl_chain_;
    KDL::Vector gravity_;
    KDL::JntArrayAcc joint_msr_states_, joint_des_states_;  // joint states (measured and desired)
    KDL::JntArrayAcc joint_msr_states_prev_, joint_des_states_prev_;  // joint states used by trapezoidal integration)

    struct limits_
    {
        KDL::JntArray min;
        KDL::JntArray max;
        KDL::JntArray center;
        KDL::JntArray velocity;
        KDL::JntArray acceleration;
        KDL::JntArray effort;
    } joint_limits_;


    std::vector<typename JI::ResourceHandleType> joint_handles_;
};

template <typename JI>
bool KinematicChainControllerBase<JI>::init(JI *robot, ros::NodeHandle &n)
{
    nh_ = n;

    // get URDF and name of root and tip from the parameter server
    std::string robot_description, root_name, tip_name;

    if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
        return false;
    }

    if (!nh_.getParam("root_name", root_name))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
        return false;
    }

    if (!nh_.getParam("tip_name", tip_name))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
        return false;
    }

    // Get the gravity vector (direction and magnitude). Value taken from Mada/custom.dat



    // take the value from the parameter server
    if ( !(ros::param::get(std::string("/lwr/gravity_wrt_robot_base/x"), gravity_(0))
           && ros::param::get(std::string("/lwr/gravity_wrt_robot_base/y"), gravity_(1))
           && ros::param::get(std::string("/lwr/gravity_wrt_robot_base/z"), gravity_(2)))){
        gravity_ = KDL::Vector(0, 0, -9.80665016);
        ROS_WARN("Default gravity vector w.r.t robot base is set: x:%f  y:%f  z:%f", gravity_(0), gravity_(1), gravity_(2));
    }

    // Construct an URDF model from the xml string
    std::string xml_string;

    if (n.hasParam(robot_description))
        n.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
        n.shutdown();
        return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
        n.shutdown();
        return false;
    }

    ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        n.shutdown();
        return false;
    }
    ROS_INFO("Successfully parsed urdf file");

    KDL::Tree kdl_tree_;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        n.shutdown();
        return false;
    }

    // Populate the KDL chain
    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
            ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }

    ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

    // Parsing joint limits from urdf model along kdl chain
    boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint_;
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.velocity.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.acceleration.resize(kdl_chain_.getNrOfJoints()                                                                                                                                                                                                                                                                                                                                                                          );
    joint_limits_.effort.resize(kdl_chain_.getNrOfJoints());
    int index;

    for (int i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
    {
        joint_ = model.getJoint(link_->parent_joint->name);
        ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());
        index = kdl_chain_.getNrOfJoints() - i - 1;

        joint_limits_.min(index) = joint_->limits->lower;
        joint_limits_.max(index) = joint_->limits->upper;
        joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;
        joint_limits_.velocity(index) = joint_->limits->velocity;
        joint_limits_.acceleration(index) = joint_->limits->velocity *0.5; // maximum velocity can be achieved in 2.0 seconds
        joint_limits_.effort(index) = joint_->limits->effort;

        link_ = model.getLink(link_->getParent()->name);
    }

    // Get joint handles for all of the joints in the chain
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    {
        joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
        ROS_DEBUG("%s", it->getJoint().getName().c_str() );
    }

    ROS_DEBUG("Number of joints in handle = %lu", joint_handles_.size() );

    joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
    joint_des_states_.resize(kdl_chain_.getNrOfJoints());
    joint_msr_states_prev_.resize(kdl_chain_.getNrOfJoints());
    joint_des_states_prev_.resize(kdl_chain_.getNrOfJoints());

    return true;
}

}

#endif
