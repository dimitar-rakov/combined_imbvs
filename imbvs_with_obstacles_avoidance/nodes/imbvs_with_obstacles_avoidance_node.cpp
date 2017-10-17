#include <ros/ros.h>
#include "imbvs_with_obstacles_avoidance/imbvs_with_obstacles_avoidance.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imbvs_with_obstacles_avoidance_node");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ImbvsWithObstaclesAvoidance vs;

    if (!vs.init(nh)){
        nh.shutdown();
        return -1;
    }

    ros::Time last_time = ros::Time::now(), init_time = ros::Time::now();
    ros::Duration msr_period(0.0),des_period(0.02), work_period(0.02);
    while(ros::ok())
    {
        init_time = ros::Time::now();
        ros::spinOnce();
        vs.update(ros::Time::now(), work_period);
        msr_period = ros::Time::now() - last_time;
        if (msr_period< des_period)
            (des_period - msr_period).sleep();
        else
            ROS_WARN ("Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
        vs.publish();
        last_time = ros::Time::now();
        work_period = last_time - init_time ;
    }
    return 0;
}
