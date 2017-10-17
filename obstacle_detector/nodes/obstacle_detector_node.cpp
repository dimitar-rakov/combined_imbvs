
#include <ros/ros.h>
#include "obstacle_detector/obstacle_detector.h"



int main(int argc, char **argv)
{

    ros::init(argc, argv, "obstacle_detector_node");
    ros::NodeHandle nh = ros::NodeHandle("~");
    ObstacleDetector od;

    if (!od.init(nh)){
        nh.shutdown();
        return -1;
    }

    ros::Time last_time = ros::Time::now(), init_time = ros::Time::now(), upd_time = ros::Time::now();
    ros::Duration msr_period(0.0),des_period(0.08), work_period(0.08);

    while(ros::ok())
    {
        init_time = ros::Time::now();
        ros::spinOnce();
        upd_time = ros::Time::now();
        od.update(ros::Time::now(), work_period);
        ROS_WARN ("Update time is %lf", (ros::Time::now() - upd_time).toSec());
        msr_period = ros::Time::now() - last_time;
        if (msr_period< des_period)
            (des_period - msr_period).sleep();
        else
            ROS_WARN ("Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
        od.publish();
        last_time = ros::Time::now();
        work_period = last_time - init_time ;
    }
    return 0;
}
