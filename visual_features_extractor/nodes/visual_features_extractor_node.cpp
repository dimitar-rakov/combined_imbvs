
#include <ros/ros.h>
#include "visual_features_extractor/visual_features_extractor.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_features_extractor_node");
    ros::NodeHandle nh = ros::NodeHandle("~");
    VisualFeaturesExtractor vfe;

    if (!vfe.init(nh)){
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
        vfe.update(ros::Time::now(), work_period);
        //ROS_WARN ("Update time is %lf", (ros::Time::now() - upd_time).toSec());
        msr_period = ros::Time::now() - last_time;
        if (msr_period< des_period)
            (des_period - msr_period).sleep();
        else
            ROS_WARN ("Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
        last_time = ros::Time::now();
        work_period = last_time - init_time ;
    }
    return 0;
}
