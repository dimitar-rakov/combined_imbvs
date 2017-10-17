#include <ros/ros.h>
#include "lwr_traj_generator/LwrTrajGenerator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lwr_traj_generator");
    ros::NodeHandle n;
    LwrTrajGenerator tg;
    tg.init(n);
    ros::Rate loop_rate(100);

    double init_time = ros::Time::now().toSec();
    double last_time=  ros::Time::now().toSec();
    double time = 0.0;
    ros::Duration period;

    while(ros::ok())
    {
        time = ros::Time::now().toSec() - init_time;
        period.fromSec(ros::Time::now().toSec() - last_time);
        last_time =  ros::Time::now().toSec();
        tg.update(ros::Time::now() , period);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
