#include <ros/ros.h>
#include "kinect_fusion/kinect_fusion.h"



void updateCB(const ros::TimerEvent& event, KinectFusion &kf)
{

  //ros::spinOnce();
  kf.update(event.current_real, event.current_real - event.last_real);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_fusion_node");
    ros::NodeHandle nh = ros::NodeHandle("~");

    KinectFusion kf;
    kf.init(nh);
    //ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(&updateCB, _1, kf));

    ros::Time last_time = ros::Time::now(), init_time = ros::Time::now(), upd_time = ros::Time::now();

    // Remark currently all fusion is done in callback, therefore des_period does not matter
    ros::Duration msr_period(0.010),des_period(0.010), work_period(0.010);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    while(ros::ok())
    {
        init_time = ros::Time::now();
        //ros::spinOnce();
        upd_time = ros::Time::now();
        kf.update(ros::Time::now(), work_period);
        //ROS_WARN ("Update time is %lf", (ros::Time::now() - upd_time).toSec());
        msr_period = ros::Time::now() - last_time;
        if (msr_period< des_period)
            (des_period - msr_period).sleep();
        else
            ROS_WARN ("Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
        last_time = ros::Time::now();
        work_period = last_time - init_time ;
    }
    spinner.stop();

    return 0;
}

