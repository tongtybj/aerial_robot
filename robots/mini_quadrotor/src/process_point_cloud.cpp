#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
void doPointcloudCallback(const sensor_msgs::PointCloud& msg){
    ROS_INFO("time stamp is: %f",msg.header.stamp );
}
int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"pointcloud");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/quadrotor/livox/scan10,doPointcloudCallback);
    ros::spin();
    return 0;
}
