#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <livox_ros_driver2/CustomMsg.h>
class CeilingDistance
{
public:
  CeilingDistance(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp) {

    nhp_.param("theta_thresh", theta_thresh_, 40.0); // deg
    theta_thresh_ *= (M_PI / 180.0);
    nhp_.param("circle_thresh_", circle_thresh_, 10.0); // deg
    circle_thresh_*= 0.3;

    pub_= nh_.advertise<std_msgs::Float32>("/quadrotor/ceiling/min_distance",10);
    sub_= nh_.subscribe("/quadrotor/livox/scan",1, &CeilingDistance::pointcloudCallback,this);
    // pub_trans= nh_.advertise<std_msgs::Float32>("/quadrotor/ceiling/min_distance",10);
    sub_trans_= nh_.subscribe("/livox/lidar",1, &CeilingDistance::customMsg_transformCallback,this);

  }

  ~CeilingDistance(){}
  void customMsg_transformCallback(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg){
    // ROS_INFO("get livox custom msg");
    float min_pos_z = 1e6;
    float max_theta = 0;
    for(int i=0; i < livox_msg->points.size(); i++){
      float position_x = livox_msg->points.at(i).x;
      float position_y = livox_msg->points.at(i).y;
      float position_z = livox_msg->points.at(i).z;
      float theta = atan2(position_z, sqrt(position_y * position_y + position_x * position_x));

      if (theta > max_theta) {
        max_theta = theta;
      }

      if (theta < theta_thresh_) {
        continue;
      }

      // get the pos_z of the valid point

      // get the minimum position z when target point in circle 
      if (position_z < min_pos_z && pow(position_x,2) < pow(circle_thresh_,2) &&pow(position_y,2) < pow(circle_thresh_,2))
        {
          // ROS_INFO("OK");
          min_pos_z = position_z;
        }
    }

    ROS_INFO("max theta is : %f, min_pos_z: %f", max_theta, min_pos_z);

    // ROS_INFO("min position z(the distance to ceiling) is %f ", min_pos_z);
    std_msgs::Float32 dist_msg;
    dist_msg.data = min_pos_z;
    pub_.publish(dist_msg);

  }
  void pointcloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
     // ROS_INFO("time stamp is: %f, point size is %d ",msg->header.stamp.toSec(), msg->points.size());
    // ROS_INFO("get point cloud msg");
    float min_pos_z = 1e6;
    // int d = 0;
    // int e = 0;
    // read the points xyz position
    for(int i=0; i < msg->points.size(); i++){
      // int i = 0;
      //  for (auto const point: msg->points ) {
      // print out all point position
      // ROS_INFO("x:%f y:%f z:%f", pc.points[i].x, pc.points[i].y, pc.points[i].z);
      // step1: calculate theta
      float position_x = msg->points.at(i).x;
      float position_y = msg->points.at(i).y;
      float position_z = msg->points.at(i).z;
      float theta = atan2(position_z, sqrt(position_y * position_y + position_x * position_x));///change from double
      //ROS_INFO("theta is : %f",theta);
      float thresh = 50 * M_PI / 180.0; ///change from double

      // step1.5: skip if theta is too small
      if (theta < thresh) {
        // d++;
        continue;
      }

      // get the pos_z of the valid point

      // get the minimum position z
      if (position_z < min_pos_z )
        {
          min_pos_z = position_z;
        }
    }
    ROS_INFO("OK")
    // ROS_INFO("min position z(the distance to ceiling) is %f ", min_pos_z);
    std_msgs::Float32 dist_msg;
    dist_msg.data = min_pos_z;
    pub_.publish(dist_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  // ros::Publisher pub_trans_;
  ros::Subscriber sub_trans_;
  double circle_thresh_;
  double theta_thresh_;
};

int main(int argc, char  *argv[])
{
  ros::init(argc,argv,"pointcloud");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  CeilingDistance ceiling_distance(nh, nhp);

  ros::spin();
  return 0;
}
