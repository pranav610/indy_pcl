#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <algorithm>
#include <random>
#include <cmath>

float mean_translational = 0;
float mean_rotational = 0;
float stddev_translational = 0.50;
float stddev_rotational = 0.08726642;

ros::Publisher pub;

void scanCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  nav_msgs::Odometry msg1;
  msg1.pose = msg->pose;

  std::random_device rd;

  std::default_random_engine generator1;
  generator1.seed(rd()); //Now this is seeded differently each time.
  std::normal_distribution<double> distribution1(mean_translational, stddev_translational);

  std::default_random_engine generator2;
  generator2.seed(rd()); //Now this is seeded differently each time.
  std::normal_distribution<double> distribution2(mean_rotational, stddev_rotational);

  float error1 = distribution1(generator1);
  float error2 = distribution2(generator2);

  msg1.pose.pose.position.x += error1;
  msg1.pose.pose.position.y += error1;
  msg1.pose.pose.position.z += error1;

  msg1.pose.pose.orientation.x += error2;
  msg1.pose.pose.orientation.y += error2;
  msg1.pose.pose.orientation.z += error2;
  msg1.pose.pose.orientation.w += error2;

  msg1.header = msg->header;
  msg1.child_frame_id = msg->child_frame_id;

  pub.publish(msg1);

  ROS_INFO("I heard: %f", msg1.pose.pose.position.x);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "noisy_data");
  ros::NodeHandle noisydata("~");

  pub = noisydata.advertise<nav_msgs::Odometry>("noise_added", 100);
  ros::Subscriber sub = noisydata.subscribe("/carla/vehicle/086/odometry", 100, scanCallback);

  ros::spin();
  return 0;
}
