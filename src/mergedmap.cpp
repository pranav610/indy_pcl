#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace message_filters;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud merged;
int count = 0;

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg2, const geometry_msgs::TransformStamped::ConstPtr &msg1)
{
  geometry_msgs::Transform temp_tf = msg1->transform;
  sensor_msgs::PointCloud2 msg;
    
  pcl_ros::transformPointCloud("map", temp_tf, *msg2, msg);
  PointCloud::Ptr temp_cloud (new PointCloud);
  pcl::fromROSMsg(msg,*temp_cloud);

  merged += (*temp_cloud);
  count++;
  ROS_INFO("Transformed %d point clouds.", count);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mergedmap");

  ros::NodeHandle nh_mergedmap;

  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh_mergedmap, "/carla/vehicle/086/lidar/front/point_cloud", 1);
  message_filters::Subscriber<geometry_msgs::TransformStamped> tf_sub(nh_mergedmap, "/pub", 1);
  TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> sync(pcl_sub, tf_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  int n = pcl::io::savePCDFileASCII("MergedMap.pcd", merged);
  return 0;
}
