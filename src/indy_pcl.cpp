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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud merged;
geometry_msgs::Transform temp_tf; 

void scanCallback1(const tf2_msgs::TFMessage::ConstPtr &msg1)
{  tf2_msgs::TFMessage tempp=*msg1;
if((tempp.transforms).size()==2){
  temp_tf=(tempp.transforms[1]).transform;}	
}
void scanCallback2(const sensor_msgs::PointCloud2::ConstPtr &msg2)
{
sensor_msgs::PointCloud2 msg;

pcl_ros::transformPointCloud("map",temp_tf,*msg2,msg);
pcl::PCLPointCloud2 temp1;
//PointCloud::Ptr temp2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl_conversions::toPCL(msg,temp1);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromPCLPointCloud2(temp1,*temp_cloud);
merged+=(*temp_cloud);
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "indy_pcl");
  ros::NodeHandle nh("~");
  
  ros::Subscriber sub1 = nh.subscribe("/tf", 100, scanCallback1);
  
  ros::Subscriber sub2 = nh.subscribe("/carla/vehicle/086/lidar/front/point_cloud", 100, scanCallback2);
  
  ros::spin();
  int n=pcl::io::savePCDFileASCII("Merged.pcd",merged);
  return 0;
}
