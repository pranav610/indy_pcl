#include <iostream>

#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/registration/icp.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>

using namespace message_filters;

ros::Publisher vis;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr merged(new PointCloud); //MergedMap_downsampled.pcd
PointCloud::Ptr cloud(new PointCloud); //saving localized point cloud
PointCloud Final; //alligned point cloud aftre applying ICP 

float resolution = 30.0f; //min voxel size for ict tree 
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

int count = 0;
int iterations = 1000;
float radius = 30.0f;

void Callback(const nav_msgs::Odometry::ConstPtr &odom, const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const geometry_msgs::TransformStamped::ConstPtr &tf_msg)
{
  ROS_INFO("callback is running");
  
  PointCloud::Ptr temp_cloud(new PointCloud);
  pcl::fromROSMsg(*pcl_msg, *temp_cloud);

  pcl::PCLPointCloud2::Ptr point_cloud2(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*temp_cloud, *point_cloud2);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(point_cloud2);
  sor.setLeafSize(0.3f, 0.3f, 0.3f);
  sor.filter(*point_cloud2);

  pcl::fromPCLPointCloud2(*point_cloud2, *temp_cloud);

  ROS_INFO("Downsampled Input");

  pcl::PointXYZ searchPoint;

  searchPoint.x = odom->pose.pose.position.x;
  searchPoint.y = odom->pose.pose.position.y;
  searchPoint.z = odom->pose.pose.position.z;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      (*cloud).points.push_back((*merged)[pointIdxRadiusSearch[i]]);
    }
  }
  count++;
  ROS_INFO("Local map extraction of %d point clouds.(Oct-Tree Done)", count);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud);
  icp.setInputTarget(temp_cloud);
  icp.setMaximumIterations(iterations);
  icp.align(Final);
  ROS_INFO("has converged %d score.(ICP Done)", icp.hasConverged());
  
  //rviz visualisation 
  sensor_msgs::PointCloud2 temp_sensor;
  pcl::toROSMsg(Final, temp_sensor);
  temp_sensor.header.frame_id = "map";
  vis.publish(temp_sensor);
  ROS_INFO("Publissing Done");

  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  Eigen::Matrix3f mat; //rotation matrix
  Eigen::Vector3f trans; //translation vector
  
  ROS_INFO("Transformation Given By ICP is,");
  ROS_INFO("%f %f %f",transformation(0,3),transformation(1,3),transformation(2,3));
  
  //Eigen::Isometry3d temp_matrix;
  Eigen::Matrix4f ground_truth;
  //temp_matrix = tf2::transformToEigen(*tf_msg);

  //ground_truth = (temp_matrix.matrix()); 
  Eigen::Quaternionf quats;
  quats.x()=tf_msg->transform.rotation.x;
  quats.y()=tf_msg->transform.rotation.y;
  quats.z()=tf_msg->transform.rotation.z;

  Eigen::Matrix3f R = (quats.normalized()).toRotationMatrix();
  
  for(int i=0; i<4; i++){
    for(int j=0; j<3; j++){
      if(i==3){
        ground_truth(i,j) = 0;
      }else{
        ground_truth(i,j) = R(i,j);
      }
    }
  }
  
  ground_truth(0,3) = tf_msg->transform.translation.x;
  ground_truth(1,3) = tf_msg->transform.translation.y;
  ground_truth(2,3) = tf_msg->transform.translation.z;
  ground_truth(3,3) = 1;

  Eigen::Matrix4f error;

  error = (ground_truth)*(transformation.inverse());
  
  ROS_INFO("Error Matrix is,");
  ROS_INFO_STREAM(error);

  (*cloud).points.clear();
  return;
}

int main(int argc, char **argv)
{

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/pranav/catkin_ws/src/indy_pcl/MergedMap_downsampled.pcd", *(merged)) == -1) // load the file
  {
    PCL_ERROR("Couldn't read file MergedMap_downsampled.pcd \n");
    return (-1);
  }
  octree.setInputCloud(merged);
  octree.addPointsFromInputCloud();
  ROS_INFO("Map is loaded");
  ros::init(argc, argv, "ICP");

  ros::NodeHandle nh_ICP;

  vis = nh_ICP.advertise<sensor_msgs::PointCloud2>("/viscloud", 10);
  message_filters::Subscriber<nav_msgs::Odometry> sub(nh_ICP, "/noisy_data/noise_added", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh_ICP, "/carla/vehicle/086/lidar/front/point_cloud", 10);
  message_filters::Subscriber<geometry_msgs::TransformStamped> tf_sub(nh_ICP, "/pub", 10);
  TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> sync(sub, pcl_sub, tf_sub, 10);
  ROS_INFO("Going into callback..");
  sync.registerCallback(boost::bind(&Callback, _1, _2, _3));

  ros::spin();
  return 0;
}