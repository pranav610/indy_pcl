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

//ros::Publisher vis;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr merged(new PointCloud); //MergedMap_downsampled.pcd
PointCloud Final;                       //alligned point cloud aftre applying ICP
Eigen::Matrix4f transformation_prev;
int count_callback = 0;
//int count = 0;
//int iterations = 200;

void Callback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const geometry_msgs::TransformStamped::ConstPtr &tf_msg)
{
  count_callback++;
  ROS_INFO("callback is running");
  geometry_msgs::Transform temp_tf;
  sensor_msgs::PointCloud2 msg;

  ROS_INFO("%f %f %f", tf_msg->transform.translation.x, tf_msg->transform.translation.y, tf_msg->transform.translation.z);

  Eigen::Quaternionf quats_once;
  quats_once.x() = tf_msg->transform.rotation.x;
  quats_once.y() = tf_msg->transform.rotation.y;
  quats_once.z() = tf_msg->transform.rotation.z;
  quats_once.w() = tf_msg->transform.rotation.w;

    Eigen::Matrix3f R_once = ((quats_once.normalized()).toRotationMatrix());

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        if (i == 3)
        {
          transformation_prev(i, j) = 0.00;
        }
        else
        {
          transformation_prev(i, j) = R_once(i, j);
        }
      }
    }

    transformation_prev(0, 3) = tf_msg->transform.translation.x;
    transformation_prev(1, 3) = tf_msg->transform.translation.y;
    transformation_prev(2, 3) = tf_msg->transform.translation.z;
    transformation_prev(3, 3) = 1.00;

    ROS_INFO("Guess transformation is loaded.");
  
  pcl_ros::transformPointCloud("map", tf_msg->transform, *pcl_msg, msg);
  PointCloud::Ptr temp_cloud(new PointCloud);
  pcl::fromROSMsg(msg, *temp_cloud);

  ROS_INFO("Used guess transform.");

  pcl::PCLPointCloud2::Ptr point_cloud2(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*temp_cloud, *point_cloud2);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(point_cloud2);
  sor.setLeafSize(0.3f, 0.3f, 0.3f);
  sor.filter(*point_cloud2);

  pcl::fromPCLPointCloud2(*point_cloud2, *temp_cloud);

  ROS_INFO("Downsampled Input.");

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(temp_cloud);
  icp.setInputTarget(merged);
  icp.setMaxCorrespondenceDistance(1);
  //icp.setMaximumIterations(iterations);
  icp.align(Final);
  ROS_INFO("Has converged %d score. (ICP Done)", icp.hasConverged());

  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  //transformation = transformation.inverse();
  Eigen::Matrix3f mat;   //rotation matrix
  Eigen::Vector3f trans; //translation vector

  transformation_prev = (transformation_prev) * (transformation);

  ROS_INFO("Overall Transformation Given By ICP is,");
  ROS_INFO("%f %f %f", transformation_prev(0, 3), transformation_prev(1, 3), transformation_prev(2, 3));
  ROS_INFO("Residual Transformation Given By ICP is,");
  ROS_INFO("%f %f %f", transformation(0, 3), transformation(1, 3), transformation(2, 3));
  ROS_INFO("Transformation Given By tf is,");
  ROS_INFO("%f %f %f", tf_msg->transform.translation.x, tf_msg->transform.translation.y, tf_msg->transform.translation.z);

  //Eigen::Isometry3d temp_matrix;
  Eigen::Matrix4f ground_truth;
  //temp_matrix = tf2::transformToEigen(*tf_msg);

  //ground_truth = (temp_matrix.matrix());
  Eigen::Quaternionf quats;
  quats.x() = tf_msg->transform.rotation.x;
  quats.y() = tf_msg->transform.rotation.y;
  quats.z() = tf_msg->transform.rotation.z;
  quats.w() = tf_msg->transform.rotation.w;

  Eigen::Matrix3f R = ((quats.normalized()).toRotationMatrix());

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (i == 3)
      {
        ground_truth(i, j) = 0.00;
      }
      else
      {
        ground_truth(i, j) = R(i, j);
      }
    }
  }

  ground_truth(0, 3) = tf_msg->transform.translation.x;
  ground_truth(1, 3) = tf_msg->transform.translation.y;
  ground_truth(2, 3) = tf_msg->transform.translation.z;
  ground_truth(3, 3) = 1.00;

  Eigen::Matrix4f error;

  error = (ground_truth) * (transformation_prev.inverse());

  ROS_INFO("Error Matrix is,");
  ROS_INFO_STREAM(error);

  return;
}

int main(int argc, char **argv)
{

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/pranav/catkin_ws/src/MergedMap_downsampled.pcd", *(merged)) == -1) // load the file
  {
    PCL_ERROR("Couldn't read file MergedMap_downsampled.pcd ");
    return (-1);
  }
  ROS_INFO("Map is loaded");
  ros::init(argc, argv, "ICP_test");

  ros::NodeHandle nh_ICPtest;

  //vis = nh_ICP.advertise<sensor_msgs::PointCloud2>("/viscloud", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh_ICPtest, "/carla/vehicle/086/lidar/front/point_cloud", 10);
  message_filters::Subscriber<geometry_msgs::TransformStamped> tf_sub(nh_ICPtest, "/pub", 10);
  TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> sync(pcl_sub, tf_sub, 10);
  ROS_INFO("Going into callback..");
  sync.registerCallback(boost::bind(&Callback, _1, _2));

  ros::spin();
  return 0;
}