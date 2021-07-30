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
PointCloud Final;                       //alligned point cloud aftre applying ICP
Eigen::Matrix4f transformation_prev;    //Resultant transformation from ICP
Eigen::Quaternionf quat_prev;

PointCloud::Ptr cloud(new PointCloud); //saving localized point cloud

float resolution = 30.0f; //min voxel size for ict tree 
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

float radius = 30.0f;
int count_callback = 0;
//int count = 0;
//int iterations = 10;

void Callback(const nav_msgs::Odometry::ConstPtr &odom, const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const geometry_msgs::TransformStamped::ConstPtr &tf_msg, const geometry_msgs::TransformStamped::ConstPtr &tf_msg_wrt)
{
  count_callback++;
  ROS_INFO("callback is running");
  geometry_msgs::Transform temp_tf;
  sensor_msgs::PointCloud2 msg;

  if (count_callback == 1)
  {
    temp_tf.translation.x = odom->pose.pose.position.x + tf_msg_wrt->transform.translation.x;
    temp_tf.translation.y = odom->pose.pose.position.y + tf_msg_wrt->transform.translation.y;
    temp_tf.translation.z = odom->pose.pose.position.z + tf_msg_wrt->transform.translation.z;

    temp_tf.rotation.x = odom->pose.pose.orientation.x + tf_msg_wrt->transform.rotation.x;
    temp_tf.rotation.y = odom->pose.pose.orientation.y + tf_msg_wrt->transform.rotation.y;
    temp_tf.rotation.z = odom->pose.pose.orientation.z + tf_msg_wrt->transform.rotation.z;
    temp_tf.rotation.w = odom->pose.pose.orientation.w + tf_msg_wrt->transform.rotation.w;

    ROS_INFO("%f %f %f",temp_tf.translation.x,temp_tf.translation.y,temp_tf.translation.z);
    ROS_INFO("%f %f %f",odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    ROS_INFO_STREAM(odom->header.stamp);

    Eigen::Quaternionf quats_once;
    quats_once.x() = odom->pose.pose.orientation.x;
    quats_once.y() = odom->pose.pose.orientation.y;
    quats_once.z() = odom->pose.pose.orientation.z;
    quats_once.w() = odom->pose.pose.orientation.w;

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

    transformation_prev(0, 3) = odom->pose.pose.position.x;
    transformation_prev(1, 3) = odom->pose.pose.position.y;
    transformation_prev(2, 3) = odom->pose.pose.position.z;
    transformation_prev(3, 3) = 1.00;

    quat_prev = quats_once;

    ROS_INFO("Guess transformation is loaded using odom");
  }
  else
  {
    temp_tf.translation.x = transformation_prev(0, 3);
    temp_tf.translation.y = transformation_prev(1, 3);
    temp_tf.translation.z = transformation_prev(2, 3);

    temp_tf.rotation.x = quat_prev.x();
    temp_tf.rotation.y = quat_prev.y();
    temp_tf.rotation.z = quat_prev.z();
    temp_tf.rotation.w = quat_prev.w();

    ROS_INFO("Guess transformation is loaded using prev ICP output");
  }
  pcl_ros::transformPointCloud("map", temp_tf, *pcl_msg, msg);
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

  ROS_INFO("Local map extraction of %d point clouds.(Oct-Tree Done)", count_callback);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud);
  icp.setInputTarget(temp_cloud);
  //icp.setMaximumIterations(iterations);
  icp.align(Final);
  ROS_INFO("Has converged %d score. (ICP Done)", icp.hasConverged());

  //rviz visualisation
  sensor_msgs::PointCloud2 temp_sensor;
  pcl::toROSMsg(Final, temp_sensor);
  temp_sensor.header.frame_id = "map";
  vis.publish(temp_sensor);
  ROS_INFO("Publishing to Rviz Done");

  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  //transformation = transformation.inverse();
  Eigen::Matrix3f mat;   //rotation matrix
  Eigen::Vector3f trans; //translation vector
  
  transformation_prev = (transformation_prev) * (transformation);

  mat << transformation_prev(0,0), transformation_prev(0,1), transformation_prev(0,2),
         transformation_prev(1,0), transformation_prev(1,1), transformation_prev(1,2),
         transformation_prev(2,0), transformation_prev(2,1), transformation_prev(2,2);

  Eigen::Quaternionf quat_prev(mat); //rotation matrix stored as a quaternion

  ROS_INFO("Transformation Given By ICP is,");
  ROS_INFO("%f %f %f", transformation_prev(0, 3), transformation_prev(1, 3), transformation_prev(2, 3));
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
    PCL_ERROR("Couldn't read file MergedMap_downsampled.pcd \n");
    return (-1);
  }
  ROS_INFO("Map is loaded");
  octree.setInputCloud(merged);
  octree.addPointsFromInputCloud();
  ros::init(argc, argv, "ICP");

  ros::NodeHandle nh_ICP;

  vis = nh_ICP.advertise<sensor_msgs::PointCloud2>("/viscloud", 10);
  message_filters::Subscriber<nav_msgs::Odometry> sub(nh_ICP, "/noisy_data/noise_added", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh_ICP, "/carla/vehicle/086/lidar/front/point_cloud", 10);
  message_filters::Subscriber<geometry_msgs::TransformStamped> tf_sub(nh_ICP, "/tf/lidar", 10);
  message_filters::Subscriber<geometry_msgs::TransformStamped> tf_sub_wrt(nh_ICP, "/tf/lidar/wrt/odom", 10);
  TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> sync(sub, pcl_sub, tf_sub, tf_sub_wrt, 10);
  ROS_INFO("Going into callback..");
  sync.registerCallback(boost::bind(&Callback, _1, _2, _3,_4));

  ros::spin();
  return 0;
}