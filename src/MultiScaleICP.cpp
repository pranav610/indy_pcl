/*#include <iostream>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <open3d/Open3D.h>
#include <open3d/t/geometry/Utility.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/t/geometry/PointCloud.h>

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
#include <sensor_msgs/point_cloud2_iterator.h>

#define DONT_RESAMPLE -1

// using namespace open3d;
using namespace message_filters;

open3d::t::geometry::PointCloud merged;
open3d::geometry::PointCloud temp_merged;
std::vector< double > voxel_sizes (0.05, DONT_RESAMPLE);
std::vector< double > max_dist (0.07, 5);

auto criteria1 = open3d::t::pipelines::registration::ICPConvergenceCriteria(0.01, 0.01, 15);
auto criteria2 = open3d::t::pipelines::registration::ICPConvergenceCriteria(0.001, 0.001, 10);
std::vector<open3d::t::pipelines::registration::ICPConvergenceCriteria> criterias_{criteria1,criteria2};

// criterias_.push_back(criteria1);
// criterias_.push_back(criteria2);
//auto criteria = ICPConvergenceCriteria(relative_fitness[i], relative_rmse[i], max_iterations[i]);

std::shared_ptr<open3d::t::pipelines::registration::TransformationEstimation> estimation_={std::make_shared<open3d::t::pipelines::registration::TransformationEstimationPointToPoint>()};
// *estimation_ = std::make_shared<open3d::t::pipelines::registration::TransformationEstimationPointToPoint>();

open3d::core::Tensor initial_transform = open3d::core::Tensor::Eye(4, open3d::core::Dtype::Float64, open3d::core::Device("CPU:0"));

open3d::core::Tensor result_transform;

void Callback(const nav_msgs::Odometry::ConstPtr &odom, const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const geometry_msgs::TransformStamped::ConstPtr &tf_msg, const geometry_msgs::TransformStamped::ConstPtr &tf_msg_wrt){
  open3d::geometry::PointCloud temp_lidarScan;
  open3d::t::geometry::PointCloud lidarScan;

  sensor_msgs::PointCloud2ConstIterator<float> pcl_msg_x(*pcl_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> pcl_msg_y(*pcl_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> pcl_msg_z(*pcl_msg, "z");
  temp_lidarScan.points_.reserve(pcl_msg->height * pcl_msg->width);
  if (pcl_msg->fields.size() == 3)
  {
    for (size_t i = 0; i < pcl_msg->height * pcl_msg->width; ++i, ++pcl_msg_x, ++pcl_msg_y, ++pcl_msg_z)
    {
      temp_lidarScan.points_.push_back(Eigen::Vector3d(*pcl_msg_x, *pcl_msg_y, *pcl_msg_z));
    }
  }
  //lidarScan = temp_lidarScan.ToLegacyPointCloud();
  Eigen::Matrix4d transformation;
  Eigen::Quaternionf quats;
  
  quats.x() = odom->pose.pose.orientation.x + tf_msg_wrt->transform.rotation.x;
  quats.y() = odom->pose.pose.orientation.y + tf_msg_wrt->transform.rotation.y;
  quats.z() = odom->pose.pose.orientation.z + tf_msg_wrt->transform.rotation.z;
  quats.w() = odom->pose.pose.orientation.w + tf_msg_wrt->transform.rotation.w;

  Eigen::Matrix3f R_once = ((quats.normalized()).toRotationMatrix());

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        if (i == 3)
        {
          transformation(i, j) = 0.00;
        }
        else
        {
          transformation(i, j) = R_once(i, j);
        }
      }
    }

    transformation(0, 3) = odom->pose.pose.position.x + tf_msg_wrt->transform.translation.x; 
    transformation(1, 3) = odom->pose.pose.position.y + tf_msg_wrt->transform.translation.y;
    transformation(2, 3) = odom->pose.pose.position.z + tf_msg_wrt->transform.translation.z;
    transformation(3, 3) = 1.00;
    
    temp_lidarScan.Transform(transformation);
    lidarScan = open3d::t::geometry::PointCloud::FromLegacyPointCloud(temp_lidarScan);

    open3d::t::pipelines::registration::RegistrationResult result;

    result = open3d::t::pipelines::registration::RegistrationMultiScaleICP(lidarScan, merged, voxel_sizes, criterias_, max_dist, initial_transform,*estimation_);

    // ROS_INFO_STREAM("x="<<result.transformation_[0][3]<<"y="<<result.transformation_[1][3]<<"z="<<result.transformation_[2][3]);
    ROS_INFO("x=%lf, y=%lf, z=%lf", tf_msg->transform.translation.x, tf_msg->transform.translation.y, tf_msg->transform.translation.z);
}


int main(int argc, char **argv)
{

  if (open3d::io::ReadPointCloudFromPCD("/home/pranav/catkin_ws/src/MergedMap.pcd",temp_merged,{"auto", false, false, true})==false) // load the file
  {
    ROS_ERROR("Couldn't read file MergedMap.pcd");
    return (-1);
  }
  merged = open3d::t::geometry::PointCloud::FromLegacyPointCloud(temp_merged);
  ROS_INFO("Map is loaded");
  
  ros::init(argc, argv, "MultiScaleICP");
  ros::NodeHandle nh_MultiScaleICP;

  message_filters::Subscriber<nav_msgs::Odometry> sub(nh_MultiScaleICP, "/noisy_data/noise_added", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh_MultiScaleICP, "/carla/vehicle/086/lidar/front/point_cloud", 10);
  message_filters::Subscriber<geometry_msgs::TransformStamped> tf_sub(nh_MultiScaleICP, "/tf/lidar", 10);
  message_filters::Subscriber<geometry_msgs::TransformStamped> tf_sub_wrt(nh_MultiScaleICP, "/tf/lidar/wrt/odom", 10);
  TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> sync(sub, pcl_sub, tf_sub, tf_sub_wrt, 10);
  sync.registerCallback(boost::bind(&Callback, _1, _2, _3, _4));

  ROS_INFO("Going into callback..");

  ros::spin();
  return 0;
}*/