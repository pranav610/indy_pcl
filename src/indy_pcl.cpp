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
int count1=0;
int count=0;
int flag;
void callback(const sensor_msgs::PointCloud2::ConstPtr& msg2, const geometry_msgs::TransformStamped::ConstPtr& msg1)
{
  // Solve all of perception here...
  geometry_msgs::Transform temp_tf=msg1->transform;
  sensor_msgs::PointCloud2 msg;

pcl_ros::transformPointCloud("map",temp_tf,*msg2,msg);
pcl::PCLPointCloud2 temp1;
//PointCloud::Ptr temp2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl_conversions::toPCL(msg,temp1);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromPCLPointCloud2(temp1,*temp_cloud);
//if(msg2->header.stamp==temp_tf.header.stamp)

merged+=(*temp_cloud);
count++;
ROS_INFO("Transformed %d point clouds",count);

  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "indy_pcl");

  ros::NodeHandle nh;
  
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/carla/vehicle/086/lidar/front/point_cloud", 1);
  message_filters::Subscriber<geometry_msgs::TransformStamped> tf_sub(nh, "/pub", 1);
  TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> sync(pcl_sub, tf_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  int n=pcl::io::savePCDFileASCII("Merged0.25.pcd",merged);
  return 0;
}



/*

geometry_msgs::Transform temp_tf; 
int count=0;
int count1=0;
int count2=0;
int flag;
void scanCallback1(const tf2_msgs::TFMessage::ConstPtr &msg1)
{ 
 	geometry_msgs::TransformStamped msg;
 tf2_msgs::TFMessage tempp=*msg1;
flag=0;
if((tempp.transforms).size()==2){
	msg=msg1->transforms[0];
  int index=0;
  std::string s2="86";
  if(!((msg.child_frame_id).compare(s2))){
  	msg=msg1->transforms[1];
  	index=1;
  	}
  temp_tf=(tempp.transforms[index]).transform;
  count1++;
  flag=1;	
}}
void scanCallback2(const sensor_msgs::PointCloud2::ConstPtr &msg2)
{
sensor_msgs::PointCloud2 msg;

pcl_ros::transformPointCloud("map",temp_tf,*msg2,msg);
pcl::PCLPointCloud2 temp1;
//PointCloud::Ptr temp2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl_conversions::toPCL(msg,temp1);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromPCLPointCloud2(temp1,*temp_cloud);
//if(msg2->header.stamp==temp_tf.header.stamp)
if(flag){
merged+=(*temp_cloud);
count++;}
ROS_INFO("Transformed %d point clouds & used %d messeges",count,count1);
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
*/
/*
int count1=0;
int count2=0;
ros::Publisher pu;
void scanCallback1(const tf2_msgs::TFMessage::ConstPtr &msg1)
{
	count2++;
	 geometry_msgs::TransformStamped msg;
	 if((msg1->transforms).size()==2){
	 
  msg=msg1->transforms[0];
  int index=-1;
  std::string s2="86";
  if(!((msg.child_frame_id).compare(s2))){
  	msg=msg1->transforms[1];
  	}
 /* for(int i=0;i=1;i++){
  	if(((msg1->transforms[i]).child_frame_id.compare(s2))==0) index=i;
  	}
 	
  count1++;
  //flag=1;
  
  pu.publish(msg);
  ROS_INFO_STREAM_THROTTLE(0.001,msg.child_frame_id<<count1<<count2);
}
}
int main(int argc, char ** argv){

	ros::init (argc,argv,"indy_pcl");
	ros::NodeHandle nh("~");
	pu=nh.advertise<geometry_msgs::TransformStamped>("/pub",100);
	ros::Subscriber sub1 = nh.subscribe("/tf", 100, scanCallback1);
	
	ros::spin();
	
	
}*/























