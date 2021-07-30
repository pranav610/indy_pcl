#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>

int count1 = 0;
int count2 = 0;
ros::Publisher pu1;
ros::Publisher pu2;
void scanCallback1(const tf2_msgs::TFMessage::ConstPtr &msg1)
{
  count1++;
  int index=-1;
  if(msg1->transforms.size()==2){
  if(((msg1->transforms[0].child_frame_id).compare(msg1->transforms[1].child_frame_id))!=0){
  geometry_msgs::TransformStamped msg_tf_lidar_odom;
  geometry_msgs::TransformStamped msg_tf_lidar;
  std::string s2 = "vehicle/086/lidar/front";
  for (int i = 0; i < (msg1->transforms).size(); i++)
  {
    if (((msg1->transforms[i].child_frame_id).compare(s2)) == 0)
    {
      index=i;      
    }
  }
  if(index!=(-1)){
    count2++;
      msg_tf_lidar=msg1->transforms[index];
      msg_tf_lidar_odom.child_frame_id = "Lidar/wrt/odom";
      msg_tf_lidar_odom.header = msg1->transforms[index].header;
      msg_tf_lidar_odom.transform.translation.x = msg1->transforms[index].transform.translation.x - msg1->transforms[1-index].transform.translation.x ;
      msg_tf_lidar_odom.transform.translation.y = msg1->transforms[index].transform.translation.y - msg1->transforms[1-index].transform.translation.y ;
      msg_tf_lidar_odom.transform.translation.z = msg1->transforms[index].transform.translation.z - msg1->transforms[1-index].transform.translation.z ;

      msg_tf_lidar_odom.transform.rotation.x = msg1->transforms[index].transform.rotation.x - msg1->transforms[1-index].transform.rotation.x ;
      msg_tf_lidar_odom.transform.rotation.y = msg1->transforms[index].transform.rotation.y - msg1->transforms[1-index].transform.rotation.y ;
      msg_tf_lidar_odom.transform.rotation.z = msg1->transforms[index].transform.rotation.z - msg1->transforms[1-index].transform.rotation.z ;
      msg_tf_lidar_odom.transform.rotation.w = msg1->transforms[index].transform.rotation.w - msg1->transforms[1-index].transform.rotation.w ;

      pu1.publish(msg_tf_lidar);
      pu2.publish(msg_tf_lidar_odom);

      ROS_INFO("generated %d messeges from %d callbacks.",count2,count1);
    
  }
  
  
  }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "createbag");
  ros::NodeHandle nh_createbag("~");
  pu1 = nh_createbag.advertise<geometry_msgs::TransformStamped>("/tf/lidar", 100);
  pu2 = nh_createbag.advertise<geometry_msgs::TransformStamped>("/tf/lidar/wrt/odom", 100);
  ros::Subscriber sub1 = nh_createbag.subscribe("/tf", 100, scanCallback1);

  ros::spin();
}