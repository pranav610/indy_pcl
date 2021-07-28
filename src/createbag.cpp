#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>

int count1 = 0;
int count2 = 0;
ros::Publisher pu;
void scanCallback1(const tf2_msgs::TFMessage::ConstPtr &msg1)
{
  count2++;
  geometry_msgs::TransformStamped msg;
  std::string s2 = "vehicle/086/lidar/front";
  for (int i = 0; i < (msg1->transforms).size(); i++)
  {
    msg = msg1->transforms[i];
    if (((msg.child_frame_id).compare(s2)) == 0)
    {
      pu.publish(msg);
      count1++;
      ROS_INFO_STREAM_THROTTLE(0.001, msg.child_frame_id << count1 << " " << count2);
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "createbag");
  ros::NodeHandle nh_createbag("~");
  pu = nh_createbag.advertise<geometry_msgs::TransformStamped>("/pub", 100);
  ros::Subscriber sub1 = nh_createbag.subscribe("/tf", 100, scanCallback1);

  ros::spin();
}
