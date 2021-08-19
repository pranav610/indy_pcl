#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

float Y_previous = 0;
float P_previous = 0;
float R_previous = 0;
float Y_previous2 = 0;
float P_previous2 = 0;
float R_previous2 = 0;
float Y_previous3 = 0;
float P_previous3 = 0;
float R_previous3 = 0;

ros::Publisher pubb;
ros::Publisher pub2;

void scanCallback1(const geometry_msgs::Vector3::ConstPtr &msg1){
    
    geometry_msgs::Vector3 publishhh;
    geometry_msgs::Vector3 publish2;
    
    publishhh.x =(msg1->x + Y_previous + Y_previous2 + Y_previous3)/4.0 ;
    publishhh.y =(msg1->y + P_previous + P_previous2 + P_previous3)/4.0 ;
    publishhh.z =(msg1->z + R_previous + R_previous2 + R_previous3)/4.0 ;

    publish2.x =(msg1->x + Y_previous)/2.0 ;
    publish2.y =(msg1->y + P_previous)/2.0 ;
    publish2.z =(msg1->z + R_previous)/2.0 ;
    
    Y_previous3 = Y_previous2 ;
    P_previous3 = P_previous2 ;
    R_previous3 = R_previous2 ;
    
    Y_previous2 = Y_previous ; 
    P_previous2 = P_previous ;
    R_previous2 = R_previous ;
    
    Y_previous = msg1->x ;
    P_previous = msg1->y ;
    R_previous = msg1->z ;  
    
    pubb.publish(publishhh);
    pub2.publish(publish2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "average_finder");
  ros::NodeHandle nh_average_finder("~");
  ros::Subscriber sub1 = nh_average_finder.subscribe("/visYPR", 100, scanCallback1);
  pubb = nh_average_finder.advertise<geometry_msgs::Vector3>("/averaged4YPR", 100);
  pub2 = nh_average_finder.advertise<geometry_msgs::Vector3>("/averaged2YPR", 100);

  ros::spin();
}