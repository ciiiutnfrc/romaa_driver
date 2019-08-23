#include "ros/ros.h"

#include <nav_msgs/Odometry.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "romaa");
  ros::NodeHandle nh("~"); // private names

  ROS_INFO("Opening RoMAA communication.");
  // Open robot communication

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);

  // Message variables
  nav_msgs::Odometry odom_msg;

  ros::Time current_time;
  ros::Rate loop_rate(10);// 10 Hz
  while(ros::ok())
  {
    current_time = ros::Time::now();

    // Odometry message
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";

    // Set odometry pose...

		//publish the message
		odom_pub.publish(odom_msg);

		ros::spinOnce();
		loop_rate.sleep();
  }

  return 0;
}
