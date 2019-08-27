#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <romaa_comm/romaa_comm.h>
romaa_comm *romaa;

const std::string def_port = "/dev/ttyUSB0";
const int def_baud = 115200;

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "romaa");
  ros::NodeHandle nh("~"); // private names

  // Communication port variables
  std::string port;
  int baud;
  nh.param<std::string>("port", port, def_port);
  nh.param<int>("baud", baud, def_baud);

  // Open robot communication
  ROS_INFO_STREAM("Opening RoMAA communication in " << port << " at " << baud);
  romaa = new romaa_comm(port.c_str(), baud);
  
  if(romaa->is_connected() == true)
    ROS_INFO("Connected to RoMAA.");
  else
  {
    ROS_FATAL("Could not connect to RoMAA.");
    ROS_BREAK();
  }
  romaa->enable_motor();

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);

  // Odometry variables
  float x, y, a, v, w;
  nav_msgs::Odometry odom_msg;

  ros::Time current_time;
  ros::Rate loop_rate(10);// 10 Hz
  while(ros::ok())
  {
    current_time = ros::Time::now();
    
    // Read odometry and speed from RoMAA robot.
    if( romaa->get_odometry(x, y, a) == -1 )
      ROS_WARN("Unable to read odometry.");
    usleep(100000);
    if( romaa->get_speed(v, w) == -1 )
      ROS_WARN("Unable to read speed.");

    // Odometry message
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(a);

    // Set the position.
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation = odom_quat;

    // Set the velocity (in the child frame).
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = w;

		//publish the message
		odom_pub.publish(odom_msg);

		ros::spinOnce();
		loop_rate.sleep();
  }

  ros::shutdown();
  romaa->set_speed(0.0, 0.0);
  usleep(100000);
  romaa->disable_motor();
  delete romaa;
  return 0;
}
