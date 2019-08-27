#include "ros/ros.h"

// Messages headers.
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// tf2 library headers.
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>//toMsg

// The srv class for the services.
#include <std_srvs/Empty.h> 
#include <std_srvs/SetBool.h>
#include <romaa_ros/SetOdometry.h>

#include <romaa_comm/romaa_comm.h>
romaa_comm *romaa;

const std::string def_port = "/dev/ttyUSB0";
const int def_baud = 115200;

// Callback function definitions.
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& );
bool resetOdometrySrvCb(std_srvs::Empty::Request &,
                        std_srvs::Empty::Response &);
bool setOdometrySrvCb(romaa_ros::SetOdometry::Request &,
                      romaa_ros::SetOdometry::Response &);
bool enableMotorSrvCb(std_srvs::SetBool::Request &,
                      std_srvs::SetBool::Response &);

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

  // Publisher and subscriber objects.
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel",\
      1, cmdVelCallback);

  // Services objects.
  ros::ServiceServer reset_odom_srv = nh.advertiseService("reset_odometry", 
      &resetOdometrySrvCb);
  ros::ServiceServer set_odom_srv = nh.advertiseService("set_odometry", 
      &setOdometrySrvCb);
  ros::ServiceServer motorsrv = nh.advertiseService("enable_motor", 
      &enableMotorSrvCb);

  // Odometry variables
  float x, y, a, v, w;
  nav_msgs::Odometry odom_msg;

  // tf2 variables.
  tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped odom_tf;
  tf2::Quaternion odom_quat_tf;

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

    // tf2 orientation. Yaw angle to quaternion.
    odom_quat_tf.setRPY(0, 0, a);

    // Odometry message
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    geometry_msgs::Quaternion odom_quat_msg = tf2::toMsg(odom_quat_tf);

    // Set the position.
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation = odom_quat_msg;

    // Set the velocity (in the child frame).
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = w;

		//publish the message
		odom_pub.publish(odom_msg);

    // tf2.
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    odom_tf.transform.translation.x = x;        // X
    odom_tf.transform.translation.y = y;        // Y
    odom_tf.transform.translation.z = 0.0;      // Z
    odom_tf.transform.rotation = odom_quat_msg; // Q

    // Send the transform.
    tf_broadcaster.sendTransform(odom_tf);

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

// 'cmd_vel' subscriber callback function.
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  romaa->set_speed(cmd_vel->linear.x, cmd_vel->angular.z);
  ROS_DEBUG_STREAM("Setting speed. v: " << cmd_vel->linear.x 
      << ", w: " << cmd_vel->angular.z);
}

// 'reset_odometry' services callback function.
bool resetOdometrySrvCb(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp)
{
  romaa->reset_odometry();
  ROS_DEBUG("Reset odometry.");
  return true;
}

// 'set_odometry' services callback function.
bool setOdometrySrvCb(romaa_ros::SetOdometry::Request &req,
    romaa_ros::SetOdometry::Response &resp)
{
  romaa->set_odometry(req.x, req.y, req.theta);
  ROS_DEBUG_STREAM("Setting odometry to (" << req.x << ", "
      << req.y << ", " << req.theta << ")");
  return true;
}

// 'enable_motor' services callback function.
bool enableMotorSrvCb(std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &resp)
{
  if(req.data == true)
  {
    ROS_DEBUG("Enable motor");
    romaa->enable_motor();
    resp.success = true;
    resp.message = "Motor enabled.";
  }
  else
  {
    ROS_DEBUG("Disable motor");
    romaa->disable_motor();
    resp.success = true;
    resp.message = "Motor disabled.";
  }
  return true;
}

