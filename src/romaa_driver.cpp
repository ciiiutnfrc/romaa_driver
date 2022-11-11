#include "ros/ros.h"

// Messages headers.
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// tf2 library headers.
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>            // tf2::Quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  //toMsg

// The srv class for the services.
#include <std_srvs/Empty.h> 
#include <std_srvs/SetBool.h>
#include <romaa_driver/SetOdometry.h>
#include <romaa_driver/SetPid.h>

#include <romaa_comm/romaa_comm.h>
romaa_comm *romaa;

// Callback function definitions.
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& );
bool resetOdometrySrvCb(std_srvs::Empty::Request & ,
        std_srvs::Empty::Response & );
bool setOdometrySrvCb(romaa_driver::SetOdometry::Request & ,
        romaa_driver::SetOdometry::Response & );
bool enableMotorSrvCb(std_srvs::SetBool::Request & ,
        std_srvs::SetBool::Response & );
bool resetSrvCb(std_srvs::Empty::Request & ,
        std_srvs::Empty::Response & );
bool setLinearSpeedSrvCb(romaa_driver::SetPid::Request & ,
        romaa_driver::SetPid::Response & );
bool setAngularSpeedSrvCb(romaa_driver::SetPid::Request & ,
        romaa_driver::SetPid::Response & );

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "romaa_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Node parameters.
  float frequency;
  bool reset_odom, enable_motor;
  std::string port;
  int baud;
  float wheelbase, wheel_radius;
  pnh.param<float>("frequency", frequency, 10.0);
  pnh.param<std::string>("port", port, "/dev/ttyUSB0");
  pnh.param<int>("baud", baud, 115200);
  pnh.param<bool>("reset_odom", reset_odom, true);
  pnh.param<bool>("enable_motor", enable_motor, false);
  pnh.param<float>("wheelbase", wheelbase, 0.44);
  pnh.param<float>("wheel_radius", wheel_radius, 0.075);

  // PID controller variables.
  float v_pid_kp, v_pid_ki, v_pid_kd;
  float w_pid_kp, w_pid_ki, w_pid_kd;
  pnh.param<float>("v_pid_kp", v_pid_kp, 1800.0);
  pnh.param<float>("v_pid_ki", v_pid_ki, 100.0);
  pnh.param<float>("v_pid_kd", v_pid_kd, 10.0);
  pnh.param<float>("w_pid_kp", w_pid_kp, 2000.0);
  pnh.param<float>("w_pid_ki", w_pid_ki, 150.0);
  pnh.param<float>("w_pid_kd", w_pid_kd, 20.0);

  // Open robot communication
  ROS_INFO("Opening RoMAA communication in %s at %d", port.c_str(), baud);
  romaa = new romaa_comm(port.c_str(), baud);
  
  if(romaa->is_connected() == true)
    ROS_INFO("Connected to RoMAA.");
  else
  {
    ROS_FATAL("Could not connect to RoMAA.");
    ROS_BREAK();
  }

  if(enable_motor == true)
  {
    romaa->enable_motor();
    ROS_WARN("Enable motors.");
  }
  else
  {
    romaa->disable_motor();
    ROS_WARN("Disable motors.");
  }

  // Setting embedded parameters.
  ROS_INFO("Setting PIDs parameters.");
  romaa->set_v_pid(v_pid_kp, v_pid_ki, v_pid_kd);
  romaa->set_w_pid(w_pid_kp, w_pid_ki, w_pid_kd);
  ROS_INFO("Setting kinematic parameters.");
  romaa->set_kinematic_params(wheelbase, wheel_radius);

  usleep(500000);
  ROS_INFO("Reading parameters:");
  romaa->get_v_pid(v_pid_kp, v_pid_ki, v_pid_kd);
  romaa->get_w_pid(w_pid_kp, w_pid_ki, w_pid_kd);
  ROS_INFO_STREAM("Linear speed PID (Kp, Ki, Kd): " <<
      v_pid_kp << ", " << v_pid_ki << ", " << v_pid_kd);
  ROS_INFO_STREAM("Linear angular PID (Kp, Ki, Kd): " <<
      w_pid_kp << ", " << w_pid_ki << ", " << w_pid_kd);
  romaa->get_kinematic_params(wheelbase, wheel_radius);
  ROS_INFO("Kinematic parameters.");
  ROS_INFO_STREAM(" Wheelbase: " << wheelbase);
  ROS_INFO_STREAM(" Left wheel radious: " << wheel_radius);

  if(reset_odom == true)
  {
    romaa->reset_odometry();
    ROS_INFO("Reset odometry.");
  }

  // Publisher and subscriber objects.
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel",\
      1, cmdVelCallback);

  // Services objects.
  ros::ServiceServer reset_odom_srv = nh.advertiseService("reset_odometry", 
          &resetOdometrySrvCb);
  ros::ServiceServer set_odom_srv = nh.advertiseService("set_odometry", 
          &setOdometrySrvCb);
  ros::ServiceServer motorsrv = nh.advertiseService("enable_motor", 
          &enableMotorSrvCb);
  ros::ServiceServer server4 = nh.advertiseService("reset", &resetSrvCb);
  ros::ServiceServer set_v_pid_srv = nh.advertiseService("set_linear_speed_pid",
          &setLinearSpeedSrvCb);
  ros::ServiceServer set_w_pid_srv = nh.advertiseService("set_angular_speed_pid",
          &setAngularSpeedSrvCb);

  // Odometry variables.
  float x, y, a, v, w;
  nav_msgs::Odometry odom_msg;

  // tf2 variables.
  tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped odom_tf;
  geometry_msgs::Quaternion odom_quat_msg;
  tf2::Quaternion odom_quat_tf;

  ros::Time current_time;
  ros::Rate loop_rate(frequency);
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
    odom_quat_msg = tf2::toMsg(odom_quat_tf);

    // Set the position.
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation = odom_quat_msg;

    // Set the velocity (in the child frame).
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = w;

    // Publish the message.
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

    // Processes incoming messages via callbacks.
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
  ROS_DEBUG("Setting speed. v: %3f, w: %3f.", cmd_vel->linear.x, 
          cmd_vel->angular.z);
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
bool setOdometrySrvCb(romaa_driver::SetOdometry::Request &req,
    romaa_driver::SetOdometry::Response &resp)
{
  romaa->set_odometry(req.x, req.y, req.theta);
  ROS_DEBUG("Setting odometry to (%.3f, %.3f, %.3f)", req.x,
          req.y, req.theta);
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

bool resetSrvCb(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp)
{
  romaa->reset();
  ROS_DEBUG("Reset embedded controller...");
  return true;
}

bool setLinearSpeedSrvCb(romaa_driver::SetPid::Request &req,
    romaa_driver::SetPid::Response &resp)
{
  romaa->set_v_pid(req.kp, req.ki, req.kd);
  ROS_DEBUG("Setting linear speed PID to (%.3f, %.3f, %.3f)", req.kp,
          req.ki, req.kd);
  return true;
}

bool setAngularSpeedSrvCb(romaa_driver::SetPid::Request &req,
    romaa_driver::SetPid::Response &resp)
{
  romaa->set_w_pid(req.kp, req.ki, req.kd);
  ROS_DEBUG("Setting angular speed PID to (%.3f, %.3f, %.3f)", req.kp,
          req.ki, req.kd);
  return true;
}
