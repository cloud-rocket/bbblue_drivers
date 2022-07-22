/**
 * @file bb-blue-macanum-motor-driver-node.cpp
 *
 *
 * @brief     macanum wheels drive control for the motor controller on Beaglbone Blue
 *            using librobotcontrol (by James Strawson).
 *            based on: https://www.robotsforroboticists.com/drive-kinematics/
 *
 * @author    Meir Tseitlin
 * @date      July 2022
 *
 * @license
 * MIT License
 *
 * Copyright (c) 2018 usxbrix
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

#include <rc/motor.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// global variables
ros::Time g_msg_received;
bool g_driving = 0;
int g_front_left_motor;      // param default 4
int g_front_right_motor;     // param default 1
int g_rear_left_motor;      // param default 3
int g_rear_right_motor;     // param default 2
double g_maxspeed;     // param default 0.4
double g_minspeed;     // param default 0.1
double g_turnspeed;    // param default 1
// double g_wheelbase;    // param default 0.2
double g_duty_factor;  // param default 2.0
int g_rate;            // param default 10
double g_wheel_radius; // param default 0.042 m
double g_wheel_left_right_distance; // param default 0.165 m
double g_wheel_front_rear_distance; // param default 0.205 m

double vx = 0;
double vy = 0;
double vth = 0;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  g_msg_received = ros::Time::now();
  ROS_INFO("cmd_vel Linear: [%f, %f, %f] Angular: [%f, %f, %f]", cmd_vel->linear.x, cmd_vel->linear.y,
           cmd_vel->linear.z, cmd_vel->angular.x, cmd_vel->angular.y, cmd_vel->angular.z);

  vx = cmd_vel->linear.x;
  vy = cmd_vel->linear.y;
  vth = cmd_vel->angular.z;

  if (vx > g_maxspeed)
  {
    ROS_INFO("Velocity %f larger than %f! Limiting speed to %f.", vx, g_maxspeed, g_maxspeed);
    vx = g_maxspeed;
  }

  /*
  https://people.cs.umu.se/thomash/reports/KinematicsEquationsForDifferentialDriveAndArticulatedSteeringUMINF-11.19.pdf
  http://robotsforroboticists.com/drive-kinematics/

  velocity_left_cmd = (linear_velocity – angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS;
  velocity_right_cmd = (linear_velocity + angular_velocity * WHEEL_BASE / 2.0)/WHEEL_RADIUS;

  self.right = 1.0 * self.dx + self.dr * self.w / 2
  self.left = 1.0 * self.dx - self.dr * self.w / 2

  */


  double wheel_separation_width = g_wheel_left_right_distance / 2;
  double wheel_separation_length = g_wheel_front_rear_distance / 2;


  double wheel_front_left = (1/g_wheel_radius) * (vx - vy - (wheel_separation_width + wheel_separation_length)*vth);
  double wheel_front_right = (1/g_wheel_radius) * (vx + vy + (wheel_separation_width + wheel_separation_length)*vth);
  double wheel_rear_left = -1 * (1/g_wheel_radius) * (vx + vy - (wheel_separation_width + wheel_separation_length)*vth);
  double wheel_rear_right = -1 * (1/g_wheel_radius) * (vx - vy + (wheel_separation_width + wheel_separation_length)*vth);

  double duty_front_left = g_duty_factor * wheel_front_left;
  double duty_front_right = g_duty_factor * wheel_front_right;
  double duty_rear_left = g_duty_factor * wheel_rear_left;
  double duty_rear_right = g_duty_factor * wheel_rear_right;


  ROS_INFO("FRONT LEFT motor: velocity:%f duty:%f \nFRONT RIGHT motor: velocity:%f duty:%f\nREAR LEFT motor: velocity:%f duty:%f \nREAR RIGHT motor: velocity:%f duty:%f\n", 
           wheel_front_left, duty_front_left, wheel_front_right, duty_front_right, wheel_rear_left, duty_rear_left, wheel_rear_right, duty_rear_right);


  rc_motor_set(g_front_left_motor, duty_front_left);
  rc_motor_set(g_front_right_motor, duty_front_right);
  rc_motor_set(g_rear_left_motor, duty_rear_left);
  rc_motor_set(g_rear_right_motor, duty_rear_right);


//   double velocity_left = (vx - vth * g_wheelbase / 2.0);
//   double velocity_right = (vx + vth * g_wheelbase / 2.0);

//   // calcaulate duty cycle form velocity and duty factor
//   double duty_left = g_duty_factor * velocity_left;
//   double duty_right = g_duty_factor * velocity_right;

//   ROS_INFO("set LEFT motor: velocity:%f duty:%f RIGHT motor: velocity:%f duty:%f", velocity_left, duty_left,
//            velocity_right, duty_right);

//   rc_motor_set(g_left_motor, duty_left);
//   rc_motor_set(g_right_motor, duty_right);
  g_driving = 1;
}

auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

int main(int argc, char** argv)
{
  // ROS and node initialize
  ros::init(argc, argv, "macanum_motor_driver");

  ros::NodeHandle n;

  ROS_INFO("Initializing node %s in namespace: %s", ros::this_node::getName().c_str(),
           ros::this_node::getNamespace().c_str());

  g_msg_received = ros::Time::now();

  // get parameter

  int cmd_vel_timeout;
  std::string base_frame_id_;

  // n.param("~timeout", cmd_vel_timeout, 5);
  ros::param::param("~timeout", cmd_vel_timeout, 5);
  ros::param::param("~front_left_motor", g_front_left_motor, 1);
  ros::param::param("~front_right_motor", g_front_right_motor, 2);
  ros::param::param("~rear_left_motor", g_rear_left_motor, 1);
  ros::param::param("~rear_right_motor", g_rear_right_motor, 2);
  ros::param::param("~maxspeed", g_maxspeed, 0.4);
  ros::param::param("~minspeed", g_minspeed, 0.1);
  // ros::param::param("~wheelbase", g_wheelbase, 0.2);
  ros::param::param("~turnspeed", g_turnspeed, 1.0);
  ros::param::param("~duty_factor", g_duty_factor, 1.0);
  ros::param::param("~rate", g_rate, 10);
  ros::param::param("~wheel_radius", g_wheel_radius, 0.042);
  ros::param::param("~wheel_left_right_distance", g_wheel_left_right_distance, 0.165);
  ros::param::param("~wheel_front_rear_distance", g_wheel_front_rear_distance, 0.205);

  if (g_front_left_motor < 1 or g_front_left_motor > 4 or 
        g_front_right_motor < 1 or g_front_right_motor > 4 or
        g_rear_left_motor < 1 or g_rear_left_motor > 4 or 
        g_rear_right_motor < 1 or g_rear_right_motor > 4)
  {
    ROS_ERROR("ERROR: Wrong parameter: motors must be between 1-4");
    return -1;
  }

  n.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  // TODO: motor parameter

  // initialize motor hardware first
  int pwm_freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;  // 25000
  if (rc_motor_init_freq(pwm_freq_hz))
  {
    ROS_ERROR("Initialize motors %d/%d/%d/%d with %d: FAILED", g_front_left_motor, g_front_right_motor, g_rear_left_motor, g_rear_right_motor, pwm_freq_hz);
    return -1;
  }
  ROS_INFO("Initialize motors %d/%d/%d/%d with %d: OK", g_front_left_motor, g_front_right_motor, g_rear_left_motor, g_rear_right_motor, pwm_freq_hz);

  // odometry publisher
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf2_ros::TransformBroadcaster odom_broadcaster;

  // cmd_vel subscriber 
  ros::Subscriber sub = n.subscribe("cmd_vel", 100, cmd_velCallback);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  // %EndTag(SUBSCRIBER)%

  ROS_INFO("Node is up and Subsciber started");

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  // %Tag(SPIN)%

  ros::Rate r(g_rate);

  while (ros::ok())
  {
    ros::spinOnce();

    current_time = ros::Time::now();

    // stopping motor when no message received within timeout

    if (g_driving && (ros::Time::now().toSec() - g_msg_received.toSec()) > cmd_vel_timeout)

    {
      ROS_INFO("TIMEOUT: No cmd_vel received: setting motors to 0");

      // looks like 0 for all motors doesn't work
      // rc_motor_set(0,0);
      rc_motor_set(g_front_left_motor, 0);
      rc_motor_set(g_front_right_motor, 0);
      rc_motor_set(g_rear_left_motor, 0);
      rc_motor_set(g_rear_right_motor, 0);

      g_driving = 0;
    }

    // compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = createQuaternionMsgFromYaw(th);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    // publish the message
    odom_pub.publish(odom);

    last_time = current_time;

    r.sleep();
  }

  // %EndTag(SPIN)%

  // close motor hardware
  ROS_INFO("Calling rc_motor_cleanup()");
  rc_motor_cleanup();
  return 0;
}
// %EndTag(FULLTEXT)%
