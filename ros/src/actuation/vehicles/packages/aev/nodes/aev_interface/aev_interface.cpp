/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "aev_interface.h"

namespace aev
{
// Constructor
AevInterface::AevInterface() :
    private_nh_("~"),
    control_mode_(false)
{
  initForROS();
}

// Destructor
AevInterface::~AevInterface()
{
}

void AevInterface::initForROS()
{
  // ros parameter settings
  private_nh_.param<double>("acceleration_limit", acceleration_limit_, 3.0);
  private_nh_.param<double>("deceleration_limit", deceleration_limit_, 3.0);
  private_nh_.param<double>("max_curvature_rate", max_curvature_rate_, 0.75);

  // setup subscriber
  twist_cmd_sub_    = nh_.subscribe("twist_cmd", 10, &AevInterface::callbackFromTwistCmd, this);
  control_mode_sub_ = nh_.subscribe("/aev/control_mode", 10, &AevInterface::callbackFromControlMode, this);
  speed_sub_        = nh_.subscribe("/vehicle/steering_report", 10, &AevInterface::callbackFromSteeringReport, this);

  // setup publisher
  //steer_mode_pub_    = nh_.advertise<automotive_platform_msgs::SteerMode>("/aev/arbitrated_steering_commands", 10);
  //speed_mode_pub_    = nh_.advertise<automotive_platform_msgs::SpeedMode>("/aev/arbitrated_speed_commands", 10);
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("aev_current_twist", 10);

  steer_cmd_pub_    = nh_.advertise<pacmod_msgs::PositionWithSpeed>("as_rx/steer_cmd", 10);
  speed_cmd_pub_    = nh_.advertise<pacmod_msgs::PacmodCmd>("as_rx/speed_cmd", 10);
  brake_cmd_pub_    = nh_.advertise<pacmod_msgs::PacmodCmd>("as_rx/brake_cmd", 10);
}

void AevInterface::run()
{
  ros::spin();
}


void AevInterface::callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr &msg)
{
  int mode;
  //double curvature = msg->twist.angular.z / msg->twist.linear.x;
  float acceleration_limit = 3.0;
  float deceleration_limit = 3.0;
  if (control_mode_)
  {
    mode = 1;
  }
  else
  {
    mode = 0;
  }
  pacmod_msgs::PacmodCmd brake_cmd;
  pacmod_msgs::PacmodCmd speed_cmd;
  pacmod_msgs::PositionWithSpeed steer_cmd;

  // Required information: Desired wheel angle [rad], angle velocity [rad/s], throttle pos [0,1]

  // Set headers
  speed_cmd.header = msg->header;
  steer_cmd.header = msg->header;
  brake_cmd.header = msg->header;
  
  // Implement a hack such that desired steer angle and vehicle velocity stored in speed_cmd
  steer_cmd.angular_position = msg->twist.angular.z / msg->twist.linear.x;
  steer_cmd.angular_velocity_limit = msg->twist.linear.x;

  // Print status
  std::cout << "mode: "  << mode << std::endl;
  std::cout << "speed: " << steer_cmd.angular_velocity_limit << std::endl;
  std::cout << "steer: " << steer_cmd.angular_position << std::endl;

  // Publish data
  steer_cmd_pub_.publish(steer_cmd);
}

void AevInterface::callbackFromControlMode(const std_msgs::BoolConstPtr &msg)
{
  control_mode_ = msg->data;
}

void AevInterface::callbackFromSteeringReport(const dbw_mkz_msgs::SteeringReportConstPtr &msg)
{
  geometry_msgs::TwistStamped ts;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  ts.header = header;
  ts.twist.linear.x = msg->speed; // [m/sec]
  // Can we get angular velocity?

  current_twist_pub_.publish(ts);
}

}  // aev
