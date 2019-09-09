/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gps_common/conversions.h>
#include <string>

#include <iostream>
#include <gnss/geo_pos_conv.hpp>
#include <cmath>

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw;
static std::string _fix;
static double _origin_easting;
static double _origin_northing;
static double _map_offset_x;
static double _map_offset_y;
static double rotation_offset;

static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (msg->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting, x_rot, y_rot;
  std::string zone;

  gps_common::LLtoUTM(msg->latitude, msg->longitude, northing, easting, zone);
  x_rot = cos(rotation_offset)*(easting - _origin_easting) - sin(rotation_offset)*(northing - _origin_northing);
  y_rot = sin(rotation_offset)*(easting - _origin_easting) + cos(rotation_offset)*(northing - _origin_northing);

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;
  
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  // pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = x_rot + _map_offset_x;
  pose.pose.position.y = y_rot + _map_offset_y;
  pose.pose.position.z = msg->altitude;

 
  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                         pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  //std::cout << "distance : " << distance << std::endl;

  if (distance > 0.2)
  {
    yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    _prev_pose = pose;
  }

  pose.pose.orientation = _quat;
  pose_publisher.publish(pose);
  stat_publisher.publish(gnss_stat_msg);

  //座標変換
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  q.setRPY(0, 0, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fix2tfpose");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("fix", _fix);
  private_nh.getParam("origin_easting", _origin_easting);
  private_nh.getParam("origin_northing", _origin_northing);
  private_nh.getParam("map_offset_x", _map_offset_x);
  private_nh.getParam("map_offset_y", _map_offset_y);
  private_nh.getParam("rotation_offset", rotation_offset);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
  ros::Subscriber gnss_pose_subscriber = nh.subscribe(_fix, 100, GNSSCallback);

  ros::spin();
  return 0;
}