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

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

#include "aev_interface.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv)
{
  
  // ROS objects
  ros::init(argc, argv, "aev_interface");
  ros::NodeHandle nh_;
  ros::Publisher pacmod_enable_pub_;

  // Custom sigint handler for shutdown
	signal(SIGINT, mySigIntHandler);

	// Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  // Instantiate pacmod enable publisher and aev_interface
  pacmod_enable_pub_    = nh_.advertise<std_msgs::Bool>("/as_rx/enable", 10, true);
  aev::AevInterface aev_interface;

  // Enable pacmod
  std_msgs::Bool msg;
  msg.data = true;
  pacmod_enable_pub_.publish(msg);


	// Do our own spin loop
  while (!g_request_shutdown)
  {
    ros::spinOnce();
  }

	// Node is shutting down. Unlatch autopilot
	msg.data = false;	
	pacmod_enable_pub_.publish(msg);
	ros::shutdown();

  return 0;
}
