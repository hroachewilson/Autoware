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
  ros::Rate loop_rate(100); // Process input at 100Hz

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
    loop_rate.sleep();
  }

	// Node is shutting down. Unlatch autopilot
	msg.data = false;	
	pacmod_enable_pub_.publish(msg);
	ros::shutdown();

  return 0;
}
