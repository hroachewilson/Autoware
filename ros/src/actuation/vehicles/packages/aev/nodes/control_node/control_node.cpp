#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

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
  ros::init(argc, argv, "aev/control_node", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  ros::NodeHandle nh_;
  ros::Publisher aev_control_mode_pub;
  ros::Rate loop_rate(10); // Publish control_mode at 10Hz

  std_msgs::Bool msg;
  msg.data = true;

  // Instantiate control node publisher
  
  aev_control_mode_pub      = nh_.advertise<std_msgs::Bool>("/aev/control_mode", 10, true);
  aev_control_mode_pub.publish(msg);

  // Do our own spin loop
  while (!g_request_shutdown)
  {
    // Do non-callback stuff
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Node is shutting down. Unlatch autopilot
  msg.data = false;	
  aev_control_mode_pub.publish(msg);
  ros::shutdown();

  return 0;
}
