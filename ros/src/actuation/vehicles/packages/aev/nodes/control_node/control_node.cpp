#include <ros/ros.h>
#include <std_msgs/Bool.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "aev/control_node");

	ros::NodeHandle nh_;
	std_msgs::Bool msg;
	ros::Publisher control_mode_pub;

	msg.data = true;

  // Control node publisher
  control_mode_pub    = nh_.advertise<std_msgs::Bool>("/aev/control_mode", 10, true);
	control_mode_pub.publish(msg);

	// Wait until node is shutdown before sending false
	while(ros::ok());

	// Node is shutting down. Unlatch autopilot
	msg.data = false;
	control_mode_pub.publish(msg);

  return 0;
}
