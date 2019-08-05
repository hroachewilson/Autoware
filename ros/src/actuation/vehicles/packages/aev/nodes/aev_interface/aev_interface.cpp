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
  twist_cmd_sub_        = nh_.subscribe("twist_cmd", 10, &AevInterface::callbackFromTwistCmd, this);
  control_mode_sub_     = nh_.subscribe("/aev/control_mode", 10, &AevInterface::callbackFromControlMode, this);
  speed_sub_            = nh_.subscribe("/vehicle/steering_report", 10, &AevInterface::callbackFromSteeringReport, this);
  joy_sub_              = nh_.subscribe("joy", 1000, &AevInterface::callbackFromJoy, this);

  // setup publisher
  //steer_mode_pub_     = nh_.advertise<automotive_platform_msgs::SteerMode>("/aev/arbitrated_steering_commands", 10);
  //speed_mode_pub_     = nh_.advertise<automotive_platform_msgs::SpeedMode>("/aev/arbitrated_speed_commands", 10);
  current_twist_pub_    = nh_.advertise<geometry_msgs::TwistStamped>("aev_current_twist", 10);

  steer_cmd_pub_        = nh_.advertise<pacmod_msgs::PositionWithSpeed>("as_rx/steer_cmd", 10);
  speed_cmd_pub_        = nh_.advertise<pacmod_msgs::PacmodCmd>("as_rx/speed_cmd", 10);
  brake_cmd_pub_        = nh_.advertise<pacmod_msgs::PacmodCmd>("as_rx/brake_cmd", 10);
}

void AevInterface::run()
{
  // Ros spin
  ros::spin();
}

/*
 * Called when a game controller message is received
 */
void AevInterface::callbackFromJoy(const sensor_msgs::Joy::ConstPtr& msg)
{

  if (!control_mode_)
  {
    // Calculate steering angle from joystick msg
    pacmod_msgs::PositionWithSpeed steer_cmd;
    float range_scale = fabs(msg->axes[STEERING_AXIS]) *
        (STEER_OFFSET - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB;

    // Set steering position
    steer_cmd.angular_velocity_limit = 0.0;
    steer_cmd.angular_position = (range_scale * MAX_ROT_RAD_DEFAULT) * msg->axes[STEERING_AXIS];
    steer_cmd.header = msg->header;

    // Print status
    //std::cout << "mode: "  << control_mode_ << std::endl;
    //std::cout << "speed: " << steer_cmd.angular_velocity_limit << std::endl;
    //std::cout << "steer: " << steer_cmd.angular_position << std::endl;

    // Publish data
    steer_cmd_pub_.publish(steer_cmd);
  }
}

void AevInterface::callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr &msg)
{
  if (control_mode_)
  {
    //double curvature = msg->twist.angular.z / msg->twist.linear.x;
    //float acceleration_limit = 3.0;
    //float deceleration_limit = 3.0;
    //pacmod_msgs::PacmodCmd brake_cmd;
    //pacmod_msgs::PacmodCmd speed_cmd;
    pacmod_msgs::PositionWithSpeed steer_cmd;

    // Required information: Desired wheel angle [rad], angle velocity [rad/s], throttle pos [0,1]

    // Set headers
    //speed_cmd.header = msg->header;
    //brake_cmd.header = msg->header;
    steer_cmd.header = msg->header;
    
    // Implement a hack such that desired steer angle and vehicle velocity stored in speed_cmd
    steer_cmd.angular_position = msg->twist.angular.z / msg->twist.linear.x;
    steer_cmd.angular_velocity_limit = msg->twist.linear.x;

    // Print status
    //std::cout << "mode: "  << control_mode_ << std::endl;
    //std::cout << "speed: " << steer_cmd.angular_velocity_limit << std::endl;
    //std::cout << "steer: " << steer_cmd.angular_position << std::endl;

    // Publish data
    steer_cmd_pub_.publish(steer_cmd);
  }
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
