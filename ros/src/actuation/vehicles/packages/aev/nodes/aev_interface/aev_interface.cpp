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

  front_steer_cmd_pub_        = nh_.advertise<pacmod_msgs::PositionWithSpeed>("as_rx/front_steer_cmd", 10);
  rear_steer_cmd_pub_        = nh_.advertise<pacmod_msgs::PositionWithSpeed>("as_rx/rear_steer_cmd", 10);
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
    pacmod_msgs::PositionWithSpeed front_steer_cmd;
    pacmod_msgs::PositionWithSpeed rear_steer_cmd;

    // Use linear steering ratio
    float range_scale = (STEER_OFFSET - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB;

    // Use parabolic steering ratio
    //float range_scale = fabs(msg->axes[STEERING_AXIS]) *
    //    (STEER_OFFSET - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB;


    // Get controller state
    bool right_pad = msg->buttons[4];
    bool left_pad = msg->buttons[5];
    float steering_angle = (range_scale * MAX_ROT_RAD_DEFAULT) * msg->axes[STEERING_AXIS];

    // Set front steering position
    front_steer_cmd.angular_velocity_limit = 0.0;
    front_steer_cmd.angular_position = steering_angle; 
    front_steer_cmd.header = msg->header;

    // If left crab steer enable
    if (left_pad && steering_angle > 0) 
    {
     // Set rear left steering position
     rear_steer_cmd.angular_velocity_limit = 0.0;
     rear_steer_cmd.angular_position = steering_angle; 
     rear_steer_cmd.header = msg->header;
    }

    // If right crab steer enable
    if (right_pad && steering_angle < 0) 
    {
     // Set rear right steering position
     rear_steer_cmd.angular_velocity_limit = 0.0;
     rear_steer_cmd.angular_position = steering_angle; 
     rear_steer_cmd.header = msg->header;
    }

    // Disable crab steer under certain conditions, hence set rear steer angle to zero
    if ((!right_pad && !left_pad) || // No pads engaged (normal steering)
       (left_pad && steering_angle < 0 && !right_pad) || // Left pad engaged turning right
       (right_pad && steering_angle > 0 && !left_pad)) // Right pad engaged turning left
    {
      rear_steer_cmd.angular_position = 0.0;
    }

    // Publish steer cmds
    front_steer_cmd_pub_.publish(front_steer_cmd);
    rear_steer_cmd_pub_.publish(rear_steer_cmd);
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
    pacmod_msgs::PacmodCmd speed_cmd;
    pacmod_msgs::PositionWithSpeed steer_cmd;

    // Required information: Desired wheel angle [rad], angle velocity [rad/s], throttle pos [0,1]

    // Set headers
    speed_cmd.header = msg->header;
    //brake_cmd.header = msg->header;
    steer_cmd.header = msg->header;
    
    // Send steering command
    steer_cmd.angular_position = msg->twist.angular.z / msg->twist.linear.x;
    speed_cmd.f64_cmd = msg->twist.linear.x;
    steer_cmd.angular_velocity_limit = msg->twist.linear.x;

    // Publish data
    front_steer_cmd_pub_.publish(steer_cmd);
    speed_cmd_pub_.publish(speed_cmd);
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
