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
  spacenav_twist_sub_   = nh_.subscribe("/spacenav/twist", 1000, &AevInterface::callbackFromSpacenavTwist, this);

  // setup publisher
  //steer_mode_pub_     = nh_.advertise<automotive_platform_msgs::SteerMode>("/aev/arbitrated_steering_commands", 10);
  //speed_mode_pub_     = nh_.advertise<automotive_platform_msgs::SpeedMode>("/aev/arbitrated_speed_commands", 10);
  current_twist_pub_    = nh_.advertise<geometry_msgs::TwistStamped>("aev_current_twist", 10);

  front_steer_cmd_pub_  = nh_.advertise<pacmod_msgs::PositionWithSpeed>("as_rx/front_steer_cmd", 10);
  rear_steer_cmd_pub_   = nh_.advertise<pacmod_msgs::PositionWithSpeed>("as_rx/rear_steer_cmd", 10);
  speed_cmd_pub_        = nh_.advertise<pacmod_msgs::PacmodCmd>("as_rx/speed_cmd", 10);
  brake_cmd_pub_        = nh_.advertise<pacmod_msgs::PacmodCmd>("as_rx/brake_cmd", 10);

  steer_mode_ = 0;
  std::cout << "STEERING MODE 1:     2WS - Front Steering" << std::endl;
}

void AevInterface::run()
{
  // Ros spin
  ros::spin();
}


/*
 * Called when a spacenav joy message is received
 */
void AevInterface::callbackFromSpacenavTwist(const geometry_msgs::Twist::ConstPtr& msg)
{

  if (!control_mode_ && steer_mode_ == 7)
  {

    // Steering command messages
    pacmod_msgs::PositionWithSpeed front_steer_cmd;
    pacmod_msgs::PositionWithSpeed rear_steer_cmd;
    front_steer_cmd.angular_position = msg->angular.z;
    rear_steer_cmd.angular_position = msg->angular.z * (msg->linear.z * 1.333333);

    // Pulish cmds
    front_steer_cmd_pub_.publish(front_steer_cmd);
    rear_steer_cmd_pub_.publish(rear_steer_cmd);
    //std::cout << "front: " << front_steer_cmd.angular_position << std::endl;
    //std::cout << "rear: " << rear_steer_cmd.angular_position << std::endl;
  }
}


/*
 * Called when a game controller message is received
 */
void AevInterface::callbackFromJoy(const sensor_msgs::Joy::ConstPtr& msg)
{

  if (!control_mode_)
  {
    // Steering command messages
    pacmod_msgs::PositionWithSpeed front_steer_cmd;
    pacmod_msgs::PositionWithSpeed rear_steer_cmd;
    front_steer_cmd.header = msg->header;
    rear_steer_cmd.header = msg->header;

    // Notify the user of steering mode and update mode select button
    if (msg->buttons[19] || msg->buttons[20])
    {

      if (msg->buttons[19])
      {
        steer_mode_ += 1;
        steer_mode_ %= 8;
      } 
      else
      {
        steer_mode_ -= 1;
        if (steer_mode_ < 0)
        {
          steer_mode_ = 7;
        }
      }
      
      if (steer_mode_ == 0)
      {
        std::cout << "STEERING MODE 1:     2WS - Front Steering" << std::endl;
      } 
      else if (steer_mode_ == 1 )
      {
        std::cout << "STEERING MODE 2:     2WS - Rear Steering" << std::endl;
      }
      else if (steer_mode_ == 2 )
      {
        std::cout << "STEERING MODE 3:     4WS - Diamond Steer Mode" << std::endl;
      }
      else if (steer_mode_ == 3 )
      {
        std::cout << "STEERING MODE 4:     4WS - Crab Steer Mode" << std::endl;
      }
      else if (steer_mode_ == 4 )
      {
        std::cout << "STEERING MODE 5:     4WS - Threshold Crab Mode" << std::endl;
      }
      else if (steer_mode_ == 5 ) 
      {
        std::cout << "STEERING MODE 6:     4WS - Threshold Tight Mode" << std::endl;
      }
      else if (steer_mode_ == 6 ) 
      {
        std::cout << "STEERING MODE 7:     4WS - Threshold Crab Mode" << std::endl;
      }
      else 
      {
        std::cout << "STEERING MODE 8:     4WS - SpaceMouse Mode" << std::endl;
      }
    }

    // Get steering angle
    float steering_angle = MAX_ROT_RAD_DEFAULT * msg->axes[STEERING_AXIS];

    if (steer_mode_ == 0) // Two wheel front steer
    {

      // Set steering positions
      front_steer_cmd.angular_position = steering_angle; 
      rear_steer_cmd.angular_position = 0.0;

    } 
    else if (steer_mode_ == 1 ) // Two wheel rear steer
    {

      // Set steering positions
      front_steer_cmd.angular_position = 0.0; 
      rear_steer_cmd.angular_position = steering_angle;

    }
    else if (steer_mode_ == 2 ) // Four wheel diamond steer
    {
      // Set steering positions
      front_steer_cmd.angular_position = steering_angle; 
      rear_steer_cmd.angular_position = -steering_angle;
    }
    else if (steer_mode_ == 3 ) // Four wheel crab steer
    {
      // Set steering positions
      front_steer_cmd.angular_position = steering_angle; 
      rear_steer_cmd.angular_position = steering_angle;
    }
    else if (steer_mode_ == 4 ) // Four wheel paddle crab
    {

      // Get controller state
      bool right_pad = msg->buttons[4];
      bool left_pad = msg->buttons[5];

      // Set front steering position
      front_steer_cmd.angular_position = steering_angle; 

      // If left crab steer enable
      if (left_pad && steering_angle > 0) 
      {
       // Set rear left steering position
       rear_steer_cmd.angular_position = steering_angle; 
      }

      // If right crab steer enable
      if (right_pad && steering_angle < 0) 
      {
       // Set rear right steering position
       rear_steer_cmd.angular_position = steering_angle; 
      }

      // Disable crab steer under following conditions, hence set rear steer angle to zero
      if ((!right_pad && !left_pad) || // No pads engaged (normal steering)
         (left_pad && steering_angle < 0 && !right_pad) || // Left pad engaged turning right
         (right_pad && steering_angle > 0 && !left_pad)) // Right pad engaged turning left
      {
        rear_steer_cmd.angular_position = 0.0;
      }
    }
    else if (steer_mode_ == 5) //Threshold inverted mode
    {
      front_steer_cmd.angular_position = steering_angle; 

      if (steering_angle > RANGE_THRESH)
      {
       rear_steer_cmd.angular_position = -(steering_angle - RANGE_THRESH) * RANGE_SCALE;
      }
      else if (steering_angle < -RANGE_THRESH)
      {
        rear_steer_cmd.angular_position = -(steering_angle + RANGE_THRESH) * RANGE_SCALE;
      }
      else 
      {
        rear_steer_cmd.angular_position = 0.0;
      }
    }
    else if (steer_mode_ == 6) //Threshold crab mode
    {
      front_steer_cmd.angular_position = steering_angle; 

      if (steering_angle > RANGE_THRESH)
      {
       rear_steer_cmd.angular_position = (steering_angle - RANGE_THRESH) * RANGE_SCALE;
      }
      else if (steering_angle < -RANGE_THRESH)
      {
        rear_steer_cmd.angular_position = (steering_angle + RANGE_THRESH) * RANGE_SCALE;
      }
      else 
      {
        rear_steer_cmd.angular_position = 0.0;
      }
    }
    else {} //Spacemouse mode

    // Publish steer cmds unless we're in spacemouse mode
    if (steer_mode_ != 7)
    {
      front_steer_cmd_pub_.publish(front_steer_cmd);
      rear_steer_cmd_pub_.publish(rear_steer_cmd);
      //std::cout << "front: " << front_steer_cmd.angular_position << std::endl;
      //std::cout << "rear: " << rear_steer_cmd.angular_position << std::endl;
    }
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
