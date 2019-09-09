
## AEV Vehicle Interface

The aev interface is designed to be a set of ros packages that can be included in autoware.ai source builds to add functionality for aev development mule vehicles. Communication to the CAN bus is handled with a kvaser leaflite 2 and associated open source ROS package developed by autonomoustuff. Similarly, CAN messages are structured and sent using a Pacmod fork that has been extened to handle AEV CAN messages. This architecture allows us to rapidly test new autoware.ai functions and four-wheel steer control regimes.

### Contents

1.  aev_interface ROS package
2.  Drive by wire modes
3.  control_node ROS package

### 1. aev_interface
The `aev_interface` ROS node acts as an intermediary between actuation commands received on `vehicle_cmd` and `joy` topics, and the Pacmod (Platform Actuation and Control Module) ROS package. Pacmod is used to parse vehicle commands and safely send actuation commands over CAN through the use of a mutex object. Messages published to `vehicle_cmd` (and hence received by `aev_interface`) are Autoware driving commands, and those published on `joy` are steering angle signals generated by the drive-by-wire (DBW) steering wheel fitted to our 006 development mule. 

Switching logic to pass vehicle actuation control between DBW Autoware and DBW steering wheel is handled by the `aev/control_mode` topic published by `aev/control_node`. See the /*control_node*/ section for details. Our AEV Pacmod fork is described in the /*Pacmod AEV Vehicle*/ section.

The *aev_interface* shutdown method has been modified (overrided) so that a message can be sent between the time the node receives a shutdown signal, and the time the node shuts down. See [this ROS forum thread](https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/) for details. The effect of this modification is that actuation can be smoothly passed between mechanical link steering and DBW steering by toggling the /*AEV Interface*/ button on the Autoware Process Manager. To achieve this effect, the node is programmed to send a true messages to `/as_rx/enable` on startup. When `/as_rx/enable` receives a true message, the system is instructed to go on CAN, and when it is false (triggered on `aev_interface` shutdown), the system is told to go off CAN.

### 2. Drive by wire Modes
There are six steering modes that are implemented using a Logitech G29 steering controller and a SpaceMouse controller. Switching between steering modes is controlled by the +/- buttons on the Logitech G29 steering controller. Steering modes behaviour defined below.

#### STEERING MODE 1:     2WS - Front Steering
Normal front wheel steer. Every time a FW message is sent, RW is set to zero.

#### STEERING MODE 2:     2WS - Rear Steering
Normal rear wheel steer. Every time a FW message is sent, FW is set to zero.

#### STEERING MODE 3:     4WS - Paddle Crab Mode
Four wheel steering such that the left paddle enables crab steer when turning left only, and the right paddle commands crab steer when turning right only. Both paddles engaged enable full crab steer. Engaging only one of the paddles as the driver steers through the zero point should initiate angular momentum of the vehicle.

#### STEERING MODE 4:     4WS - Threshold Tight Mode
In threshold tight mode the turning circle of the car is decreased at the edge of the steering wheel angle travel distance (on both sides) via engaging rear steer such that the rear turns in the opposite direction. In the middle region of the steering wheel the mode is front steer only.

#### STEERING MODE 5:     4WS - Threshold Crab Mode
Threshold crab mode is the inverse of threshold tight mode such that at the extermal regions of the steering wheel travel, the rear wheels are engaged in the same direction as the front wheels. This has the effect of increasing the turning radius at the extremal regions.

#### STEERING MODE 6:     4WS - SpaceMouse Mode
Space mouse mode utilises two degrees of freedom in the space mouse. Twisting around the z axis controls the direction of the front wheels, and translating along the z axis changes the rear steering angle in relation to the front steering angle. At the zero point along the z axis, it is front steer only, push down and rear steer becomes a reflection of front steer (i.e. very tight turning circle), and pull up along the z axis and rear steer becomes a duplicate of front steer (i.e. crab steer)

### 3. control_node
`aev/control_node` is a ROS node that is toggled (launched and killed) with a button on the Autoware Process Manager. The purpose of the node is to toggle between DBW steering wheel control and DBW Autoware control. To achieve this functionality, the shutdown method has been overrided in a similar manner to that described in the `aev_interface` section, with the only difference being that boolean messages are sent to `aev/control_mode` topic instead of `/as_rx/enable` when the node is toggled. Toggle on (button down) corresponds to DBW Autoware control and toggle off (button up) corresponds to DBW steering wheel control. 

## Pacmod AEV Vehicle Fork
AEV Pacmod is a fork from Dataspeed Pacmod repo at commit [165b742bd655413dba06e335aaf0e2fde921d81e](https://github.com/astuff/pacmod/commit/165b742bd655413dba06e335aaf0e2fde921d81e). The repo was forked so that a new vehicle could be added to the package that complies with AEV requirements, and to ensure that future updates to the repo doesn't break AEV 006 dev mule functionality. Pacmod is compatible with Kvaser LeafLite 2 and a Dataspeed can interface alternative. Since we're using a Kvaser Leaf Lite 2 device, Pacmod has been set up to communicate with the Kvaser driver. See launch file for details. More details on Pacmod message structure are contained in the README at `../as/pacmod/README.md`.

### Pacmod CAN Message Structure
To simplify the actuation CAN message strucutre used to actuate the vehicle, an existing message type was hacked to include both steering wheel angle and throttle position, as opposed to sending each command separately. The message, sub

```
steer_request = msg->twist.angular.z / msg->twist.linear.x;
speed_request = msg->twist.linear.x;
```

```
void SteerCmdMsg::encode(double steer_pos, double steer_spd)
{
  data.assign(8, 0); 
  int32_t raw_pos = static_cast<int32_t>(1000.0 * steer_pos);
  uint32_t raw_spd = (uint32_t)(1000.0 * steer_spd);

  data[0] = (raw_pos & 0xFF000000) >> 24; 
  data[1] = (raw_pos & 0x00FF0000) >> 16; 
  data[2] = (raw_pos & 0x0000FF00) >> 8;
  data[3] = raw_pos & 0x000000FF;
  data[4] = (raw_spd & 0xFF000000) >> 24; 
  data[5] = (raw_spd & 0x00FF0000) >> 16; 
  data[6] = (raw_spd & 0x0000FF00) >> 8;
  data[7] = raw_spd & 0x000000FF;
}
```

## Usage Instructions

1. Click Launch Nodes
2. Click Drive By Wire. Steering control will switch to DBW steering wheel
3. Provide location estimate in Rviz (Draw 2D pose estimate on map)
4. Click Autopilot


## Notes

Information on ssc interface here https://github.com/CPFL/Autoware/pull/1945

TODO: Add a road_occupancy node to launch files as described here:
https://github.com/CPFL/Autoware/blob/master/ros/src/computing/perception/semantics/packages/road_occupancy_processor/README.md