/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <map>
#include <string>

namespace teleop_twist_joy
{
/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;

  int deadman_switch;

  // bool require_turbo_button;
  // int enable_turbo_button;

  std::map<std::string, double> min_linear_map;
  std::map<std::string, double> max_linear_map;

  std::map<std::string, double> min_angular_map;
  std::map<std::string, double> max_angular_map;

  std::map<std::string, double> deadzone_linear_map;
  std::map<std::string, double> deadzone_angular_map;

  double deadzone_rotation;

  std::map<std::string, int> axis_linear_map;

  std::map<std::string, int> axis_angular_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

  nh_param->param<int>("deadman_switch", pimpl_->deadman_switch, -1);

  // nh_param->param<bool>("require_turbo_button", pimpl_->require_turbo_button, true);
  // nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (!nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
  {
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
  }

  if (!nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
  }

  if (!nh_param->getParam("deadzone_linear", pimpl_->deadzone_linear_map))
  {
    for (const auto& fieldname : pimpl_->axis_linear_map)
    {
      nh_param->param<double>("deadzone_linear", pimpl_->deadzone_linear_map[fieldname.first], 0.0);
    }
  }

  if (!nh_param->getParam("deadzone_angular", pimpl_->deadzone_angular_map))
  {
    for (const auto& fieldname : pimpl_->axis_angular_map)
    {
      nh_param->param<double>("deadzone_angular", pimpl_->deadzone_angular_map[fieldname.first], 0.0);
    }
  }

  if (!nh_param->getParam("deadzone_rotation", pimpl_->deadzone_rotation))
  {
    nh_param->param<double>("deadzone_rotation", pimpl_->deadzone_rotation, 0.0);
  }

  // ROS_INFO_NAMED("TeleopTwistJoy", "Teleop require turbo button %i.", pimpl_->require_turbo_button);
  // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo on button %i.", pimpl_->enable_turbo_button);
  ROS_INFO_COND_NAMED(pimpl_->deadman_switch >= 0, "TeleopTwistJoy", "Teleop deadman_switch %i.", pimpl_->deadman_switch);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin(); it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i", it->first.c_str(), it->second);
    // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo for linear axis %s", it->first.c_str());
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin(); it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i", it->first.c_str(), it->second);
    // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo for angular axis %s", it->first.c_str());
  }

  for (std::map<std::string, double>::iterator it = pimpl_->deadzone_linear_map.begin(); it != pimpl_->deadzone_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear deadzone of %s is %f", it->first.c_str(), it->second);
  }

  for (std::map<std::string, double>::iterator it = pimpl_->deadzone_angular_map.begin(); it != pimpl_->deadzone_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular deadzone of %s is %f", it->first.c_str(), it->second);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop deadzone rotation %f.", pimpl_->deadzone_rotation);

  pimpl_->sent_disable_msg = false;

  pimpl_->min_linear_map["x"] = -0.261;
  pimpl_->max_linear_map["x"] = 0.261;
  pimpl_->min_linear_map["y"] = -0.261;
  pimpl_->max_linear_map["y"] = 0.261;

  pimpl_->min_angular_map["roll"] = -0.3;
  pimpl_->max_angular_map["roll"] = 0.3;
  pimpl_->min_angular_map["yaw"] = -0.555;
  pimpl_->max_angular_map["yaw"] = 0.555;
}

/**
 * @return the sign of the given value
 */
int sgn(double val) { return (val < 0) - (0 < val); }

/**
 * @return zero if the given value is within the deadzone in range [-1,1]
 * else returns adjusted to the deadzone new value
 */
double ignoreDeadzone(double value, double deadzone) { return (std::abs(value) <= deadzone) ? 0.0 : value - deadzone * sgn(value); }

double updateLinearXToRotationDeadzone(double lin_x_value, double ang_z_value, double deadzone_rotation)
{
  double interpolated_deadzone_rotation = deadzone_rotation * ang_z_value;

  return (std::abs(lin_x_value) <= interpolated_deadzone_rotation) ? 0.0 : lin_x_value;
}

/**
 * @return value adjusted to the given deadzone and rescaled to a new value range according to the given min and max
 */
double rescaleValue(double value, double min, double max, double deadzone)
{
  double old_min = -1.0 - deadzone;
  double old_max = 1.0 + deadzone;

  return (max - min) * (ignoreDeadzone(value, deadzone) - old_min) / (old_max - old_min) + min;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map, const std::map<std::string, double>& deadzone_map,
              const std::string& fieldname, const std::map<std::string, double>& min_map, const std::map<std::string, double>& max_map, double deadzone_rotation = 0.0,
              double yaw = 0.0)
{
  if (axis_map.find(fieldname) == axis_map.end() || joy_msg->axes.size() <= axis_map.at(fieldname) || deadzone_map.find(fieldname) == deadzone_map.end() ||
      min_map.find(fieldname) == min_map.end() || max_map.find(fieldname) == max_map.end())
  {
    return 0.0;
  }

  double value = joy_msg->axes[axis_map.at(fieldname)];

  if (fieldname == "x")
  {
    value = updateLinearXToRotationDeadzone(value, yaw, deadzone_rotation);
  }

  return rescaleValue(value, min_map.at(fieldname), max_map.at(fieldname), deadzone_map.at(fieldname));
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, deadzone_linear_map, "x", min_linear_map, max_linear_map, deadzone_rotation, joy_msg->axes[axis_angular_map.at("yaw")]);
  cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, deadzone_linear_map, "y", min_linear_map, max_linear_map);
  cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, deadzone_linear_map, "z", min_linear_map, max_linear_map);
  // multiply the z angular velocity with the right factor for the robot to steer in the right direction
  cmd_vel_msg.angular.z = (cmd_vel_msg.linear.x < 0.0 ? 1.0 : -1.0) * getVal(joy_msg, axis_angular_map, deadzone_angular_map, "yaw", min_angular_map, max_angular_map);
  cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, deadzone_angular_map, "pitch", min_angular_map, max_angular_map);
  cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, deadzone_angular_map, "roll", min_angular_map, max_angular_map);

  cmd_vel_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  /*if (!require_turbo_button || enable_turbo_button >= 0 && joy_msg->buttons.size() > enable_turbo_button && joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
  }
  else*/
  if (deadman_switch == -1 || joy_msg->buttons.size() > deadman_switch && joy_msg->buttons[deadman_switch])
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    // When deadman switch is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      geometry_msgs::Twist cmd_vel_msg;
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy
