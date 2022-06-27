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

  bool require_enable_button;
  bool require_turbo_button;
  int enable_button;
  int enable_turbo_button;

  double min_lin;
  double max_lin;
  double min_ang;
  double max_ang;
  double min_ICC;
  double max_ICC;

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

  nh_param->param<bool>("require_enable_button", pimpl_->require_enable_button, false);
  nh_param->param<bool>("require_turbo_button", pimpl_->require_turbo_button, true);
  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (!nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
  {
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
  }

  if (!nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop require turbo button %i.", pimpl_->require_turbo_button);
  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop require enable button %i.", pimpl_->require_enable_button);
  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin(); it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i", it->first.c_str(), it->second);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo for linear axis %s", it->first.c_str());
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin(); it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i", it->first.c_str(), it->second);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo for angular axis %s", it->first.c_str());
  }

  pimpl_->sent_disable_msg = false;

  pimpl_->min_lin = -0.05151;
  pimpl_->max_lin = 0.05151;
  pimpl_->min_ang = -0.11;
  pimpl_->max_ang = 0.11;
  pimpl_->min_ICC = -1.0;
  pimpl_->max_ICC = 1.0;
}

double rescaleValue(double value, double min, double max) { return (max - min) * (value + 1) / 2 + min; }

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map, const std::string& fieldname, double min, double max)
{
  if (axis_map.find(fieldname) == axis_map.end() || joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return rescaleValue(joy_msg->axes[axis_map.at(fieldname)], min, max);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, "x", min_lin, max_lin);
  cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, "y", min_lin, max_lin);
  cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, "z", min_lin, max_lin);
  // multiply the z angular velocity with the right factor for the robot to steer in the right direction
  int sign = -1;
  if (cmd_vel_msg.linear.x < 0)
  {
    sign = 1;
  }
  cmd_vel_msg.angular.z = sign * getVal(joy_msg, axis_angular_map, "yaw", min_ang, max_ang);
  cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, "pitch", min_ang, max_ang);
  cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, "roll", min_ICC, max_ICC);

  cmd_vel_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (!require_turbo_button || enable_turbo_button >= 0 && joy_msg->buttons.size() > enable_turbo_button && joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
  }
  else if (!require_enable_button || joy_msg->buttons.size() > enable_button && joy_msg->buttons[enable_button])
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
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
