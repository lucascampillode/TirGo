/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef PAL_ROS_CONTROLLERS_CURRENT_LIMIT_CONTROLLER_H
#define PAL_ROS_CONTROLLERS_CURRENT_LIMIT_CONTROLLER_H

// C++ standard
#include <vector>

// Boost
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>

// ros_controls
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller.h>

// PAL-specific
#include <pal_control_msgs/ActuatorCurrentLimit.h>
#include <pal_hardware_interfaces/current_limit_interface.h>

namespace pal_ros_controllers
{

/** \brief Controller that allows to set the current limit for a group of actuators. */
class CurrentLimitController : public controller_interface::Controller<pal_ros_control::CurrentLimitInterface>
{
public:
  CurrentLimitController() : controller_interface::Controller<pal_ros_control::CurrentLimitInterface>() {}

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(pal_ros_control::CurrentLimitInterface* hw, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

private:
  typedef realtime_tools::RealtimePublisher<pal_control_msgs::ActuatorCurrentLimit> StatePublisher;
  typedef boost::scoped_ptr<StatePublisher> StatePublisherPtr;

  std::string ctrl_name_; ///< Controller name.
  std::vector<std::string> names_; ///< Names of actuators used by this controller.
  realtime_tools::RealtimeBuffer<std::vector<double> > cmd_; ///< Ordered as the list of actuator names.
  std::vector<double> null_cmd_;
  std::vector<pal_ros_control::CurrentLimitHandle> handles_; ///< Handles to controlled actuators.
  ros::Subscriber curr_lim_sub_;
  StatePublisherPtr state_pub_;

  ros::Duration state_pub_period_;
  ros::Time last_state_pub_time_; // TODO: Populate with controller uptime, not system time

  void commandCB(const pal_control_msgs::ActuatorCurrentLimitConstPtr& msg);
  void publishState(const ros::Time& time);
};

} // namespace

#endif // Header guard
