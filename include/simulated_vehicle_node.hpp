/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Thomas Lobig
 *    Mikkel Skov Maarssø
 *    Sanath Himasekhar Konthala
 *    Marko Mizdrak
 ********************************************************************************/

#pragma once

#include <chrono>
#include <map>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "adore_dynamics_conversions.hpp"
#include "adore_ros2_msgs/msg/gear_state.hpp"
#include "adore_ros2_msgs/msg/state_monitor.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/vehicle_command.hpp"

#include "dynamics/OdeRK4.hpp"
#include "dynamics/integration.hpp"
#include "dynamics/vehicle_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace adore
{
namespace simulated_vehicle
{
class SimulatedVehicleNode : public rclcpp::Node
{
public:

  SimulatedVehicleNode();

private:

  void load_parameters();
  void create_subscribers();
  void create_publishers();
  void timer_callback();

  void simulate_ego_vehicle();
  void publish_vehicle_states();
  void publish_ego_transform();
  void add_noise();
  void publish_traffic_participants();

  void update_dynamic_subscriptions();

  /******************************* PUBLISHERS ************************************************************/
  rclcpp::Publisher<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr   publisher_vehicle_state_dynamic;
  rclcpp::Publisher<adore_ros2_msgs::msg::StateMonitor>::SharedPtr          publisher_state_monitor;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                            tf_transform_broadcaster;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher_traffic_participant_set;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr    publisher_traffic_participant;

  /******************************* SUBSCRIBERS ************************************************************/
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleCommand>::SharedPtr subscriber_vehicle_command;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr            subscriber_teleop_controller;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                  subscriber_automation_toggle;

  using StateSubscriber = rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr;
  std::unordered_map<std::string, StateSubscriber> other_vehicle_traffic_participant_subscribers;

  void vehicle_command_callback( const adore_ros2_msgs::msg::VehicleCommand& msg );
  void teleop_controller_callback( const geometry_msgs::msg::Twist& msg );
  void automation_toggle_callback( const std_msgs::msg::Bool& msg );
  void other_vehicle_traffic_participant_callback( const adore_ros2_msgs::msg::TrafficParticipant& msg,
                                                   const std::string&                              vehicle_namespace );

  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::TimerBase::SharedPtr dynamic_subscription_timer;

  /******************************* OTHER MEMBERS ************************************************************/
  dynamics::PhysicalVehicleParameters model;

  adore::dynamics::VehicleStateDynamic                          current_vehicle_state;
  adore::dynamics::TrafficParticipant                           current_traffic_participant;
  adore::dynamics::VehicleStateDynamic                          vehicle_state_last_time_step;
  adore::dynamics::VehicleCommand                               latest_vehicle_command;
  std::unordered_map<std::string, dynamics::TrafficParticipant> other_vehicles;

  rclcpp::Time current_time;
  rclcpp::Time last_update_time;

  double integration_step_size = 0.005;
  double time_step_s           = 0.05;

  double              ego_vehicle_start_position_x = 0;
  double              ego_vehicle_start_position_y = 0;
  double              ego_vehicle_start_psi        = 0;
  std::vector<double> ego_vehicle_shape            = { 0.0, 0.0, 0.0 };

  bool   manual_control_override = false;
  bool   controllable            = false;
  double sensor_range            = 100;

  // Create normal distributions for each variable
  double pos_stddev;
  double vel_stddev;
  double yaw_stddev;
  double accel_stddev;

  std::normal_distribution<double> pos_noise;
  std::normal_distribution<double> vel_noise;
  std::normal_distribution<double> yaw_noise;
  std::normal_distribution<double> accel_noise;
};
} // namespace simulated_vehicle
} // namespace adore
