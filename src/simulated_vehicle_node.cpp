/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "simulated_vehicle_node.hpp"

#include <cstdint>

#include "adore_math/distance.h"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include <adore_math/point.h>

namespace adore
{
namespace simulated_vehicle
{
SimulatedVehicleNode::SimulatedVehicleNode( const rclcpp::NodeOptions& options ) :
  Node( "simulated_vehicle_node", options )
{
  current_time     = now();
  last_update_time = now();

  load_parameters();
  create_publishers();
  create_subscribers();

  pos_noise   = std::normal_distribution( 0.0, pos_stddev );
  vel_noise   = std::normal_distribution( 0.0, vel_stddev );
  yaw_noise   = std::normal_distribution( 0.0, yaw_stddev );
  accel_noise = std::normal_distribution( 0.0, accel_stddev );

  if( controllable )
    dynamic_subscription_timer = create_wall_timer( 1s, std::bind( &SimulatedVehicleNode::update_dynamic_subscriptions, this ) );
}

void
SimulatedVehicleNode::load_parameters()
{
  const std::string vehicle_model_file = declare_parameter<std::string>( "vehicle_model_file", "" );
  model                                = dynamics::PhysicalVehicleModel( vehicle_model_file, false );

  // Controllability
  controllable = declare_parameter<bool>( "controllable", true );

  // Start position and shape
  ego_vehicle_start_position_x = declare_parameter<double>( "set_start_position_x", 0.0 );
  ego_vehicle_start_position_y = declare_parameter<double>( "set_start_position_y", 0.0 );
  ego_vehicle_start_psi        = declare_parameter<double>( "set_start_psi", 0.0 );

  // Noise parameters
  pos_stddev   = declare_parameter<double>( "position_noise_stddev", 0.0 );
  vel_stddev   = declare_parameter<double>( "velocity_noise_stddev", 0.0 );
  yaw_stddev   = declare_parameter<double>( "yaw_noise_stddev", 0.0 );
  accel_stddev = declare_parameter<double>( "acceleration_noise_stddev", 0.0 );

  // Vehicle ID
  traffic_participant.id     = declare_parameter<int>( "vehicle_id", 0 );
  traffic_participant.v2x_id = declare_parameter<int>( "v2x_id", 0 );

  current_vehicle_state.x               = ego_vehicle_start_position_x;
  current_vehicle_state.y               = ego_vehicle_start_position_y;
  current_vehicle_state.yaw_angle       = ego_vehicle_start_psi;
  current_vehicle_state.time            = current_time.seconds();
  std::string current_vehicle_namespace = get_namespace();
  if( !current_vehicle_namespace.empty() && current_vehicle_namespace.front() == '/' )
  {
    current_vehicle_namespace = current_vehicle_namespace.substr( 1 );
  }
  current_vehicle_state.frame_id          = current_vehicle_namespace; // Set the frame_id to the node's namespace
  traffic_participant.state               = current_vehicle_state;
  traffic_participant.physical_parameters = model.params;

  tf_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( *this );
}

void
SimulatedVehicleNode::create_publishers()
{
  publisher_vehicle_state_dynamic   = create_publisher<StateAdapter>( "vehicle_state_dynamic", 10 );
  publisher_traffic_participant_set = create_publisher<ParticipantSetAdapter>( "traffic_participants", 10 );
  publisher_traffic_participant     = create_publisher<ParticipantAdapter>( "simulated_traffic_participant", 10 );
}

void
SimulatedVehicleNode::create_subscribers()
{
  main_timer = create_wall_timer( time_step_s * 1000ms, std::bind( &SimulatedVehicleNode::timer_callback, this ) );

  subscriber_vehicle_command = create_subscription<VehicleCommandAdapter>( "next_vehicle_command", 10,
                                                                           std::bind( &SimulatedVehicleNode::vehicle_command_callback, this,
                                                                                      std::placeholders::_1 ) );

  subscriber_teleop_controller = create_subscription<geometry_msgs::msg::Twist>(
    "teleop_controller", 10, std::bind( &SimulatedVehicleNode::teleop_controller_callback, this, std::placeholders::_1 ) );

  subscriber_automation_toggle = create_subscription<std_msgs::msg::Bool>( "automation_toggle", 10,
                                                                           std::bind( &SimulatedVehicleNode::automation_toggle_callback,
                                                                                      this, std::placeholders::_1 ) );
}

void
SimulatedVehicleNode::update_dynamic_subscriptions()
{
  auto       topic_names_and_types = get_topic_names_and_types();
  std::regex valid_topic_regex( R"(^/([^/]+)/simulated_traffic_participant$)" );
  std::regex valid_type_regex( R"(^adore_ros2_msgs/msg/TrafficParticipant$)" );

  for( const auto& topic : topic_names_and_types )
  {
    const std::string&              topic_name = topic.first;
    const std::vector<std::string>& types      = topic.second;

    std::smatch match;
    if( std::regex_match( topic_name, match, valid_topic_regex )
        && std::any_of( types.begin(), types.end(),
                        [&]( const std::string& type ) { return std::regex_match( type, valid_type_regex ); } ) )
    {
      std::string vehicle_namespace = match[1].str();

      // Skip subscribing to own namespace
      if( vehicle_namespace == std::string( get_namespace() ).substr( 1 ) )
      {
        continue;
      }

      // Check if already subscribed
      if( other_vehicle_traffic_participant_subscribers.count( vehicle_namespace ) > 0 )
      {
        continue;
      }

      // Create a new subscription
      auto subscription = create_subscription<ParticipantAdapter>( topic_name, 10,
                                                                   [this, vehicle_namespace]( const dynamics::TrafficParticipant& msg ) {
        other_vehicle_traffic_participant_callback( msg, vehicle_namespace );
      } );

      other_vehicle_traffic_participant_subscribers[vehicle_namespace] = subscription;

      RCLCPP_INFO( get_logger(), "Subscribed to new vehicle namespace: %s", vehicle_namespace.c_str() );
    }
  }
}

void
SimulatedVehicleNode::timer_callback()
{
  current_time = now();
  if( controllable )
  {
    simulate_ego_vehicle();
    publish_traffic_participants();
  }

  last_update_time = current_time;
  publish_vehicle_states();
  publish_ego_transform();
}

void
SimulatedVehicleNode::simulate_ego_vehicle()
{
  auto   prev_state = current_vehicle_state;
  double prev_v     = current_vehicle_state.vx;

  current_vehicle_state = dynamics::integrate_rk4( current_vehicle_state, latest_vehicle_command, time_step_s, model.motion_model );
  double current_v      = current_vehicle_state.vx;
  current_vehicle_state.frame_id = prev_state.frame_id;

  current_vehicle_state.ax             = ( current_v - prev_v ) / time_step_s;
  current_vehicle_state.steering_angle = latest_vehicle_command.steering_angle;

  current_vehicle_state.steering_rate = math::normalize_angle( current_vehicle_state.steering_angle - prev_state.steering_angle )
                                      / time_step_s;

  current_vehicle_state.yaw_rate = math::normalize_angle( current_vehicle_state.yaw_angle - prev_state.yaw_angle ) / time_step_s;

  traffic_participant.state = current_vehicle_state;
}

void
SimulatedVehicleNode::publish_ego_transform()
{
  auto vehicle_frame = dynamics::conversions::vehicle_state_to_transform( current_vehicle_state, last_update_time, get_namespace() );
  tf_transform_broadcaster->sendTransform( vehicle_frame );
}

void
SimulatedVehicleNode::teleop_controller_callback( const geometry_msgs::msg::Twist& msg )
{
  if( !manual_control_override )
    return;

  constexpr double MAX_STEER = 0.7;
  constexpr double MAX_ACC   = 1.0;

  double& steer  = latest_vehicle_command.steering_angle;
  double& acc    = latest_vehicle_command.acceleration;
  steer         += msg.linear.y / 10.0;
  steer          = std::max( -MAX_STEER, std::min( steer, MAX_STEER ) );

  acc += msg.linear.x / 10.0;
  acc  = std::max( -MAX_ACC, std::min( acc, MAX_ACC ) );
}

void
SimulatedVehicleNode::automation_toggle_callback( const std_msgs::msg::Bool& msg )
{
  manual_control_override = msg.data;
}

void
SimulatedVehicleNode::vehicle_command_callback( const dynamics::VehicleCommand& msg )
{
  if( !manual_control_override )
    latest_vehicle_command = msg;
}

void
SimulatedVehicleNode::publish_vehicle_states()
{
  traffic_participant.state                = current_vehicle_state;
  traffic_participant.physical_parameters  = model.params;
  traffic_participant.state.time           = now().seconds();
  auto noisy_state                         = current_vehicle_state;
  auto generator                           = std::default_random_engine( std::chrono::system_clock::now().time_since_epoch().count() );
  noisy_state.x                           += pos_noise( generator );
  noisy_state.y                           += pos_noise( generator );
  noisy_state.vx                          += vel_noise( generator );
  noisy_state.yaw_angle                   += yaw_noise( generator );

  publisher_vehicle_state_dynamic->publish( noisy_state );
  publisher_traffic_participant->publish( traffic_participant );
}

void
SimulatedVehicleNode::other_vehicle_traffic_participant_callback( const dynamics::TrafficParticipant& msg,
                                                                  const std::string&                  vehicle_namespace )
{
  other_vehicles[vehicle_namespace] = msg;
}

void
SimulatedVehicleNode::publish_traffic_participants()
{

  dynamics::TrafficParticipantSet traffic_participants;

  for( const auto& [vehicle_namespace, other_vehicle] : other_vehicles )
  {
    double distance = adore::math::distance_2d( other_vehicle.state, current_vehicle_state );

    if( distance > sensor_range )
      continue;

    traffic_participants.participants[other_vehicle.id] = other_vehicle;
  }

  publisher_traffic_participant_set->publish( traffic_participants );
}

} // namespace simulated_vehicle
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::simulated_vehicle::SimulatedVehicleNode>( rclcpp::NodeOptions{} ) );
  rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::simulated_vehicle::SimulatedVehicleNode )
