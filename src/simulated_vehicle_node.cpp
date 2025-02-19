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
#include "simulated_vehicle_node.hpp"

#include "adore_math/distance.h"

namespace adore
{
namespace simulated_vehicle
{
SimulatedVehicleNode::SimulatedVehicleNode() :
  Node( "simulated_vehicle_node" )
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

  // Timer for dynamically discovering and subscribing to new vehicle topics
  dynamic_subscription_timer = create_wall_timer( 1s, std::bind( &SimulatedVehicleNode::update_dynamic_subscriptions, this ) );
}

void
SimulatedVehicleNode::load_parameters()
{
  std::string vehicle_model_file;
  declare_parameter( "vehicle_model_file", "" );
  get_parameter( "vehicle_model_file", vehicle_model_file );
  model = dynamics::PhysicalVehicleModel( vehicle_model_file, false );

  declare_parameter( "controllable", true );
  get_parameter( "controllable", controllable );

  declare_parameter( "set_start_position_x", 0.0 );
  declare_parameter( "set_start_position_y", 0.0 );
  declare_parameter( "set_start_psi", 0.0 );
  declare_parameter<std::vector<double>>( "set_shape", { 4.5, 2.0, 2.0 } );

  get_parameter( "set_start_position_x", ego_vehicle_start_position_x );
  get_parameter( "set_start_position_y", ego_vehicle_start_position_y );
  get_parameter( "set_start_psi", ego_vehicle_start_psi );
  get_parameter( "set_shape", ego_vehicle_shape );

  declare_parameter( "position_noise_stddev", 0.0 );
  declare_parameter( "velocity_noise_stddev", 0.0 );
  declare_parameter( "yaw_noise_stddev", 0.00 );
  declare_parameter( "acceleration_noise_stddev", 0.00 );

  get_parameter( "position_noise_stddev", pos_stddev );
  get_parameter( "velocity_noise_stddev", vel_stddev );
  get_parameter( "yaw_noise_stddev", yaw_stddev );
  get_parameter( "acceleration_noise_stddev", accel_stddev );

  declare_parameter( "vehicle_id", 0 );
  get_parameter( "vehicle_id", current_traffic_participant.id );

  current_vehicle_state.x              = ego_vehicle_start_position_x;
  current_vehicle_state.y              = ego_vehicle_start_position_y;
  current_vehicle_state.z              = 0;
  current_vehicle_state.yaw_angle      = ego_vehicle_start_psi;
  current_vehicle_state.vx             = 0;
  current_vehicle_state.vy             = 0;
  current_vehicle_state.yaw_rate       = 0;
  current_vehicle_state.steering_angle = 0;
  current_vehicle_state.steering_rate  = 0;
  current_vehicle_state.ax             = 0;
  current_vehicle_state.time           = current_time.seconds();

  current_traffic_participant.bounding_box.length = ego_vehicle_shape[0];
  current_traffic_participant.bounding_box.width  = ego_vehicle_shape[1];
  current_traffic_participant.bounding_box.height = ego_vehicle_shape[2];

  latest_vehicle_command.steering_angle = 0;
  latest_vehicle_command.acceleration   = 0;
}

void
SimulatedVehicleNode::create_publishers()
{
  publisher_vehicle_state_dynamic = create_publisher<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state/dynamic", 10 );
  publisher_state_monitor         = create_publisher<adore_ros2_msgs::msg::StateMonitor>( "vehicle_state/monitor", 10 );

  publisher_traffic_participant_set = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants", 10 );

  publisher_traffic_participant = create_publisher<adore_ros2_msgs::msg::TrafficParticipant>( "simulated_traffic_participant", 10 );

  tf_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>( *this );
}

void
SimulatedVehicleNode::create_subscribers()
{
  main_timer = create_wall_timer( time_step_s * 1000ms, std::bind( &SimulatedVehicleNode::timer_callback, this ) );

  subscriber_vehicle_command = create_subscription<adore_ros2_msgs::msg::VehicleCommand>(
    "next_vehicle_command", 10, std::bind( &SimulatedVehicleNode::vehicle_command_callback, this, std::placeholders::_1 ) );

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
      auto subscription = create_subscription<adore_ros2_msgs::msg::TrafficParticipant>(
        topic_name, 10, [this, vehicle_namespace]( const adore_ros2_msgs::msg::TrafficParticipant& msg ) {
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
    if( current_traffic_participant.id == 0 )
      publish_traffic_participants();
  }

  last_update_time = current_time;
  publish_vehicle_states();
  if( current_traffic_participant.id == 0 )
    publish_ego_transform();
}

void
SimulatedVehicleNode::simulate_ego_vehicle()
{
  auto t                = current_vehicle_state.time;
  current_vehicle_state = dynamics::integrate_rk4( current_vehicle_state, latest_vehicle_command, time_step_s, model.motion_model );
  current_traffic_participant.state = current_vehicle_state;
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
SimulatedVehicleNode::vehicle_command_callback( const adore_ros2_msgs::msg::VehicleCommand& msg )
{
  if( !manual_control_override )
    latest_vehicle_command = adore::dynamics::conversions::to_cpp_type( msg );
}

void
SimulatedVehicleNode::publish_ego_transform()
{
  auto vehicle_frame = dynamics::conversions::vehicle_state_to_transform( current_vehicle_state, last_update_time );
  tf_transform_broadcaster->sendTransform( vehicle_frame );
}

void
SimulatedVehicleNode::publish_vehicle_states()
{
  adore_ros2_msgs::msg::VehicleStateDynamic dynamic_msg = dynamics::conversions::to_ros_msg( current_vehicle_state );
  publisher_vehicle_state_dynamic->publish( dynamic_msg );

  adore_ros2_msgs::msg::StateMonitor state_monitor_msg;
  state_monitor_msg.localization_error = pos_stddev;
  publisher_state_monitor->publish( state_monitor_msg );

  adore_ros2_msgs::msg::TrafficParticipant ego_as_traffic_participant = dynamics::conversions::to_ros_msg( current_traffic_participant );
  publisher_traffic_participant->publish( ego_as_traffic_participant );
}

void
SimulatedVehicleNode::other_vehicle_traffic_participant_callback( const adore_ros2_msgs::msg::TrafficParticipant& msg,
                                                                  const std::string&                              vehicle_namespace )
{
  other_vehicles[vehicle_namespace] = dynamics::conversions::to_cpp_type( msg );
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
  publisher_traffic_participant_set->publish( dynamics::conversions::to_ros_msg( traffic_participants ) );
}
} // namespace simulated_vehicle
} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::simulated_vehicle::SimulatedVehicleNode>() );
  rclcpp::shutdown();
}
