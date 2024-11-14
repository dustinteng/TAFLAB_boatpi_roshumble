#include <cmath>
#include <vector>
#include "Coordinate_Calculations.h"
#include "Waypoint_Queue.hpp"
#include "rclcpp/rclcpp.hpp"
#include "coordinate_calculations_node.hpp"

const unsigned int EARTH_RADIUS = 6371000; // Earth radius in meters
constexpr float DEG_TO_RAD = M_PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / M_PI;
constexpr float IRONS_DEGREE = 30;

float CoordinateCalculations::calculate_distance(Datatypes::Coordinate coord1, Datatypes::Coordinate coord2)
{
    float delta_lat = (coord2.latitude - coord1.latitude) * DEG_TO_RAD;
    float delta_lon = (coord2.longitude - coord1.longitude) * DEG_TO_RAD;
    float a = std::sin(delta_lat / 2) * std::sin(delta_lat / 2) +
              std::cos(coord1.latitude * DEG_TO_RAD) * std::cos(coord2.latitude * DEG_TO_RAD) *
              std::sin(delta_lon / 2) * std::sin(delta_lon / 2);

    float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    RCLCPP_INFO(rclcpp::get_logger("CoordinateCalculations"), "Calculated distance: %.2f meters", EARTH_RADIUS * c);
    return EARTH_RADIUS * c;
}

float CoordinateCalculations::calculate_bearing(Datatypes::Coordinate curr_coord, Datatypes::Coordinate tar_coord)
{
    float curr_lat_rad = curr_coord.latitude * DEG_TO_RAD;
    float curr_lon_rad = curr_coord.longitude * DEG_TO_RAD;
    float tar_lat_rad = tar_coord.latitude * DEG_TO_RAD;
    float tar_lon_rad = tar_coord.longitude * DEG_TO_RAD;

    float longitude_distance = tar_lon_rad - curr_lon_rad;
    float y = std::sin(longitude_distance) * std::cos(tar_lat_rad);
    float x = std::cos(curr_lat_rad) * std::sin(tar_lat_rad) - std::sin(curr_lat_rad) * std::cos(tar_lat_rad) * std::cos(longitude_distance);
    float bearing = std::atan2(y, x) * RAD_TO_DEG;

    bearing = std::fmod((bearing + 360), 360);
    RCLCPP_INFO(rclcpp::get_logger("CoordinateCalculations"), "Calculated bearing: %.2f degrees", bearing);
    return bearing;
}

void CoordinateCalculations::plan_path(const Datatypes::Coordinate& curr_position, const Datatypes::Coordinate& next_waypoint)
{
    float destination_bearing = calculate_bearing(curr_position, next_waypoint);
    float wind_direction = 10.0; //Change this!!!

    float angle_to_wind = std::abs(wind_direction - destination_bearing);
    if (angle_to_wind > 180) 
    {
        angle_to_wind = 360 - angle_to_wind;
    }

    if (angle_to_wind > max_upwind_angle) 
    {
        RCLCPP_INFO(rclcpp::get_logger("CoordinateCalculations"), "Tacking required for path planning.");
    }

    float tack_distance = calculate_distance(curr_position, next_waypoint) / std::cos(max_upwind_angle * DEG_TO_RAD);
    float first_tack_angle, second_tack_angle;

    float bearing_difference = wind_direction - destination_bearing;
    if (bearing_difference < 0) 
    {
        bearing_difference += 360;
    }

    if (bearing_difference < 180) 
    {
        first_tack_angle = destination_bearing + max_upwind_angle;
        second_tack_angle = destination_bearing - max_upwind_angle;
    } 
    else 
    {
        first_tack_angle = destination_bearing - max_upwind_angle;
        second_tack_angle = destination_bearing + max_upwind_angle;
    }

    Datatypes::Coordinate first_tack_waypoint = convert_to_position(curr_position, first_tack_angle, tack_distance / 2);
    Datatypes::Coordinate second_tack_waypoint = convert_to_position(first_tack_waypoint, second_tack_angle, tack_distance / 2);

    WaypointQueue::getInstance().add_front_waypoint(first_tack_waypoint);
    WaypointQueue::getInstance().add_front_waypoint(second_tack_waypoint);
    WaypointQueue::getInstance().add_front_waypoint(next_waypoint);

    RCLCPP_INFO(rclcpp::get_logger("CoordinateCalculations"), "Planned path with tacking waypoints.");
}

CoordinateCalculations& CoordinateCalculations::getInstance()
{
    static CoordinateCalculations instance;
    return instance;
}
