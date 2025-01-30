// Filename: Coordinate_Calculations.cpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: A script containing all relevant functions to do coordinate calculations (including tacking path planning)

#include <iostream>
#include <cmath>
#include <vector>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "Coordinate_Calculations.h"
#include "WaypointQueue.hpp"
#include "CoordinateCalculationsNode.hpp"


using namespace std;

const unsigned int EARTH_RADIUS = 6371000.0; // Earth radius in meters
const float DEG_TO_RAD = (M_PI / 180.0f);
const float RAD_TO_DEG = (180.0f / M_PI);
const float IRONS_DEGREE = 30;

CoordinateCalculations& CoordinateCalculations::getInstance() 
{
    // Create a static instance of the class
    static CoordinateCalculations instance;
    return instance;
}


// Calculating distance between two coordinate points using the Haversine Formula
float CoordinateCalculations::calculate_distance(sensor_msgs::msg::NavSatFix coord1, sensor_msgs::msg::NavSatFix coord2)
{
    float delta_lat = (coord2.latitude - coord1.latitude) * DEG_TO_RAD;
    float delta_lon = (coord2.longitude - coord1.longitude) * DEG_TO_RAD;
    float a = sin(delta_lat / 2) * sin(delta_lat / 2) +
              cos(coord1.latitude * DEG_TO_RAD) * cos(coord2.latitude * DEG_TO_RAD) *
              sin(delta_lon / 2) * sin(delta_lon / 2);

    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS * c;
}


float CoordinateCalculations::calculate_bearing(sensor_msgs::msg::NavSatFix curr_coord, sensor_msgs::msg::NavSatFix tar_coord)
{
    // Converting to radians
    float curr_lat_rad = curr_coord.latitude * DEG_TO_RAD;
    float curr_lon_rad = curr_coord.longitude * DEG_TO_RAD;
    float tar_lat_rad = tar_coord.latitude * DEG_TO_RAD;
    float tar_lon_rad = tar_coord.longitude * DEG_TO_RAD;

    // Calculate the difference in longitudes
    float longitude_distance = tar_lon_rad - curr_lon_rad;

    // Bearing formula obtained from https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
    float y = sin(longitude_distance) * cos(tar_lat_rad);
    float x = cos(curr_lat_rad) * sin(tar_lat_rad) - sin(curr_lat_rad) * cos(tar_lat_rad) * cos(longitude_distance);
    float bearing = atan2(y, x);

    bearing = bearing * RAD_TO_DEG;
    bearing = fmod((bearing + 360), 360);

    return bearing;
}

sensor_msgs::msg::NavSatFix CoordinateCalculations::convert_to_position(sensor_msgs::msg::NavSatFix position, float angle, float distance)
{

    // Convert latitude and longitude from degrees to radians
    float lat_rad = position.latitude * DEG_TO_RAD;
    float lon_rad = position.longitude * DEG_TO_RAD;
    float angle_rad = angle * DEG_TO_RAD;

    // Calculate the new latitude
    float new_lat_rad = asin(sin(lat_rad) * cos(distance / EARTH_RADIUS) + cos(lat_rad) * sin(distance / EARTH_RADIUS) * cos(angle_rad));

    // Calculate the new longitude
    float new_lon_rad = lon_rad + atan2(sin(angle_rad) * sin(distance / EARTH_RADIUS) * cos(lat_rad), cos(distance / EARTH_RADIUS) - sin(lat_rad) * sin(new_lat_rad));

    // Convert the new latitude and longitude from radians to degrees
    float new_lat = new_lat_rad * RAD_TO_DEG;
    float new_long = new_lon_rad * RAD_TO_DEG;

    sensor_msgs::msg::NavSatFix new_position;
    new_position.latitude = new_lat;
    new_position.longitude = new_long;
    // Return the new position as a Coordinate
    return new_position;
}

std::pair<std::string, float> CoordinateCalculations::calculate_directional_bearing(sensor_msgs::msg::NavSatFix target_waypoint)
{
    CoordinateCalculationsNode node_instance;
    float current_heading = node_instance.get_magnetometer_heading().data;
    sensor_msgs::msg::NavSatFix current_position = node_instance.get_curr_pos();

    float target_bearing = calculate_bearing(current_position, target_waypoint);
    float bearing_difference = target_bearing - current_heading;
    
    bearing_difference = fmod(bearing_difference + 360.0f, 360.0f);
    
    // Adjust to be within the range [-180, 180] for easier handling of turns
    if (bearing_difference > 180.0f)
    {
        bearing_difference -= 360.0f;
    }

    // Determine the turn direction and angle
    std::string turn_direction;
    float turn_angle;

    if (bearing_difference > 0)
    {
        // Target is to the right
        turn_direction = "right";
        turn_angle = bearing_difference;
    } 
    else{
        // Target is to the left
        turn_direction = "left";
        turn_angle = fabs(bearing_difference);
    }
    return std::make_pair(turn_direction, turn_angle);
}

//TODO: Check if this function is actually needed
float CoordinateCalculations::calculate_angle_to_wind() 
{   
    CoordinateCalculationsNode node_instance;
    float wind_direction = node_instance.get_wind_angle().data;
    float current_heading = node_instance.get_magnetometer_heading().data;
    // Calculate the angle difference between the current heading and wind direction
    float angle_difference = abs(current_heading - wind_direction);

    // Normalize to range [0, 180] degrees
    if (angle_difference > 180) 
    {
        angle_difference = 360 - angle_difference;
    }

    return angle_difference;
}


std::vector<sensor_msgs::msg::NavSatFix> CoordinateCalculations::plan_path(const sensor_msgs::msg::NavSatFix& curr_position, const sensor_msgs::msg::NavSatFix& next_waypoint, const float curr_wind_angle)
{
    std::vector<sensor_msgs::msg::NavSatFix> waypoints;

    float destination_bearing = calculate_bearing(curr_position, next_waypoint);
    float wind_direction = curr_wind_angle;  // Get current wind direction
    
    // Calculate angle between wind direction and destination bearing
    float angle_to_wind = abs(wind_direction - destination_bearing);
    if (angle_to_wind > 180) 
    {
        angle_to_wind = 360 - angle_to_wind;  // Normalize angle to 0-180 degrees
    }

    // Check if tacking is needed by comparing with the max allowable upwind angle
    if (angle_to_wind > max_upwind_angle) 
    {
        // Tacking is required, so calculate intermediate waypoints
        float tack_distance = calculate_distance(curr_position, next_waypoint) / cos(max_upwind_angle * M_PI / 180.0);
        float first_tack_angle, second_tack_angle;

        // Determine tack direction based on wind and destination bearings
        float bearing_difference = wind_direction - destination_bearing;
        if (bearing_difference < 0) 
        {
            bearing_difference += 360;  // Normalize bearing difference to 0-360
        }

        // Set tack angles based on which direction the boat should tack first
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

        // Calculate the two tack waypoints and add them to the vector
        sensor_msgs::msg::NavSatFix first_tack_waypoint = convert_to_position(curr_position, first_tack_angle, tack_distance / 2);
        sensor_msgs::msg::NavSatFix second_tack_waypoint = convert_to_position(first_tack_waypoint, second_tack_angle, tack_distance / 2);
        
        waypoints.push_back(first_tack_waypoint);
        waypoints.push_back(second_tack_waypoint);
    }
    else
    {
        // If no tacking is required, only add the original destination waypoint
        waypoints.push_back(next_waypoint);
    }

    return waypoints;
}

//Constructor used by getInstance
CoordinateCalculations::CoordinateCalculations() {
    // Initialize any member variables if needed
}