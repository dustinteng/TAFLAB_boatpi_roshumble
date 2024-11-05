// Filename: WaypointQueue.cpp
// Author: Kieran Pereira
// Date: 10/11/2024
// Description: Implementation file for the WaypointQueue class, providing a queue-based waypoint management system.

#include <iostream>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>

#include "WaypointQueue.hpp"
#include "Coordinate_Calculations.h"
#include "TAF_AS5600.h"
#include "TAF_GTU7.h"
#include "Boat_steer.h"

void WaypointQueue::initialize_autonomous_mode()
{
    lock_guard<mutex> lock(queue_mutex);
    autonomous_mode = true;
    execution_thread = thread(&WaypointQueue::execute_waypoints, this);
}

Datatypes::Coordinate WaypointQueue::get_next_waypoint()
{
    unique_lock<mutex> lock(queue_mutex);
    while (waypoints.empty() && autonomous_mode)
    {
        queue_condition.wait(lock);
    }
    if (!waypoints.empty())
    {
        Datatypes::Coordinate next_waypoint = waypoints.front();
        waypoints.pop_front();
        previous_waypoint = next_waypoint;
        return next_waypoint;
    }
    return Datatypes::Coordinate(); // Return default coordinate if queue is empty
}

std::size_t WaypointQueue::get_len_queue()
{
    return waypoints.size();
}

void WaypointQueue::add_waypoint(Datatypes::Coordinate waypoint)
{
    lock_guard<mutex> lock(queue_mutex);
    waypoints.push_back(waypoint);
    queue_condition.notify_one();
}

void WaypointQueue::add_front_waypoint(Datatypes::Coordinate waypoint)
{
    lock_guard<mutex> lock(queue_mutex);
    waypoints.push_front(waypoint);
    queue_condition.notify_one();
}

void WaypointQueue::clear_queue()
{
    lock_guard<mutex> lock(queue_mutex);
    while (!waypoints.empty())
    {
        waypoints.clear();
    }
}

bool WaypointQueue::is_empty() const
{
    return waypoints.empty();
}

void turn_boat(float desired_angle_heading, bool speed_needed)
{
    while (true) 
    {
        // 1. Get the current turn direction and angle difference
        auto [turn_direction, turn_angle] = CoordinateCalculations::getInstance().calculate_directional_bearing({desired_angle_heading});

        // 2. Check if the turn angle is within a small tolerance
        if (abs(turn_angle) <= 5.0f)  // For example, within 5 degrees of desired heading
        {
            break; // Exit the loop as the boat is close enough to the desired heading
        }

        // 3. Adjust rudder based on turn direction
        if (turn_direction == "right") 
        {
            set_rudder_servo(45);
        } 
        else if (turn_direction == "left") 
        {
            set_rudder_servo(-45);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    set_rudder_servo(0);  // TODO: Test to determine if 0 is the neutral rudder position
}

void WaypointQueue::execute_waypoints()
{
    
    // Main loop: Continue while the boat is in autonomous mode
    while (autonomous_mode) 
    {
        // If the waypoint queue is empty, check if the boat is near its final destination
        if (waypoints.empty()) 
        {
            // Get the current position of the boat
            Datatypes::Coordinate curr_position = get_curr_coordinate();

            // Check if the boat is within m of the last known waypoint
            //TODO: Check if 0.05 corresponds with 5m
            if (CoordinateCalculations::getInstance().calculate_distance(curr_position, previous_waypoint) <= 0.05) 
            {
                // If the boat is close enough to the destination, set it to idle by pointing sail into wind
                set_rudder_servo(get_avg_angle());
                set_sail_servo(get_avg_angle());
                break;
            }
            continue;  // Continue checking until waypoints are available or goal is reached
        }

        // Retrieve the next waypoint from the queue
        Datatypes::Coordinate next_waypoint = get_next_waypoint();
        bool tack_required = false;  // Flag to determine if tacking is necessary

        // Get the boat's current position and calculate bearing to the next waypoint
        Datatypes::Coordinate curr_position = get_curr_coordinate();
        float destination_bearing = CoordinateCalculations::getInstance().calculate_bearing(curr_position, next_waypoint);
        float wind_direction = get_avg_angle();  // Get current wind direction

        // Determine the angle between wind direction and destination bearing
        float angle_to_wind = abs(wind_direction - destination_bearing);
        if (angle_to_wind > 180) 
        {
            angle_to_wind = 360 - angle_to_wind;  // Normalize angle to 0-180 degrees
        }

        // Check if a tack is needed by comparing to the max allowable upwind angle
        if (angle_to_wind <= max_upwind_angle) 
        {
            tack_required = true;  // Set flag if tacking is necessary
        }

        // If tacking is required, calculate intermediate tack waypoints
        if (tack_required) 
        {
            // Calculate tack distance using trigonometry based on max_upwind_angle
            float tack_distance = CoordinateCalculations::getInstance().calculate_distance(curr_position, next_waypoint) / cos(max_upwind_angle * M_PI / 180.0);
            float first_tack_angle, second_tack_angle;

            // Determine tack direction by comparing wind and destination bearings
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

            // Calculate the two tack waypoints and add them to the queue
            Datatypes::Coordinate first_tack_waypoint = CoordinateCalculations::getInstance().convert_to_position(curr_position, first_tack_angle, tack_distance / 2);
            Datatypes::Coordinate second_tack_waypoint = CoordinateCalculations::getInstance().convert_to_position(first_tack_waypoint, second_tack_angle, tack_distance / 2);
            add_front_waypoint(first_tack_waypoint);
            add_front_waypoint(second_tack_waypoint);
            add_waypoint(next_waypoint);  // Finally, add the original destination as the last waypoint
        } 
        else 
        {
            // If no tack is required, proceed directly to the next waypoint
            add_waypoint(next_waypoint);
        }

        // Main navigation loop: Move the boat along the path while waypoints are available
        while (autonomous_mode && !waypoints.empty()) 
        {
            // Get updated boat position and check the distance to the next waypoint
            curr_position = get_curr_coordinate();
            next_waypoint = get_next_waypoint();

            float distance_to_waypoint = CoordinateCalculations::getInstance().calculate_distance(curr_position, next_waypoint);

            // If the boat is within 0.05m of the next waypoint, dequeue it and move to the next
            if (distance_to_waypoint <= 0.05) 
            {
                waypoints.pop_front();  // Remove the waypoint from the queue
                continue;  // Continue to the next waypoint in the queue
            }

            // Calculate bearing to the next waypoint and adjust servos to steer the boat
            float bearing_to_waypoint = CoordinateCalculations::getInstance().calculate_bearing(curr_position, next_waypoint);
            steering_boat(bearing_to_waypoint);

            // Add a delay to prevent excessive looping, allowing for smooth movement
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Final check after the queue is empty to ensure the boat is at the final destination
        if (waypoints.empty() && CoordinateCalculations::getInstance().calculate_distance(curr_position, next_waypoint) <= 0.05) 
        {
            set_rudder_servo(get_avg_angle());  // Point the rudder into the wind to idle
        }
    }
}


void WaypointQueue::close_autonomous_mode()
{
    
    lock_guard<mutex> lock(queue_mutex);
    autonomous_mode = false;
    queue_condition.notify_all();
    
    if (execution_thread.joinable())
    {
        execution_thread.join();
    }
    clear_queue();
}

WaypointQueue& WaypointQueue::getInstance() 
{
    static WaypointQueue instance;
    return instance;
}