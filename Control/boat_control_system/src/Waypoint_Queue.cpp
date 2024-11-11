#include "Waypoint_Queue.hpp"
#include "Coordinate_Calculations.h"
#include "waypoint_queue_node.hpp"
#include <chrono>
#include <thread>
#include <cmath>  // For std::abs

void WaypointQueue::initialize_autonomous_mode()
{
    autonomous_mode = true;
}

Datatypes::Coordinate WaypointQueue::get_next_waypoint()
{
    if (!waypoints.empty())
    {
        Datatypes::Coordinate next_waypoint = waypoints.front();
        waypoints.pop_front();
        previous_waypoint = next_waypoint;
        return next_waypoint;
    }
    return Datatypes::Coordinate();  // Return default coordinate if queue is empty
}

void WaypointQueue::add_waypoint(Datatypes::Coordinate waypoint)
{
    waypoints.push_back(waypoint);
}

void WaypointQueue::add_front_waypoint(Datatypes::Coordinate waypoint)
{
    waypoints.push_front(waypoint);
}

void WaypointQueue::clear_queue()
{
    waypoints.clear();
}

bool WaypointQueue::is_empty() const
{
    return waypoints.empty();
}

void turn_boat(float desired_angle_heading, bool speed_needed)
{
    while (true) 
    {
        auto [turn_direction, turn_angle] = CoordinateCalculations::getInstance().calculate_directional_bearing({desired_angle_heading});

        if (std::abs(turn_angle) <= 5.0f)  // Check within 5 degrees tolerance
        {
            break; // Boat is close enough to the desired heading
        }

        // Adjust rudder based on turn direction
        if (turn_direction == "right") 
        {
            // Implement or call a function to set the rudder servo for a right turn
        } 
        else if (turn_direction == "left") 
        {
            // Implement or call a function to set the rudder servo for a left turn
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Reset rudder to neutral position if needed
}

void WaypointQueue::execute_waypoints(Datatypes::Coordinate current_position)
{
    if (autonomous_mode && !waypoints.empty())
    {
        Datatypes::Coordinate next_waypoint = get_next_waypoint();
        float distance_to_waypoint = CoordinateCalculations::getInstance().calculate_distance(current_position, next_waypoint);

        if (distance_to_waypoint <= 0.05)  // Threshold should be configurable
        {
            waypoints.pop_front();  // Reached the waypoint, remove it from the queue
        }
        else
        {
            float bearing_to_waypoint = CoordinateCalculations::getInstance().calculate_bearing(current_position, next_waypoint);
            // turn_boat(bearing_to_waypoint, /*speed_needed=*/true);
        }
    }
}

void WaypointQueue::close_autonomous_mode()
{
    autonomous_mode = false;
    clear_queue();
}

WaypointQueue& WaypointQueue::getInstance()
{
    static WaypointQueue instance;
    return instance;
}
