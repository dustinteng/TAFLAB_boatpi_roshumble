// Filename: WaypointQueue.cpp
// Author: Kieran Pereira
// Date: 10/11/2024
// Description: Implementation file for the WaypointQueue class, providing a queue-based waypoint management system.

#include <iostream>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>

#include "WaypointQueue.hpp"

sensor_msgs::msg::NavSatFix WaypointQueue::get_next_waypoint()
{
    unique_lock<std::mutex> lock(queue_mutex);
    while (waypoints.empty() && autonomous_mode)
    {
        queue_condition.wait(lock);
    }
    if (!waypoints.empty())
    {
        sensor_msgs::msg::NavSatFix next_waypoint = waypoints.front();
        waypoints.pop_front();
        previous_waypoint = next_waypoint;
        return next_waypoint;
    }
    return sensor_msgs::msg::NavSatFix(); // Return default coordinate if queue is empty
}

std::size_t WaypointQueue::get_len_queue()
{
    return waypoints.size();
}

void WaypointQueue::add_waypoint(sensor_msgs::msg::NavSatFix waypoint)
{
    lock_guard<mutex> lock(queue_mutex);
    waypoints.push_back(waypoint);
    queue_condition.notify_one();
}

void WaypointQueue::add_front_waypoint(sensor_msgs::msg::NavSatFix waypoint)
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

void WaypointQueue::log_queue()
{
    lock_guard<std::mutex> lock(queue_mutex);
    if (waypoints.empty())
    {
        RCLCPP_INFO(rclcpp::get_logger("WaypointQueue"), "Queue is empty.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("WaypointQueue"), "Queue contains %zu waypoints:", waypoints.size());
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            RCLCPP_INFO(rclcpp::get_logger("WaypointQueue"), 
                        "Waypoint %zu: [lat: %.6f, lon: %.6f, alt: %.2f]",
                        i + 1, waypoints[i].latitude, waypoints[i].longitude, waypoints[i].altitude);
        }
    }
}


// Constructor for singleton class
WaypointQueue::WaypointQueue()
{
    //Init Code
}