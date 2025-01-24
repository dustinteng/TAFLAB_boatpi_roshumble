// Filename: WaypointQueue.hpp
// Author: Kieran Pereira
// Date: 10/11/2024
// Description: Header file for the WaypointQueue class, providing a queue-based waypoint management system with autonomous mode handling.

#ifndef WAYPOINTQUEUE_HPP
#define WAYPOINTQUEUE_HPP

#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

using namespace std;

class WaypointQueue {
public:

    /*
    @brief A function used get and pop the next waypoint from the waypoint queue to be executed
    @return Coordinate of next waypoint in queue
    */
    sensor_msgs::msg::NavSatFix get_next_waypoint();

    /*
    @brief Adds waypoint to queue
    */
    void add_waypoint(sensor_msgs::msg::NavSatFix waypoint);

    /*
    @brief Adds waypoint to the front of the queue
    */
    void add_front_waypoint(sensor_msgs::msg::NavSatFix waypoint);

    /*
    @brief Clears waypoint queue. Used to correct mistakes in input or redirect the boat
    */
    void clear_queue();

    /*
    @brief A helper function used to check if the waypoint queue is empty
    */
    bool is_empty() const;

    /*
    @brief A function triggered to teardown existing threads when autonomous mode is disengaged
    */
    void close_autonomous_mode();

    /*
    @brief Static method to access single instance
    */
    static WaypointQueue& getInstance();

    /*
    @brief A helper function used to check the length of the waypoint queue
    @return Length of queue as an int
    */
    std::size_t get_len_queue();



private:

    // Constructor for singleton class
    WaypointQueue();

    // Destructor (added for completeness)
    ~WaypointQueue() = default;

    WaypointQueue(const WaypointQueue&) = delete;  // Prevent copying
    
    WaypointQueue& operator=(const WaypointQueue&) = delete;

    // Mutex for protecting access to the waypoint queue
    std::mutex queue_mutex;

    // Condition variable for thread synchronization
    std::condition_variable queue_condition;

    // Flag indicating autonomous mode is active
    bool autonomous_mode = false;

    // Thread for executing waypoints
    std::thread execution_thread;

    // Stores the last known waypoint
    sensor_msgs::msg::NavSatFix previous_waypoint;

    // Maximum angle for upwind tacking
    static const unsigned int max_upwind_angle = 45;

        
    // Deque to store waypoints
    std::deque<sensor_msgs::msg::NavSatFix> waypoints;
};

#endif // WAYPOINTQUEUE_HPP
