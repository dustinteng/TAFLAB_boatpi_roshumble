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

#include "DataTypes.hpp"

using namespace std;

class WaypointQueue {
public:


    /*
    @brief A function used to start up autonomous mode sequence. Used to initialise waypoint queue and listeners
    */
    void initialize_autonomous_mode();

    /*
    @brief A function used get and pop the next waypoint from the waypoint queue to be executed
    @return Coordinate of next waypoint in queue
    */
    Datatypes::Coordinate get_next_waypoint();


    /*
    @brief Adds waypoint to queue
    */
    void add_waypoint(Datatypes::Coordinate waypoint);


    /*
    @brief Adds waypoint to the front of the queue
    */
    void add_front_waypoint(Datatypes::Coordinate waypoint);

    /*
    @brief Clears waypoint queue. Used to correct mistakes in input or redirect the boat
    */
    void clear_queue();

    /*
    @brief A helper function used to check if the waypoint queue is empty
    */
    bool is_empty() const;


    /*
    @brief The main function used to drive the boat towards the next waypoint
    */
    void execute_waypoints();

    /*
    @brief A function triggered to teardown existing threads when autonomous mode is disengaged
    */
    void close_autonomous_mode();


    /*
    @brief A function triggered to teardown existing threads when autonomous mode is disengaged
    */
    void turn_boat(float desired_angle_heading, bool speed_needed);


    // Static Method to access single instance
    static WaypointQueue& getInstance();


    /*
    @brief A helper function used to check if the waypoint queue is empty
    @return Length of queue as an int
    */
    std::size_t get_len_queue();


private:

    // Defining a constructor for a singleton class
    WaypointQueue();

    //Destructor (Added for completeness)
    ~WaypointQueue() = default;

    deque<Datatypes::Coordinate> waypoints;
    std::mutex queue_mutex;
    condition_variable queue_condition;
    bool autonomous_mode = false;
    bool running = false;
    thread execution_thread;
    Datatypes::Coordinate previous_waypoint;
    static const unsigned int max_upwind_angle = 45;
};

#endif // WAYPOINTQUEUE_HPP