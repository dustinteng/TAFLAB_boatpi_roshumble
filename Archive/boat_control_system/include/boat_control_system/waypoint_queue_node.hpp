// Filename: WaypointQueueNode.hpp
// Author: Kieran Pereira
// Date: 10/11/2024
// Description: Header file for the WaypointQueueNode class, which handles waypoint management and interactions with the ROS 2 ecosystem.

#ifndef WAYPOINT_QUEUE_NODE_HPP
#define WAYPOINT_QUEUE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "Waypoint_Queue.hpp"
#include "DataTypes.hpp"

class WaypointQueueNode : public rclcpp::Node
{
public:
    /*
    @brief Constructor for WaypointQueueNode, initializes the node, subscriptions, and publisher.
    */
    WaypointQueueNode();

    /*
    @brief Destructor for WaypointQueueNode, closes autonomous mode on the WaypointQueue instance.
    */
    ~WaypointQueueNode();

private:
    // ROS 2 communication members

    /*
    @brief Subscription to receive waypoints from the ground station.
    */
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ground_station_subscription_;

    /*
    @brief Subscription to receive calculated waypoints from the coordinate_calculations node.
    */
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr calculated_waypoints_subscription_;

    /*
    @brief Subscription to receive GPS data for the current position.
    */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;

    /*
    @brief Publisher to send waypoints to the servo_control node.
    */
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr servo_control_publisher_;

    /*
    @brief Timer to call the execute_waypoints function periodically.
    */
    rclcpp::TimerBase::SharedPtr timer_;

    /*
    @brief Callback function for receiving waypoints from the ground station.
    @param msg Shared pointer to the Point message received.
    */
    void ground_station_callback(const geometry_msgs::msg::Point::SharedPtr msg);

    /*
    @brief Callback function for receiving calculated waypoints.
    @param msg Shared pointer to the PointStamped message received.
    */
    void calculated_waypoints_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /*
    @brief Callback function for receiving GPS data.
    @param msg Shared pointer to the NavSatFix message received.
    */
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /*
    @brief Executes waypoints by calling the execute_waypoints function from the WaypointQueue instance.
    */
    void execute_waypoints();

    /*
    @brief Gets the current GPS coordinate.
    @return The current position as a Datatypes::Coordinate.
    */
    Datatypes::Coordinate get_curr_coordinate();

    /*
    @brief The current position obtained from the GPS data.
    */
    Datatypes::Coordinate current_position_;
};

#endif // WAYPOINT_QUEUE_NODE_HPP
