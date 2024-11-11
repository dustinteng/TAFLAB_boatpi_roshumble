#include "waypoint_queue_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "Waypoint_Queue.hpp"

WaypointQueueNode::WaypointQueueNode() : Node("waypoint_queue_node")
{
    // Subscription to receive waypoints from the ground station
    ground_station_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/ground_targets", 10,
        std::bind(&WaypointQueueNode::ground_station_callback, this, std::placeholders::_1));

    // Subscription to receive calculated waypoints from coordinate_calculations
    calculated_waypoints_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/calculated_waypoints", 10,
        std::bind(&WaypointQueueNode::calculated_waypoints_callback, this, std::placeholders::_1));

    // Subscription for GPS data
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps_data", 10,
        std::bind(&WaypointQueueNode::gps_callback, this, std::placeholders::_1));

    // Publisher to send waypoints to servo_control node
    servo_control_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/servo_control", 10);

    // Initialize the waypoint queue in autonomous mode
    WaypointQueue::getInstance().initialize_autonomous_mode();

    // Timer to call execute_waypoints periodically
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),  // Adjust interval as needed
        std::bind(&WaypointQueueNode::execute_waypoints, this)
    );

    // Initialize the current position (optional)
    current_position_ = Datatypes::Coordinate{};
}

WaypointQueueNode::~WaypointQueueNode()
{
    WaypointQueue::getInstance().close_autonomous_mode();
}

void WaypointQueueNode::ground_station_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // Convert the received point to a Datatypes::Coordinate and add it to the queue
    Datatypes::Coordinate waypoint;
    waypoint.latitude = msg->x;
    waypoint.longitude = msg->y;
    WaypointQueue::getInstance().add_waypoint(waypoint);
    RCLCPP_INFO(this->get_logger(), "Added waypoint from ground station to queue: [%.6f, %.6f]",
                waypoint.latitude, waypoint.longitude);
}

void WaypointQueueNode::calculated_waypoints_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Convert the received point to a Datatypes::Coordinate and add it to the front of the queue
    Datatypes::Coordinate waypoint;
    waypoint.latitude = msg->point.x;
    waypoint.longitude = msg->point.y;
    WaypointQueue::getInstance().add_front_waypoint(waypoint);
    RCLCPP_INFO(this->get_logger(), "Added calculated waypoint to the front of the queue: [%.6f, %.6f]",
                waypoint.latitude, waypoint.longitude);
}

void WaypointQueueNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    current_position_.latitude = msg->latitude;
    current_position_.longitude = msg->longitude;
    RCLCPP_INFO(this->get_logger(), "Received GPS data: [%.6f, %.6f]", msg->latitude, msg->longitude);
}

void WaypointQueueNode::execute_waypoints()
{
    // Call execute_waypoints from the WaypointQueue class
    WaypointQueue::getInstance().execute_waypoints(get_curr_coordinate());
    RCLCPP_INFO(this->get_logger(), "Executed waypoints from the queue");
}

Datatypes::Coordinate WaypointQueueNode::get_curr_coordinate()
{
    return current_position_;  // Use updated GPS data from the subscriber
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointQueueNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
