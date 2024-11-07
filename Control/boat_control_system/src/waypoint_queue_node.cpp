#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "WaypointQueue.hpp"  // Include your custom waypoint queue header

class WaypointQueueNode : public rclcpp::Node
{
public:
    WaypointQueueNode() : Node("waypoint_queue_node")
    {
        // Subscription to receive waypoints from the ground station
        ground_station_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/ground_targets", 10,
            std::bind(&WaypointQueueNode::ground_station_callback, this, std::placeholders::_1));

        // Subscription to receive calculated waypoints from coordinate_calculations
        calculated_waypoints_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/calculated_waypoints", 10,
            std::bind(&WaypointQueueNode::calculated_waypoints_callback, this, std::placeholders::_1));

        // Publisher to send waypoints to servo_control node
        servo_control_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/servo_control", 10);

        // Initialize the waypoint queue in autonomous mode
        WaypointQueue::getInstance().initialize_autonomous_mode();
    }

    ~WaypointQueueNode()
    {
        WaypointQueue::getInstance().close_autonomous_mode();
    }

private:
    // ROS 2 communication members
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ground_station_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr calculated_waypoints_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr servo_control_publisher_;

    void ground_station_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // Convert the received point to a Datatypes::Coordinate and add it to the queue
        Datatypes::Coordinate waypoint;
        waypoint.latitude = msg->x;
        waypoint.longitude = msg->y;
        WaypointQueue::getInstance().add_waypoint(waypoint);
        RCLCPP_INFO(this->get_logger(), "Added waypoint from ground station to queue: [%.6f, %.6f]",
                    waypoint.latitude, waypoint.longitude);
    }

    void calculated_waypoints_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Convert the received point to a Datatypes::Coordinate and add it to the front of the queue
        Datatypes::Coordinate waypoint;
        waypoint.latitude = msg->point.x;
        waypoint.longitude = msg->point.y;
        WaypointQueue::getInstance().add_front_waypoint(waypoint);
        RCLCPP_INFO(this->get_logger(), "Added calculated waypoint to the front of the queue: [%.6f, %.6f]",
                    waypoint.latitude, waypoint.longitude);
    }

    void execute_waypoints()
    {
        // Call execute_waypoints from the WaypointQueue class
        WaypointQueue::getInstance().execute_waypoints();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointQueueNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
