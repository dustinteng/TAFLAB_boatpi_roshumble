from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # as5600_node = Node(
    #     package='as5600_sensor',
    #     executable='as5600_node',
    #     name='as5600_node',
    #     output='screen',
    # )

    # lis3mdl_node = Node(
    #     package='lis3mdl_mag',
    #     executable='magnetometer_node',
    #     name='magnetometer_node',
    #     output='screen',
    # )

    # mpu9250_node = Node(
    #     package='mpu9250_sensor',
    #     executable='mpu9250_node',
    #     name='mpu9250_node',
    #     output='screen',
    # )
    
    # pht_sensor_node = Node(
    #     package='pht_sensor',
    #     executable='pht_sensor_node',
    #     name='pht_sensor_node',
    #     output='screen',
    # )
    
    # witmotionmag_node = Node(
    #     package='witmotionmag',
    #     executable='witmotionmag_node',
    #     name='witmotionmag_node',
    #     output='screen',
    # )

    # gps_node = Node(
    #     package='gps_gt_u7',
    #     executable='gps_node',
    #     name='gps_node',
    #     output='screen',
    # )

    # control_node = Node(
    #     package='control_py',
    #     executable='boat_control',
    #     name='boat_control_node',
    #     output='screen',
    # )

    # main_logic_node = Node(
    #     package='boat_logic',
    #     executable='main_logic_node',
    #     name='main_logic_node',
    #     output='screen',
    # )

    # xbee_node = Node(
    #     package='xbee_node',
    #     executable='xbee_node',
    #     name='xbee_node',
    #     output='screen',
    # )

    auto_waypoint_queue = Node(
        package='boat_control_system_final',
        executable='waypoint_queue_node',
        name='auto_waypoint_queue',
        output='screen',
    )

    auto_coordinate_calculations = Node(
        package='boat_control_system_final',
        executable='coordinate_calculations_node',
        name='auto_coordinate_calculations',
        output='screen',
    )

    auto_rudder_control = Node(
        package='boat_control_system_final',
        executable='rudder_servo_control_node',
        name='auto_rudder_control',
        output='screen',
    )

    auto_sail_control = Node(
        package='boat_control_system_final',
        executable='sail_servo_control_node',
        name='auto_sail_control',
        output='screen',
    )

    # Test Publisher node
    test_publisher_node = Node(
        package='test_publisher',
        executable='test_publisher',
        name='test_publisher',
        output='screen',
    )

    return LaunchDescription([
        # as5600_node,
        # # lis3mdl_node,
        # # mpu9250_node,
        # pht_sensor_node,
        # witmotionmag_node,
        # gps_node,
        # control_node,
        # main_logic_node,
        # xbee_node,
        # autonomous_control,
        auto_waypoint_queue,
        auto_coordinate_calculations,
        auto_rudder_control,
        auto_sail_control,
        test_publisher_node,
    ])
