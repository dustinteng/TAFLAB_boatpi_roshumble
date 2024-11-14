from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    as5600_node = Node(
        package='as5600_sensor',
        executable='as5600_node',
        name='as5600_node',
        output='screen',
    )

    lis3mdl_node = Node(
        package='lis3mdl_mag',
        executable='magnetometer_node',  # Replace with actual executable name
        name='magnetometer_node',
        output='screen',
    )

    gps_node = Node(
        package='gps_gt_u7',
        executable='gps_node',  # Replace with actual executable name
        name='gps_node',
        output='screen',
    )

    control_node = Node(
        package='control_py',
        executable='boat_control',
        name='boat_control_node',
        output='screen',
    )

    main_logic_node = Node(
        package='boat_logic',
        executable='main_logic_node',
        name='main_logic_node',
        output='screen',
    )
    xbee_node = Node(
        package='xbee_node',
        executable='xbee_node',
        name='xbee_node',
        output='screen',
    )

    waypoint_queue = Node(
        package='boat_control_system_final',
        executable='waypoint_queue_node',
        name='waypoint_queue_node',
        output='screen',
    )

    coordinate_calculations = Node(
        package='boat_control_system_final',
        executable='coordinate_calculations_node',
        name='coordinate_calculations_node',
        output='screen',
    )

    rudder_control = Node(
        package='boat_control_system_final',
        executable='rudder_servo_control_node',
        name='rudder_servo_control_node',
        output='screen',
    )

    sail_control = Node(
        package='boat_control_system_final',
        executable='sail_servo_control_node',
        name='sail_servo_control_node',
        output='screen',
    )

    return LaunchDescription([
        as5600_node,
        lis3mdl_node,
        control_node,
        main_logic_node,
        gps_node,
        xbee_node,
        # waypoint_queue,
        # coordinate_calculations,
        # rudder_control,
        # sail_control,
    ])

