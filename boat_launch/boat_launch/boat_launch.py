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

    return LaunchDescription([
        as5600_node,
        lis3mdl_node,
        control_node,
        main_logic_node,
        gps_node,
    ])

