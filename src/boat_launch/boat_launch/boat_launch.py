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
        executable='magnetometer_node',  
        name='magnetometer_node',
        output='screen',
    )

    mpu9250_node = Node(
        package='mpu9250_sensor',
        executable='mpu9250_node', 
        name='mpu9250_node',
        output='screen',
    )
    
    pht_sensor_node = Node(
        package='pht_sensor',
        executable='pht_sensor_node',  
        name='pht_sensor_node',
        output='screen',
    )
    
    witmotionmag_node = Node(
        package='witmotionmag',
        executable='witmotionmag_node', 
        name='witmotionmag_node',
        output='screen',
    )

    gps_node = Node(
        package='gps_gt_u7',
        executable='gps_node',  
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

    autonomous_control = Node(
        package='autonomous_control',
        executable='controller',
        name='controller',
        output='screen',
    )



    return LaunchDescription([
        as5600_node,
        # lis3mdl_node,
        pht_sensor_node,
        control_node,
        main_logic_node,
        gps_node,
        xbee_node,
        autonomous_control,
        # mpu9250_node,
        witmotionmag_node,
    ])

