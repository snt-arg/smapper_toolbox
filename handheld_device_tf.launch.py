from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transforms for handheld device
        
        # base_link to os_sensor
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_os_sensor_publisher',
            arguments=['0.000000', '0.000000', '0.000000', 
                      '0.000000', '0.000000', '0.000000', '1.000000',
                      'base_link', 'os_sensor']
        ),
        
        # os_sensor to os_lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='os_sensor_to_os_lidar_publisher',
            arguments=['0.000000', '0.000000', '0.038195', 
                      '0.000000', '0.000000', '1.000000', '0.000000',
                      'os_sensor', 'os_lidar']
        ),
        
        # os_sensor to os_imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='os_sensor_to_os_imu_publisher',
            arguments=['-0.002441', '-0.009725', '0.007533', 
                      '0.000000', '0.000000', '0.000000', '1.000000',
                      'os_sensor', 'os_imu']
        ),
        
        # base_link to realsense_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_realsense_link_publisher',
            arguments=['0.077200', '0.017500', '0.030280', 
                      '0.000000', '0.000000', '0.000000', '1.000000',
                      'base_link', 'realsense_link']
        ),
        
        # base_link to front_left
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_front_left_publisher',
            arguments=['0.053048', '0.072026', '-0.027605', 
                      '0.616745', '-0.358952', '0.351072', '-0.606241',
                      'base_link', 'front_left']
        ),
        
        # base_link to front_right
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_front_right_publisher',
            arguments=['0.051858', '-0.064176', '-0.014296', 
                      '-0.354920', '0.613745', '-0.612725', '0.349167',
                      'base_link', 'front_right']
        ),
        
        # base_link to side_left
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_side_left_publisher',
            arguments=['-0.069201', '0.063729', '-0.027022', 
                      '0.711349', '-0.003384', '-0.007293', '-0.702793',
                      'base_link', 'side_left']
        ),
        
        # base_link to side_right
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_side_right_publisher',
            arguments=['-0.070236', '-0.070069', '-0.014109', 
                      '0.004504', '-0.704462', '0.709709', '0.005144',
                      'base_link', 'side_right']
        ),
        
        # realsense_link to realsense_imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='realsense_link_to_realsense_imu_publisher',
            arguments=['-0.057549', '-0.109006', '-0.076735', 
                      '0.506319', '-0.501459', '0.499713', '0.492409',
                      'realsense_link', 'realsense_imu']
        ),
        
    ])
