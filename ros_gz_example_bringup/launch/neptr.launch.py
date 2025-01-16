import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_ros_gz_example_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_ros_gz_example_description = get_package_share_directory('ros_gz_example_description')
    
    return LaunchDescription([
        # Load robot description
        DeclareLaunchArgument(
            'robot_description',
            default_value=os.path.join(pkg_ros_gz_example_description, 'models/neptr/neptr.urdf'),
            description='Path to the robot description file'
        ),
        
        # Start Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v 4',
                 os.path.join(pkg_ros_gz_example_description, 'models/neptr/model.sdf')],
            output='screen'
        ),
        
        # Bridge between Ignition and ROS
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(pkg_ros_gz_example_bringup, 'config/ros_gz_example_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }]
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
        ),
        
        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_ros_gz_example_bringup, 'config/neptr.rviz')],
            output='screen'
        ),
    ])
