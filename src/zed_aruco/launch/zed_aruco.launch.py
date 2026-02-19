import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package path
    zed_aruco_pkg = FindPackageShare('zed_aruco')

    # Declare arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/zed2i/zed_node/left/image_rect_color',
        description='Image topic to subscribe to'
    )
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.1',
        description='Size of the ArUco marker in meters'
    )
    
    dictionary_arg = DeclareLaunchArgument(
        'aruco_dictionary',
        default_value='DICT_4X4_50',
        description='ArUco dictionary name'
    )

    # Launch the ZED node if requested (optional)
    # By default, we assume it's already running or launched separately
    # But we can include it here for convenience if the user wants.
    
    # ArUco detection node
    aruco_node = Node(
        package='zed_aruco',
        executable='zed_aruco_node',
        name='zed_aruco_node',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'marker_size': LaunchConfiguration('marker_size'),
            'aruco_dictionary': LaunchConfiguration('aruco_dictionary')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        image_topic_arg,
        marker_size_arg,
        dictionary_arg,
        aruco_node,
    ])
