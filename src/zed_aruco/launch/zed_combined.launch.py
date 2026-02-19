import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    zed_wrapper_pkg = get_package_share_directory('zed_wrapper')
    
    # Declare arguments
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='zed2i',
        description='ZED camera name (also used as namespace)'
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
    
    # ZED launch file
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_pkg, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
        }.items()
    )
    
    # ArUco detection node
    # Try the rgb/color/rect/image which was seen in your logs
    image_topic_expr = PythonExpression([
        "'/' + '", LaunchConfiguration('camera_name'), "' + '/zed_node/rgb/color/rect/image'"
    ])

    aruco_node = Node(
        package='zed_aruco',
        executable='zed_aruco_node',
        name='zed_aruco_node',
        parameters=[{
            'image_topic': image_topic_expr,
            'marker_size': LaunchConfiguration('marker_size'),
            'aruco_dictionary': LaunchConfiguration('aruco_dictionary')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_model_arg,
        camera_name_arg,
        marker_size_arg,
        dictionary_arg,
        zed_launch,
        aruco_node,
    ])
