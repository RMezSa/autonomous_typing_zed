from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    zed_launch_package_arg = DeclareLaunchArgument(
        'zed_launch_package',
        default_value='zed_wrapper',
        description='ZED launch package name'
    )

    zed_launch_file_arg = DeclareLaunchArgument(
        'zed_launch_file',
        default_value='zed_camera.launch.py',
        description='ZED launch file name'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Image topic for ArUco node'
    )

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model passed to the ZED launch file'
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('zed_launch_package')),
                'launch',
                LaunchConfiguration('zed_launch_file')
            ])
        ),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model')
        }.items()
    )
    
    # ArUco detection node
    aruco_node = Node(
        package='aruco_py',
        executable='aruco_node',
        name='aruco_node',
        parameters=[{'image_topic': LaunchConfiguration('image_topic')}],
        output='screen'
    )
    
    return LaunchDescription([
        zed_launch_package_arg,
        zed_launch_file_arg,
        image_topic_arg,
        camera_model_arg,
        zed_launch,
        aruco_node,
    ])
