from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    zed_aruco_share = get_package_share_directory('zed_aruco')

    camera_model_arg = DeclareLaunchArgument('camera_model', default_value='zed2i')
    camera_name_arg = DeclareLaunchArgument('camera_name', default_value='zed2i')
    marker_size_arg = DeclareLaunchArgument('marker_size', default_value='0.1')
    dictionary_arg = DeclareLaunchArgument('aruco_dictionary', default_value='DICT_4X4_50')

    target_z_arg = DeclareLaunchArgument('target_z', default_value='0.12')
    target_roll_arg = DeclareLaunchArgument('target_roll', default_value='0.0')
    target_pitch_arg = DeclareLaunchArgument('target_pitch', default_value='-75.0')
    min_conf_arg = DeclareLaunchArgument('min_confidence', default_value='0.7')

    zed_combined = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_aruco_share, 'launch', 'zed_combined.launch.py')
        ),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
            'marker_size': LaunchConfiguration('marker_size'),
            'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
        }.items()
    )

    coordinator = Node(
        package='zed_aruco',
        executable='typing_coordinator',
        name='typing_coordinator',
        output='screen',
        parameters=[{
            'target_z': LaunchConfiguration('target_z'),
            'target_roll': LaunchConfiguration('target_roll'),
            'target_pitch': LaunchConfiguration('target_pitch'),
            'min_confidence': LaunchConfiguration('min_confidence'),
        }]
    )

    return LaunchDescription([
        camera_model_arg,
        camera_name_arg,
        marker_size_arg,
        dictionary_arg,
        target_z_arg,
        target_roll_arg,
        target_pitch_arg,
        min_conf_arg,
        zed_combined,
        coordinator,
    ])
