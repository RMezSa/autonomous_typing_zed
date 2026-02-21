from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    use_tf_targeting_arg = DeclareLaunchArgument('use_tf_targeting', default_value='true')
    arm_base_frame_arg = DeclareLaunchArgument('arm_base_frame', default_value='arm_base')
    camera_frame_arg = DeclareLaunchArgument('camera_frame', default_value='')
    keyboard_plane_z_arg = DeclareLaunchArgument('keyboard_plane_z_m', default_value='0.45')
    camera_fx_arg = DeclareLaunchArgument('camera_fx', default_value='700.0')
    camera_fy_arg = DeclareLaunchArgument('camera_fy', default_value='700.0')
    camera_cx_arg = DeclareLaunchArgument('camera_cx', default_value='640.0')
    camera_cy_arg = DeclareLaunchArgument('camera_cy', default_value='360.0')
    arm_z_offset_arg = DeclareLaunchArgument('arm_z_offset', default_value='0.0')
    workspace_x_min_arg = DeclareLaunchArgument('workspace_x_min', default_value='-0.10')
    workspace_x_max_arg = DeclareLaunchArgument('workspace_x_max', default_value='0.55')
    workspace_y_min_arg = DeclareLaunchArgument('workspace_y_min', default_value='-0.55')
    workspace_y_max_arg = DeclareLaunchArgument('workspace_y_max', default_value='0.55')
    workspace_z_min_arg = DeclareLaunchArgument('workspace_z_min', default_value='0.02')
    workspace_z_max_arg = DeclareLaunchArgument('workspace_z_max', default_value='0.80')
    motion_enabled_arg = DeclareLaunchArgument('motion_enabled', default_value='false')
    require_transform_valid_arg = DeclareLaunchArgument('require_transform_valid', default_value='true')
    enable_calibration_probe_arg = DeclareLaunchArgument('enable_calibration_probe', default_value='true')

    static_tf_enabled_arg = DeclareLaunchArgument('static_tf_enabled', default_value='false')
    static_tf_parent_frame_arg = DeclareLaunchArgument('static_tf_parent_frame', default_value='arm_base')
    static_tf_child_frame_arg = DeclareLaunchArgument('static_tf_child_frame', default_value='zed2i_left_camera_optical_frame')
    static_tf_x_arg = DeclareLaunchArgument('static_tf_x', default_value='0.0')
    static_tf_y_arg = DeclareLaunchArgument('static_tf_y', default_value='0.0')
    static_tf_z_arg = DeclareLaunchArgument('static_tf_z', default_value='0.0')
    static_tf_roll_arg = DeclareLaunchArgument('static_tf_roll', default_value='0.0')
    static_tf_pitch_arg = DeclareLaunchArgument('static_tf_pitch', default_value='0.0')
    static_tf_yaw_arg = DeclareLaunchArgument('static_tf_yaw', default_value='0.0')

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
            'use_tf_targeting': LaunchConfiguration('use_tf_targeting'),
            'arm_base_frame': LaunchConfiguration('arm_base_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'keyboard_plane_z_m': LaunchConfiguration('keyboard_plane_z_m'),
            'camera_fx': LaunchConfiguration('camera_fx'),
            'camera_fy': LaunchConfiguration('camera_fy'),
            'camera_cx': LaunchConfiguration('camera_cx'),
            'camera_cy': LaunchConfiguration('camera_cy'),
            'arm_z_offset': LaunchConfiguration('arm_z_offset'),
            'workspace_x_min': LaunchConfiguration('workspace_x_min'),
            'workspace_x_max': LaunchConfiguration('workspace_x_max'),
            'workspace_y_min': LaunchConfiguration('workspace_y_min'),
            'workspace_y_max': LaunchConfiguration('workspace_y_max'),
            'workspace_z_min': LaunchConfiguration('workspace_z_min'),
            'workspace_z_max': LaunchConfiguration('workspace_z_max'),
            'motion_enabled': LaunchConfiguration('motion_enabled'),
            'require_transform_valid': LaunchConfiguration('require_transform_valid'),
        }]
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_arm_static_tf',
        condition=IfCondition(LaunchConfiguration('static_tf_enabled')),
        arguments=[
            LaunchConfiguration('static_tf_x'),
            LaunchConfiguration('static_tf_y'),
            LaunchConfiguration('static_tf_z'),
            LaunchConfiguration('static_tf_roll'),
            LaunchConfiguration('static_tf_pitch'),
            LaunchConfiguration('static_tf_yaw'),
            LaunchConfiguration('static_tf_parent_frame'),
            LaunchConfiguration('static_tf_child_frame'),
        ]
    )

    calibration_probe = Node(
        package='zed_aruco',
        executable='calibration_probe',
        name='calibration_probe',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_calibration_probe')),
        parameters=[{
            'arm_base_frame': LaunchConfiguration('arm_base_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'keyboard_plane_z_m': LaunchConfiguration('keyboard_plane_z_m'),
            'camera_fx': LaunchConfiguration('camera_fx'),
            'camera_fy': LaunchConfiguration('camera_fy'),
            'camera_cx': LaunchConfiguration('camera_cx'),
            'camera_cy': LaunchConfiguration('camera_cy'),
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
        use_tf_targeting_arg,
        arm_base_frame_arg,
        camera_frame_arg,
        keyboard_plane_z_arg,
        camera_fx_arg,
        camera_fy_arg,
        camera_cx_arg,
        camera_cy_arg,
        arm_z_offset_arg,
        workspace_x_min_arg,
        workspace_x_max_arg,
        workspace_y_min_arg,
        workspace_y_max_arg,
        workspace_z_min_arg,
        workspace_z_max_arg,
        motion_enabled_arg,
        require_transform_valid_arg,
        enable_calibration_probe_arg,
        static_tf_enabled_arg,
        static_tf_parent_frame_arg,
        static_tf_child_frame_arg,
        static_tf_x_arg,
        static_tf_y_arg,
        static_tf_z_arg,
        static_tf_roll_arg,
        static_tf_pitch_arg,
        static_tf_yaw_arg,
        zed_combined,
        static_tf_publisher,
        coordinator,
        calibration_probe,
    ])
