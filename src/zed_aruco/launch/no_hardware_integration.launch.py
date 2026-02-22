from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    text_arg = DeclareLaunchArgument('text', default_value='test')
    loop_text_arg = DeclareLaunchArgument('loop_text', default_value='true')
    result_mode_arg = DeclareLaunchArgument('result_mode', default_value='success')
    motion_enabled_arg = DeclareLaunchArgument('motion_enabled', default_value='true')
    accept_dry_run_arg = DeclareLaunchArgument('accept_dry_run_result', default_value='false')
    min_conf_arg = DeclareLaunchArgument('min_confidence', default_value='0.7')
    servo_mode_enabled_arg = DeclareLaunchArgument('servo_mode_enabled', default_value='false')
    servo_xy_step_max_arg = DeclareLaunchArgument('servo_xy_step_max_m', default_value='0.003')
    servo_align_enter_arg = DeclareLaunchArgument('servo_align_enter_thresh_px', default_value='8.0')
    servo_align_exit_arg = DeclareLaunchArgument('servo_align_exit_thresh_px', default_value='12.0')
    servo_align_stable_arg = DeclareLaunchArgument('servo_align_stable_cycles', default_value='4')
    servo_cmd_cooldown_arg = DeclareLaunchArgument('servo_cmd_cooldown_sec', default_value='0.08')
    servo_press_step_arg = DeclareLaunchArgument('servo_press_step_m', default_value='0.0015')
    servo_press_max_travel_arg = DeclareLaunchArgument('servo_press_max_travel_m', default_value='0.015')
    servo_press_timeout_arg = DeclareLaunchArgument('servo_press_timeout_sec', default_value='2.0')
    servo_press_direction_arg = DeclareLaunchArgument('servo_press_direction_sign', default_value='-1.0')
    servo_retract_step_arg = DeclareLaunchArgument('servo_retract_step_m', default_value='0.0025')
    emergency_stop_topic_arg = DeclareLaunchArgument('emergency_stop_topic', default_value='keyboard/emergency_stop')

    use_tf_targeting_arg = DeclareLaunchArgument('use_tf_targeting', default_value='false')
    require_transform_valid_arg = DeclareLaunchArgument('require_transform_valid', default_value='false')
    arm_base_frame_arg = DeclareLaunchArgument('arm_base_frame', default_value='arm_base')
    camera_frame_arg = DeclareLaunchArgument('camera_frame', default_value='zed2i_left_camera_optical_frame')
    keyboard_plane_z_arg = DeclareLaunchArgument('keyboard_plane_z_m', default_value='0.45')
    camera_fx_arg = DeclareLaunchArgument('camera_fx', default_value='700.0')
    camera_fy_arg = DeclareLaunchArgument('camera_fy', default_value='700.0')
    camera_cx_arg = DeclareLaunchArgument('camera_cx', default_value='640.0')
    camera_cy_arg = DeclareLaunchArgument('camera_cy', default_value='360.0')
    arm_z_offset_arg = DeclareLaunchArgument('arm_z_offset', default_value='0.0')

    static_tf_enabled_arg = DeclareLaunchArgument('static_tf_enabled', default_value='false')
    static_tf_x_arg = DeclareLaunchArgument('static_tf_x', default_value='0.0')
    static_tf_y_arg = DeclareLaunchArgument('static_tf_y', default_value='0.0')
    static_tf_z_arg = DeclareLaunchArgument('static_tf_z', default_value='0.0')
    static_tf_roll_arg = DeclareLaunchArgument('static_tf_roll', default_value='0.0')
    static_tf_pitch_arg = DeclareLaunchArgument('static_tf_pitch', default_value='0.0')
    static_tf_yaw_arg = DeclareLaunchArgument('static_tf_yaw', default_value='0.0')

    fake_vision = Node(
        package='zed_aruco',
        executable='fake_vision_publisher',
        name='fake_vision_publisher',
        output='screen',
        parameters=[{
            'text': LaunchConfiguration('text'),
            'loop_text': LaunchConfiguration('loop_text'),
            'frame_id': LaunchConfiguration('camera_frame'),
        }],
    )

    fake_action_server = Node(
        package='zed_aruco',
        executable='fake_execute_key_server',
        name='fake_execute_key_server',
        output='screen',
        parameters=[{
            'result_mode': LaunchConfiguration('result_mode'),
        }],
    )

    coordinator = Node(
        package='zed_aruco',
        executable='typing_coordinator',
        name='typing_coordinator',
        output='screen',
        parameters=[{
            'motion_enabled': LaunchConfiguration('motion_enabled'),
            'servo_mode_enabled': LaunchConfiguration('servo_mode_enabled'),
            'accept_dry_run_result': LaunchConfiguration('accept_dry_run_result'),
            'min_confidence': LaunchConfiguration('min_confidence'),
            'use_tf_targeting': LaunchConfiguration('use_tf_targeting'),
            'require_transform_valid': LaunchConfiguration('require_transform_valid'),
            'servo_xy_step_max_m': LaunchConfiguration('servo_xy_step_max_m'),
            'servo_align_enter_thresh_px': LaunchConfiguration('servo_align_enter_thresh_px'),
            'servo_align_exit_thresh_px': LaunchConfiguration('servo_align_exit_thresh_px'),
            'servo_align_stable_cycles': LaunchConfiguration('servo_align_stable_cycles'),
            'servo_cmd_cooldown_sec': LaunchConfiguration('servo_cmd_cooldown_sec'),
            'servo_press_step_m': LaunchConfiguration('servo_press_step_m'),
            'servo_press_max_travel_m': LaunchConfiguration('servo_press_max_travel_m'),
            'servo_press_timeout_sec': LaunchConfiguration('servo_press_timeout_sec'),
            'servo_press_direction_sign': LaunchConfiguration('servo_press_direction_sign'),
            'servo_retract_step_m': LaunchConfiguration('servo_retract_step_m'),
            'emergency_stop_topic': LaunchConfiguration('emergency_stop_topic'),
            'arm_base_frame': LaunchConfiguration('arm_base_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'keyboard_plane_z_m': LaunchConfiguration('keyboard_plane_z_m'),
            'camera_fx': LaunchConfiguration('camera_fx'),
            'camera_fy': LaunchConfiguration('camera_fy'),
            'camera_cx': LaunchConfiguration('camera_cx'),
            'camera_cy': LaunchConfiguration('camera_cy'),
            'arm_z_offset': LaunchConfiguration('arm_z_offset'),
        }],
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
            LaunchConfiguration('arm_base_frame'),
            LaunchConfiguration('camera_frame'),
        ],
    )

    return LaunchDescription([
        text_arg,
        loop_text_arg,
        result_mode_arg,
        motion_enabled_arg,
        accept_dry_run_arg,
        min_conf_arg,
        servo_mode_enabled_arg,
        servo_xy_step_max_arg,
        servo_align_enter_arg,
        servo_align_exit_arg,
        servo_align_stable_arg,
        servo_cmd_cooldown_arg,
        servo_press_step_arg,
        servo_press_max_travel_arg,
        servo_press_timeout_arg,
        servo_press_direction_arg,
        servo_retract_step_arg,
        emergency_stop_topic_arg,
        use_tf_targeting_arg,
        require_transform_valid_arg,
        arm_base_frame_arg,
        camera_frame_arg,
        keyboard_plane_z_arg,
        camera_fx_arg,
        camera_fy_arg,
        camera_cx_arg,
        camera_cy_arg,
        arm_z_offset_arg,
        static_tf_enabled_arg,
        static_tf_x_arg,
        static_tf_y_arg,
        static_tf_z_arg,
        static_tf_roll_arg,
        static_tf_pitch_arg,
        static_tf_yaw_arg,
        fake_vision,
        fake_action_server,
        static_tf_publisher,
        coordinator,
    ])
