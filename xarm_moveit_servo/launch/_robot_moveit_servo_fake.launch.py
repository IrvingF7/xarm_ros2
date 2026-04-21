#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file, load_yaml
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip', default='')
    report_type = LaunchConfiguration('report_type', default='normal')
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)

    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')

    teleop_device = LaunchConfiguration('teleop_device', default='gamepad')
    enable_keyboard = LaunchConfiguration('enable_keyboard', default='false')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    # 1: xbox360 wired
    # 2: xbox360 wireless
    # 3: spacemouse wireless
    joystick_type = LaunchConfiguration('joystick_type', default=1)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    moveit_config_package_name = 'xarm_moveit_config'
    controllers_name = 'controllers' if ros2_control_plugin.perform(context) == 'uf_robot_hardware/UFRobotSystemHardware' else 'fake_controllers'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}{}_controllers.yaml'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )

    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        robot_ip=robot_ip,
        report_type=report_type,
        baud_checkset=baud_checkset,
        default_gripper_baud=default_gripper_baud,
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,

    ).to_moveit_configs()

    robot_description_parameters = {}
    robot_description_parameters.update(moveit_config.robot_description)
    robot_description_parameters.update(moveit_config.robot_description_semantic)
    robot_description_parameters.update(moveit_config.robot_description_kinematics)
    robot_description_parameters.update(moveit_config.joint_limits)
    robot_description_parameters.update(moveit_config.planning_pipelines)

    servo_yaml = load_yaml('xarm_moveit_servo', "config/xarm_moveit_servo_config.yaml")
    servo_yaml['move_group_name'] = xarm_type
    xarm_traj_controller = '{}{}_traj_controller'.format(prefix.perform(context), xarm_type)
    servo_yaml['command_out_topic'] = '/{}/joint_trajectory'.format(xarm_traj_controller)
    servo_params = {"moveit_servo": servo_yaml}
    controllers = []
    if add_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
    elif add_bio_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))

    # rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'moveit.rviz'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare('xarm_moveit_servo'), 'rviz', 'servo.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description_parameters,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
            'ros2_control_params': ros2_control_params,
        }.items(),
    )

    traj_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            xarm_traj_controller,
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    # Load controllers
    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    # Launch as much as possible in components
    if teleop_device.perform(context) == "gamepad":
        container = ComposableNodeContainer(
            name='xarm_moveit_servo_container',
            namespace='/',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robot_state_publisher',
                    plugin='robot_state_publisher::RobotStatePublisher',
                    name='robot_state_publisher',
                    parameters=[robot_description_parameters],
                ),
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf2_broadcaster',
                    parameters=[{'child_frame_id': 'link_base', 'frame_id': 'world'}],
                ),
                ComposableNode(
                    package='moveit_servo',
                    plugin='moveit_servo::ServoNode',
                    name='servo_server',
                    parameters=[
                        servo_params,
                        robot_description_parameters,
                    ],
                    # extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='xarm_moveit_servo',
                    plugin='xarm_moveit_servo::JoyToServoPub',
                    name='joy_to_servo_node',
                    parameters=[
                        servo_params,
                        {
                            'dof': dof,
                            'ros_queue_size': 10,
                            'joystick_type': joystick_type,
                        },
                    ],
                    # extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='joy',
                    plugin='joy::Joy',
                    name='joy_node',
                    parameters=[
                        # {'autorepeat_rate': 50.0},
                    ],
                    # extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        )
    else:  # teleop_device == "gello"
        container = ComposableNodeContainer(
            name='xarm_moveit_servo_container',
            namespace='/',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robot_state_publisher',
                    plugin='robot_state_publisher::RobotStatePublisher',
                    name='robot_state_publisher',
                    parameters=[robot_description_parameters],
                ),
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf2_broadcaster',
                    parameters=[{'child_frame_id': 'link_base', 'frame_id': 'world'}],
                ),
                ComposableNode(
                    package='moveit_servo',
                    plugin='moveit_servo::ServoNode',
                    name='servo_server',
                    parameters=[
                        servo_params,
                        robot_description_parameters,
                    ],
                    # extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                package='xarm_moveit_servo',
                plugin='xarm_moveit_servo::GelloToServoPub',
                name='gello_to_servo_node',
                parameters=[
                    servo_params,
                    ],
                ),
                ComposableNode(
                package="xarm_moveit_servo",
                plugin="xarm_moveit_servo::EEPublisher",
                name="ee_publisher",
                parameters=[
                    moveit_config.to_dict(),  # robot_description + semantic
                    {
                        "planning_group": "xarm6",
                        "tcp_link": "link_tcp",
                        "eef_link": "link_eef",
                        "base_frame": "link_base",  # match Servo planning frame
                        "twist_frame": "spatial",    # or "body"
                        "use_fake_hardware": 'true',
                    }
                    ],
                )
            ],
            output='screen',
        )

    # Standalone keyboard node for gello mode (runs in separate terminal for stdin access)
    gello_keyboard_node = Node(
        package='xarm_moveit_servo',
        executable='gello_keyboard_input',
        name='gello_keyboard_servo_node',
        output='screen',
        parameters=[
            {
                'twist_cmd_topic': '/servo_server/delta_twist_cmds',
                'twist_frame': 'link_eef',
                'twist_linear_speed': 0.05,
                'twist_angular_speed': 0.1,
            }
        ],
        prefix='xterm -fa "Monospace" -fs 12 -geometry 100x30 -T "Gello+Keyboard Teleop" -e' if enable_keyboard.perform(context) in ('True', 'true') else '',
        condition=IfCondition(enable_keyboard),
    )

    if add_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),   # URDF + SRDF + kinematics + limits + pipelines
                # ros2_control_params,              # ← gripper-only controllers
                # {'allow_trajectory_execution': True},  # keep execution enabled (we want to command the gripper)
            ],
        )
        gripper_toggler_node = Node(
            package='xarm_moveit_servo',
            executable='xarm_gripper_toggler',
            name='xarm_gripper_toggler',
            output='screen',
            parameters=[robot_description_parameters],
        )
        start_toggler_after_move_group = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=move_group_node,
                on_start=[TimerAction(period=2.0, actions=[gripper_toggler_node])],
            )
        )

        return [  # noqa: RUF005
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=traj_controller_node,
                    on_exit=container,
                )
            ),
            rviz_node,
            joint_state_broadcaster,
            ros2_control_launch,
            traj_controller_node,
            gello_keyboard_node,
        ] + controller_nodes + [move_group_node, start_toggler_after_move_group]

    return [  # noqa: RUF005
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=traj_controller_node,
                on_exit=container,
            )
        ),
        rviz_node,
        joint_state_broadcaster,
        ros2_control_launch,
        traj_controller_node,
        gello_keyboard_node,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
