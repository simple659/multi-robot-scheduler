"""
launch.py
─────────────────────────────────────────────────────────────────
系统唯一启动入口。

一条命令启动整个系统：
  ros2 launch multi_robot_scheduler launch.py

启动顺序：
  1. Gazebo 仿真环境（加载 TurtleBot3 多机器人世界）
  2. 每台机器人的 Nav2 导航栈（robot_state_publisher + nav2_bringup）
  3. 调度节点（scheduler_node）
  4. 三个机器人节点（robot_node × 3，各自独立进程）

【为什么 Gazebo 和 Nav2 要在 launch 里启动】
  在真实项目里，launch 文件负责把所有依赖拉起来，
  不需要手动开 4 个终端。这是 ROS2 的标准做法。

  这里 Gazebo 和 Nav2 以 IncludeLaunchDescription 方式引入，
  依赖 turtlebot3_gazebo 和 nav2_bringup 两个包提供的 launch 文件。
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    # ── 外部包路径 ──
    tb3_gazebo   = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # ── 参数：机器人型号（burger / waffle） ──
    robot_model = LaunchConfiguration('robot_model', default='burger')

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_model', default_value='burger',
            description='TurtleBot3 型号: burger 或 waffle'
        ),

        # ════════════════════════════════════
        # 1. 启动 Gazebo 仿真环境
        #    加载多机器人仓库世界地图
        # ════════════════════════════════════
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # ════════════════════════════════════
        # 2. 启动三台机器人的 Nav2 导航栈
        #    每台机器人用 namespace 隔离（robot_a / robot_b / robot_c）
        #    Nav2 会为每台机器人起：
        #      - amcl（定位）
        #      - planner_server（全局路径规划）
        #      - controller_server（局部控制器）
        #      - /{robot_id}/navigate_to_pose action server
        # ════════════════════════════════════

        # Robot A
        GroupAction([
            PushRosNamespace('robot_a'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'namespace':    'robot_a',
                    'use_namespace': 'true',
                    'params_file':  os.path.join(
                        get_package_share_directory('multi_robot_scheduler'),
                        'config', 'nav2_params.yaml'     # Nav2 参数文件
                    ),
                }.items(),
            ),
        ]),

        # Robot B
        GroupAction([
            PushRosNamespace('robot_b'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'namespace':    'robot_b',
                    'use_namespace': 'true',
                    'params_file':  os.path.join(
                        get_package_share_directory('multi_robot_scheduler'),
                        'config', 'nav2_params.yaml'
                    ),
                }.items(),
            ),
        ]),

        # Robot C
        GroupAction([
            PushRosNamespace('robot_c'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'namespace':    'robot_c',
                    'use_namespace': 'true',
                    'params_file':  os.path.join(
                        get_package_share_directory('multi_robot_scheduler'),
                        'config', 'nav2_params.yaml'
                    ),
                }.items(),
            ),
        ]),

        # ════════════════════════════════════
        # 3. 延迟 5 秒后启动我们的节点
        #    确保 Gazebo 和 Nav2 都已就绪
        # ════════════════════════════════════
        TimerAction(
            period=5.0,
            actions=[

                # 调度节点（全局唯一，无 namespace）
                Node(
                    package='multi_robot_scheduler',
                    executable='scheduler_node',
                    name='scheduler_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{'use_sim_time': True}],
                ),

                # 机器人节点 A
                Node(
                    package='multi_robot_scheduler',
                    executable='robot_node',
                    name='robot_node_a',
                    namespace='robot_a',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'robot_id': 'robot_a',
                        'start_x':  0.0,
                        'start_y':  0.0,
                        'use_sim_time': True,
                    }],
                ),

                # 机器人节点 B
                Node(
                    package='multi_robot_scheduler',
                    executable='robot_node',
                    name='robot_node_b',
                    namespace='robot_b',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'robot_id': 'robot_b',
                        'start_x':  9.0,
                        'start_y':  0.0,
                        'use_sim_time': True,
                    }],
                ),

                # 机器人节点 C
                Node(
                    package='multi_robot_scheduler',
                    executable='robot_node',
                    name='robot_node_c',
                    namespace='robot_c',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'robot_id': 'robot_c',
                        'start_x':  4.0,
                        'start_y':  9.0,
                        'use_sim_time': True,
                    }],
                ),
            ]
        ),
    ])
