import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_desc   = get_package_share_directory('cartpole_description')
    pkg_gazebo  = get_package_share_directory('cartpole_gazebo')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'cartpole.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()
    world_file = os.path.join(pkg_gazebo, 'worlds', 'cartpole.world')

    # GUI mode
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1',
                        'MESA_GL_VERSION_OVERRIDE': '3.3'}
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn at 15s — GUI fully loaded by then
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'cartpole', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.15'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/cartpole_world/model/cartpole/joint_state'
            '@sensor_msgs/msg/JointState'
            '[gz.msgs.Model',
            '/model/cartpole/joint/slider_joint/cmd_force'
            '@std_msgs/msg/Float64'
            ']gz.msgs.Double',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # Controller starts 10s after spawn = t=25s total
    controller = TimerAction(
        period=25.0,
        actions=[Node(
            package='cartpole_control',
            executable='lqr_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        TimerAction(period=15.0, actions=[spawn]),
        TimerAction(period=16.0, actions=[bridge]),
        controller,
    ])
