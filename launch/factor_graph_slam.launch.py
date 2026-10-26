import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    slam_pkg_dir = get_package_share_directory('tb3_factor_graph_slam')
    rviz_config_path = os.path.join(slam_pkg_dir, 'config', 'factor_graph_slam.rviz')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Ground truth is optional -- only used for the raw/optimized drift
    # metrics. Requires the libgazebo_ros_p3d.so plugin set up in project
    # 1's README. If you skip that setup, the SLAM graph and loop closures
    # still work fine; you just won't get the drift comparison topics.
    ground_truth_topic_arg = DeclareLaunchArgument(
        'ground_truth_topic',
        default_value='/ground_truth/state',
        description='nav_msgs/Odometry ground truth topic (optional, for drift metrics)',
    )

    # When true, tb3_odom_noise republishes /odom with synthetic drift on
    # /odom_noisy and the SLAM node subscribes to that instead of /odom.
    # Requires the tb3_odom_noise package to be built and sourced.
    use_noisy_odom_arg = DeclareLaunchArgument(
        'use_noisy_odom',
        default_value='true',
        description='Inject synthetic odometry drift (true/false)',
    )

    # The optimized and raw-odometry maps are published side by side so the
    # difference is visible in one RViz view. The raw map is shifted +x by
    # raw_map_x_offset (display only; the paths are not shifted). Set it to
    # 0.0 to overlay the two maps in the same place instead.
    raw_map_x_offset_arg = DeclareLaunchArgument(
        'raw_map_x_offset',
        default_value='13.0',
        description='Display shift (m) of the raw-odometry map along x',
    )

    odom_topic = PythonExpression([
        "'/odom_noisy' if '", LaunchConfiguration('use_noisy_odom'),
        "' == 'true' else '/odom'",
    ])

    slam_node = Node(
        package='tb3_factor_graph_slam',
        executable='factor_graph_slam_node',
        name='factor_graph_slam_node',
        output='screen',
        parameters=[{
            'odom_topic': odom_topic,
            'scan_topic': '/scan',
            'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
            'keyframe_linear_threshold': 0.3,
            'keyframe_angular_threshold': 0.3,
            'loop_closure_search_radius': 1.0,
            'loop_closure_min_index_gap': 20,
            'loop_closure_fitness_threshold': 0.05,
            'icp_fitness_threshold': 0.08,
            'build_map': True,
            'map_resolution': 0.05,
            'map_width': 12.0,
            'map_height': 12.0,
            'map_origin_x': -6.0,
            'map_origin_y': -6.0,
            'lidar_offset_x': -0.064,  # waffle_pi; use -0.032 for burger
            'raw_map_x_offset': ParameterValue(
                LaunchConfiguration('raw_map_x_offset'), value_type=float),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        ground_truth_topic_arg,
        use_noisy_odom_arg,
        raw_map_x_offset_arg,
        world_launch,
        slam_node,
        rviz_node,
    ])