import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    slam_node = Node(
        package='tb3_factor_graph_slam',
        executable='factor_graph_slam_node',
        name='factor_graph_slam_node',
        output='screen',
        parameters=[{
            'odom_topic': '/odom_noisy',
            'scan_topic': '/scan',
            'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
            'keyframe_linear_threshold': 0.3,
            'keyframe_angular_threshold': 0.3,
            'loop_closure_search_radius': 1.0,
            'loop_closure_min_index_gap': 20,
            'loop_closure_fitness_threshold': 0.05,
            'icp_fitness_threshold': 0.08,
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
        world_launch,
        slam_node,
        rviz_node,
    ])