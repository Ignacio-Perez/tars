from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Share folder
    tars_share = get_package_share_directory('tars')

    # Files
    map_yaml = os.path.join(tars_share, 'scenarios', 'ccia_h.yaml')
    scenario_yaml = os.path.join(tars_share, 'scenarios', 'ccia_h.scenario.yaml')
    rviz_config = os.path.join(tars_share, 'rviz', 'navigation.rviz')  

  
    return LaunchDescription([

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}]
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'node_names': ['map_server']},
                {'autostart': True}
            ]
        ),

        # TARS node
        Node(
            package='tars',
            executable='tars_node',
            name='tars_node',
            output='screen',
            parameters=[{'scenario': scenario_yaml}]
        ),

        # TARS SFM_CONTROL
        Node(
            package='tars',
            executable='tars_sfm_control',
            name='tars_sfm_control',
            output='screen',
            parameters=[{'goal_topic': '/tars_sfm_control/r09/goal'}]
        ),

        # TARS NAVIGATION
        Node(
            package='tars',
            executable='tars_navigation',
            name='tars_navigation',
            output='screen',
            parameters=[{'local_goal_topic': '/tars_sfm_control/r09/goal'}]
        ),

        # RViz2 with fixed frame and config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
