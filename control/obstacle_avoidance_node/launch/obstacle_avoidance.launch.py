from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',  # użyj `component_container_mt` dla wielowątkowego kontenera
        composable_node_descriptions=[
            ComposableNode(
                package='obstacle_avoidance_node',
                plugin='obstacle_avoidance::ObstacleAvoidance',
                name='obstacle_avoidance_node'
            )
        ],
        output='screen',
    )
    # Węzeł odpalający skrypt Pythona
    obstacle_detection_node = Node(
        package='obstacle_avoidance_node',
        executable='obstacle_detection.py',
        name='obstacle_detection',
        output='screen'
    )

    
    return LaunchDescription([
        container,
        obstacle_detection_node
    ])
