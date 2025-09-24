from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # Launch turtle controller
        Node(
            package='ninja_turtle',
            executable='turtle_controller',
            name='turtle_controller'
        ),

        #Launch voice listener for speech recognition
        Node(
            package='ninja_turtle',
            executable='voice_listener',
            name='voice_listener'
        ),

        # Launch game manager for interactive drawing
        Node(
            package='ninja_turtle',
            executable='game_manager',
            name='game_manager',
            output='screen'
        ),
    ])
