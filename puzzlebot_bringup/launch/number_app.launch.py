from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    number_publisher_node = Node(
        package = "minichallenges",
        executable = "publisher_node"
    )
    
    number_subscriber_node = Node(
        package = "minichallenges",
        executable = "subscriber_node"
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_subscriber_node)
    return ld
    