import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('puzzlebot_bringup'),
        'config',
        'path.yaml'
    )
    
    fantasticfive_pathGenerator = Node(
        package = "fantasticfive",
        executable = "pathGenerator",
        name = "path",
        parameters = [config]
    )

    fantasticfive_Controller = Node(
        package = "fantasticfive",
        executable = "controller",
        name = "time",
        parameters = [config]
    )

    fantasticfive_Localisation = Node(
        package = "fantasticfive",
        executable = "localisation"
    )

    ld.add_action(fantasticfive_pathGenerator)
    ld.add_action(fantasticfive_Controller)
    #ld.add_action(fantasticfive_Localisation)
   
    return ld
    