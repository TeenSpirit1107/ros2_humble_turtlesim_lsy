from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        output="own_log"
    )

    turtle_spawner_node  = Node(
        package="turtle_motion",
        executable="turtle_spawner"
    )

    turt_ctrl_node =Node(
        package = 'turtle_motion',
        executable = 'go_to_goal'
    )


    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turt_ctrl_node)
    # ld.add_action(turtle_teleop_key)

    return ld
