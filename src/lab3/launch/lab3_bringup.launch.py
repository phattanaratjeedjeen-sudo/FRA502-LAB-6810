from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_description = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim_plus',
        namespace='',
        executable='turtlesim_plus_node.py',
        name='turtlesim'
        )
    
    kill_turtle1 = ExecuteProcess(
        cmd=['ros2 service call /remove_turtle turtlesim/srv/Kill "{name: turtle1}"'],
        shell=True
    )    

    pkg = 'lab3'
    # [executable, namespace, name]
    killer = ['killer.py', 'killer_turtle', 'killer_node'] 
    eater = ['eater.py', 'eater_turtle', 'eater_node']
    sampling_frequency = 100.0

    killer_node = Node(
        package=pkg,
        executable=killer[0],
        namespace=killer[1],
        name=killer[2],
        parameters=[
            {'sampling_frequency': sampling_frequency},
            {'kill_target': eater[1]}
        ],
        remappings=[
            ('/pose', f'/{eater[1]}/pose'),
            ('/pizza_count', f'/{eater[1]}/pizza_count')
        ],
    )
  
    eater_node = Node(
        package=pkg,
        executable=eater[0],
        namespace=eater[1],
        name=eater[2],
        parameters=[
            {'sampling_frequency': sampling_frequency}
        ],
    )

    spawn_killer = ExecuteProcess(
        cmd=[['ros2 service call /spawn_turtle turtlesim/srv/Spawn ',f'"{{x: 5.5, y: 5.5, theta: 0.0, name: \'{killer[1]}\'}}"']],
        shell=True
    )

    spawn_eater = ExecuteProcess(
        cmd=[['ros2 service call /spawn_turtle turtlesim/srv/Spawn ',f'"{{x: 5.5, y: 5.5, theta: 0.0, name: \'{eater[1]}\'}}"']],
        shell=True
    )


    launch_description.add_action(turtlesim_node)
    launch_description.add_action(kill_turtle1)
    launch_description.add_action(spawn_eater)
    launch_description.add_action(spawn_killer)
    launch_description.add_action(killer_node)
    launch_description.add_action(eater_node)

    return launch_description