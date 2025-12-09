from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_description = LaunchDescription()

    interface_type = 'lab1_interfaces/srv/SetNoise'
    noise_properties = [
        ('linear', 1.0, 0.1),
        ('angular', 0.0, 3.0)
    ]
    
    for ns, mean, var in noise_properties:
        set_noise = ExecuteProcess(
            cmd=[['ros2 service call ',str(ns),'/set_noise ',interface_type,' ',f'"{{mean: {{data: {mean}}},variance: {{data: {var}}}}}"']],
            shell=True
        )

    rate = LaunchConfiguration('rate')
    rate_launch_arg = DeclareLaunchArgument(
        'rate',
        default_value='5.5'
    )
    
    turtlesim_node = Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim'
        )
    

    package_name = 'lab1'
    executable_noise = 'noise_generator.py'
    namespace = ['linear', 'angular']
    rate = [9.5, 7.5]
    for i in range(len(namespace)):
        noise_gen = Node(
            package=package_name,
            namespace=namespace[i],
            executable=executable_noise,
            name=namespace[i] + '_noise',
            parameters=[
                {'rate': rate[i]}
            ]
        )
        

    velo_mux = Node(
        package=package_name,
        namespace='',
        executable='velocity_mux.py',
        name='mux',
        remappings=[('/cmd_vel', 'turtle1/cmd_vel')],
        parameters=[{'rate':25.0}]
        )
    
    launch_description.add_action(turtlesim_node)
    launch_description.add_action(noise_gen)
    launch_description.add_action(rate_launch_arg)
    launch_description.add_action(velo_mux)
    launch_description.add_action(set_noise)

    return launch_description