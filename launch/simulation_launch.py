from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('map', default_value=sl.find('ros2_nav_tutorial', 'batS.yaml'))
    sl.declare_arg('map_server', default_value=True)

    sl.include('simulation_2d', 'simulation2d_launch.py', launch_arguments= sl.arg_map(['map','map_server']).items())
    
    # run RViz2
    rviz_config_file = sl.find('ros2_nav_tutorial', 'config.rviz')
    sl.node('rviz2','rviz2', output='log', arguments=['-d', rviz_config_file])
    
    return sl.launch_description()
