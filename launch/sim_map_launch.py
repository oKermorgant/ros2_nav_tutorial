from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('map', 'batS.yaml', description = 'Name of YAML map file')
    map_file = sl.find('ros2_nav_tutorial', file_name = sl.arg('map'), file_dir = 'maps')
        
    sl.node(package='ros2_nav_tutorial', executable='simulation', name = 'simulation', parameters={'map': map_file})
    
    
    # also run map server as lifecycle node
    sl.node('nav2_map_server','map_server',name='map_server',
            parameters=[{'yaml_filename': map_file}])
    sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager_map',
    output='screen',
    parameters=[{'autostart': True},
                {'node_names': ['map_server']}])
    
    # run RViz2
    rviz_config_file = sl.find('ros2_nav_tutorial', 'config.rviz')
    sl.node('rviz2','rviz2', output='log', arguments=['-d', rviz_config_file])
        
    return sl.launch_description()
