from simple_launch import SimpleLauncher
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    '''    
    Run the given nodes in the robot's namespace
    '''
    
    sl = SimpleLauncher()
    
    with sl.group(ns='bb8'):
        
        configured_params = RewrittenYaml(
                source_file=sl.find('ros2_nav_tutorial', 'xbox.yaml'),
                root_key='bb8',
                convert_types=True,
                param_rewrites={})
        
        sl.node('joy','joy_node',
                parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])
                
        sl.node('teleop_twist_joy', 'teleop_node', name='teleop_twist_joy_node', parameters=[configured_params])
        
    return sl.launch_description()
