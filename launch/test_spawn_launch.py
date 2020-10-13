from simple_launch import SimpleLauncher
from nav2_common.launch import RewrittenYaml
from launch.substitutions import PythonExpression


def generate_launch_description():
    '''    
    Run the given nodes in the robot's namespace
    '''
    
    robots = ['bb8']
    use_amcl = False

    sl = SimpleLauncher()
    
    sl.declare_arg('robot', 'bb8', description = 'Robot name (bb8, bb9, d0, d9)')
    robot = sl.arg('robot')
    robot_type = PythonExpression(["''.join(c for c in '", robot, "' if not c.isdigit())"])
    robot_rad = PythonExpression(["'", robot_type, "'=='bb' and .027 or .016"])
    
    sl.robot_state_publisher('ros2_nav_tutorial', sl.name_join(robot_type, '.xacro'), 'urdf', xacro_args={'name': robot, 'radius': robot_rad})
    
    return sl.launch_description()
