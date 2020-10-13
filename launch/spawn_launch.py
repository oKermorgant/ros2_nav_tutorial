from simple_launch import SimpleLauncher
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    '''    
    Run the given nodes in the robot's namespace
    '''
    
    use_amcl = False

    sl = SimpleLauncher()
    
    sl.declare_arg('robot', 'bb8', description = 'Robot name (bb8, bb9, d0, d9)')
    sl.declare_arg('use_nav', 'false', description = 'Whether to use the nav stack')
    sl.declare_arg('use_js_gui', 'false', description = 'Whether to use the joint states gui')    
    
    robot = sl.arg('robot')
    
    # extract robot type (bb / d)
    robot_type = sl.py_eval("''.join(c for c in '", robot, "' if not c.isdigit())")

    
    with sl.group(ns=robot):
        
        sl.node('ros2_nav_tutorial', 'robot', name='control', output='screen')
        
        with sl.group(if_arg='use_js_gui'):
            js_file = sl.find('ros2_nav_tutorial', 'joint_sliders.yaml')
            sl.node('slider_publisher', 'slider_publisher', name='joint_manual', arguments=[js_file])
        
        # get robot type to generate description
        sl.robot_state_publisher('ros2_nav_tutorial', sl.name_join(robot_type, '.xacro'), 'urdf', xacro_args={'name': robot})
        
        if not use_amcl:
            # replace AMCL with static (perfect) transform
            sl.node('tf2_ros', 'static_transform_publisher', arguments=['0','0','0','0','0','0','map',sl.name_join(robot,'/odom')])
            
        with sl.group(unless_arg='use_nav'):
            cmd_file = sl.find('ros2_nav_tutorial', 'cmd_sliders.yaml')
            sl.node('slider_publisher', 'slider_publisher', name='cmd_vel_manual', arguments=[cmd_file])
        
        
        with sl.group(if_arg='use_nav'):
                        
            # get corresponding radius for obstacle avoidance controllers
            robot_rad = sl.py_eval("'", robot_type, "'=='bb' and .27 or .16")
        
            # nav2 stack
            nav_nodes = [('nav2_controller','controller_server'), 
                        ('nav2_planner','planner_server'),
                        ('nav2_recoveries','recoveries_server'),
                        ('nav2_bt_navigator','bt_navigator'),
                        #('nav2_waypoint_follower','waypoint_follower')
                        ]
    
            if use_amcl:
                nav_nodes.append(('nav2_amcl','amcl'))
            
            # link prefixes are updated with robot name
            rewrites = {'global_frame': sl.name_join(robot,'/odom'), 
                        'base_frame_id': sl.name_join(robot,'/base_link'),
                        'robot_base_frame': sl.name_join(robot,'/base_link'),
                        'default_bt_xml_filename': sl.find('nav2_bt_navigator','navigate_w_replanning_time.xml'),
                        'robot_radius': robot_rad, 
                        'topic': sl.name_join('/',robot, '/scan')
                        }
            
            # load param file into namespace
            configured_params = RewrittenYaml(
                            source_file=sl.find('ros2_nav_tutorial', 'param.yaml'),
                            root_key=robot,
                            param_rewrites=rewrites,
                            convert_types=True)
            remappings={'map_topic': '/map'}.items()
            
            # launch navigation nodes
            lifecycle_nodes = []
            for pkg,executable in nav_nodes:
                sl.node(pkg, executable,name=executable,
                    parameters=[configured_params],
                    remappings=remappings)
                
                lifecycle_nodes.append(executable)

            sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}])
                    
    return sl.launch_description()
