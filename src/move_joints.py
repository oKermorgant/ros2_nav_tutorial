#!/usr/bin/python
import rclpy
from rclpy.node import Node
from numpy import pi

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from simulation_2d.srv import Spawn
from ros2_nav_tutorial.srv import JointMode


class MoveJointsNode(Node):
    
    mode_servo = JointMode.Request().SERVO

    def __init__(self):        
        super().__init__('move_joints')
        
        self.js = JointState()
        self.js.name = ["wheel", "torso", "neck"]
        self.js.position = [0.,0.,0.]
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)        
        
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10)
        self.cmd_sub  # prevent unused variable warning
        
        self.mode_srv = self.create_service(JointMode, 'joint_mode', self.mode_callback)
        
        manual = self.declare_parameter("mode", 'servo').get_parameter_value().string_value == 'manual'
        self.dt = self.declare_parameter("dt", 0.05).get_parameter_value().double_value
        
        self.mode = manual and JointMode.Request().MANUAL or self.mode_servo
        
        self.spawn()
        
    def mode_callback(self, request, response):
        self.mode = request.mode
        return response
                
    def spawn(self):
        
        robot = self.get_namespace().strip('/')
        
        if robot.startswith('bb'):
            self.radius = .27
        else:
            self.radius = .16
            
        # spawn robot in simulation        
        spawner = self.create_client(Spawn, '/simulator/spawn')
        while not spawner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')
            
        req = Spawn.Request()
        req.robot_namespace = self.get_namespace()
        
        req.radius = self.radius
        req.shape = req.SHAPE_CIRCLE
        
        if robot in ('bb8', 'd0'):
            req.robot_color = [170,170,170]          
                
        if robot == 'bb8':  # actually all default values
            req.x = 0.
            req.y = 0.
            req.theta = 0.
            req.laser_color = [255,0,0]
            
        elif robot == 'bb9':
            req.x = 14.6
            req.y = 15.1
            req.theta = 2.7
            
        elif robot == 'd0':
            req.x = 13.1
            req.y = 14.
            req.theta = 2.7
            req.laser_color = [230,255,0]
        else:   # d9
            req.x = 2.3
            req.y = -7.7
            req.theta = 0.8
            
        self.future = spawner.call_async(req)
        
        
    def cmd_callback(self, msg):
        
        if self.mode == self.mode_servo:
            v = msg.linear.x
            w = msg.angular.z
            self.js.position[0] += v*self.dt/self.radius
            self.js.position[1] = v*pi/12
            self.js.position[2] = w*pi/12
            self.js.header.stamp = self.get_clock().now().to_msg()
            
            self.js_pub.publish(self.js)
        

def main(args=None):
    
    rclpy.init(args=args)

    move_joints = MoveJointsNode()

    rclpy.spin(move_joints)
    
    move_joints.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        
