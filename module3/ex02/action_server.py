import rclpy
import time
import math
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


from action_turtle_commands.action import MessageTurtleCommands


class CommandsActionServer(Node):

    def __init__(self):
        super().__init__('action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'execute_turtle_commands',
            self.execute_callback)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
        self.subscription
        self.flag = 0
        
    
    def listener_callback(self, msg):
    	self.message = msg
    	
    	pass
        #self.get_logger().info('I heard: "%s"' % msg.data)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        #self.get_logger().info(f'I heard: "{self.message}"')
        if not self.flag:
        	x0, y0 = self.message.x, self.message.y
        	self.flag = 1
        
        twist = Twist()
        
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0
        
        if goal_handle.request.command == 'forward':
        	twist.linear.x = 1.0
        	#self.publisher.publish(twist)
        	while feedback_msg.odom != goal_handle.request.s:
        	#while feedback_msg.odom != goal_handle.request.s:
        		#self.get_logger().info(f'I heard: "{self.message}"')
        		self.publisher.publish(twist)
        		feedback_msg.odom = int(math.sqrt((self.message.x - x0)+(self.message.y - y0)))
        		self.get_logger().info(f'I went: {self.message.x} {x0} m')
        		goal_handle.publish_feedback(feedback_msg)
        		time.sleep(1)
        	if feedback_msg.odom == goal_handle.request.s: self.flag = 0
        else:
        	if goal_handle.request.command == 'turn_left':
        		twist.angular.z = float(goal_handle.request.angle)*2*3.14/360
        	else: twist.angular.z = -1.0 * float(goal_handle.request.angle)*2*3.14/360
        	self.publisher.publish(twist)
        	#time.sleep(1)
        		
        goal_handle.succeed()
        
        		
        
        result = MessageTurtleCommands.Result()
        result.result = True
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = CommandsActionServer()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
