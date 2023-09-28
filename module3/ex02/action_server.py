import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

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

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        twist = Twist()
        
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0
        
        if goal_handle.request.command == 'forward':
        	twist.linear.x = 1.0
        	#self.publisher.publish(twist)
        	while feedback_msg.odom != goal_handle.request.s:
        		self.publisher.publish(twist)
        		feedback_msg.odom += 1
        		goal_handle.publish_feedback(feedback_msg)
        		time.sleep(1)
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
