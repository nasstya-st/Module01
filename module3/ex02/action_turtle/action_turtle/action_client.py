import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_turtle_commands.action import MessageTurtleCommands


class CommandsActionClient(Node):

    def __init__(self):
        super().__init__('action_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'execute_turtle_commands')
        self.isdone = 0

    def send_goal(self, command, s, angle):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = command
        goal_msg.s = s
        goal_msg.angle = angle

        self._action_client.wait_for_server()
        
        #return self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback) 
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future



    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        #self.isdone = 0

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        self.isdone += 1
        #rclpy.shutdown()
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.odom))


def main(args=None):
    rclpy.init(args=args)

    action_client = CommandsActionClient()
    
    #future = action_client.send_goal('forward', 2, 0)
   # while not future.done():
    #    rclpy.spin_once(action_client, timeout_sec=0.01)
    #    action_client.get_logger().info("Wating for action to finish")
        
   # future1 = action_client.send_goal('turn_right', 0, 90)
   # while not future1.done():
    #    rclpy.spin_once(action_client, timeout_sec=0.01)
    #    action_client.get_logger().info("Wating for action1 to finish")
        #action_client.get_logger().info(f'was the goal done{action_client.isdone}')
        #if action_client.isdone==1:
    action_client.send_goal('turn_right', 0, 90)
    #rclpy.spin(action_client)

    
            #action_client.isdone = 0
    #rclpy.spin(action_client, future)
    #action_client.shutdown()
        #if action_client.isdone==2:
    action_client.send_goal('forward', 1, 0)
        	#action_client.isdone = 0
    rclpy.spin(action_client)
    #action_client.shutdown()
    #rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
