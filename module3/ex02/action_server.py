import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_turtle_commands.action import MessageTurtleCommands


class CommandsActionServer(Node):

    def __init__(self):
        super().__init__('action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'execute_turtle_commands',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = MessageTurtleCommands.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = CommandsActionServer()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
