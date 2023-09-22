import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Subscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, "cmd_text",
         	    self.listener_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def listener_callback(self, msg):
        cmd = msg.data
        twist = Twist()

        if cmd == 'turn_right': twist.angular.z = -1.5  
        elif cmd == 'turn_left': twist.angular.z = 1.5  
        elif cmd == 'move_forward': twist.linear.x = 1.0  
        elif cmd == 'move_backward': twist.linear.x = -1.0  
        
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    text2vel = Subscriber()

    rclpy.spin(text2vel)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    text2vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

