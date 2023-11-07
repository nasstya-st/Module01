import rclpy, sys
from std_msgs.msg import String
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/fox/cmd_vel', 10)
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        sec, _ = self.get_clock().now().seconds_nanoseconds()
        twist = Twist()
        twist.linear.x = math.cos(sec) # draws star(4-pointed)
        twist.linear.y = math.cos(sec) 
        twist.angular.z = 0.5     
        self.publisher.publish(twist)
        
        
def main(args=None):
    rclpy.init(args=args)

    circling = CirclePublisher()

    rclpy.spin(circling)

    #circling.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
