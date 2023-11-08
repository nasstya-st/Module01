import rclpy, sys
from std_msgs.msg import String
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

is_obstacle = 0


class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/fox/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, "/fox/scan",
         	    self.listener_callback, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        if is_obstacle:
            twist.linear.x = 0.
        else:
            twist.linear.x = 1.
        self.publisher.publish(twist)
        

    def listener_callback(self, msg):
        data = msg.ranges
        n = len(data)
        print(data[n//2])
        global is_obstacle
        is_obstacle = 1 if data[n//2] < 4 else 0
        
        
def main(args=None):
    rclpy.init(args=args)

    circling = CirclePublisher()

    rclpy.spin(circling)

    #circling.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
