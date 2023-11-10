import rclpy, sys
from std_msgs.msg import String
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField, Image
from geometry_msgs.msg import Twist
import math
from point_cloud2 import read_points

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

is_obstacle = 0


class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/fox/cmd_vel', 10)
        self.subscription = self.create_subscription(Image, "/depth/image",
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
        cv_bridge = CvBridge()
        depth_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32)
        h, w = depth_array.shape
        print(depth_array.shape, depth_array)
        global is_obstacle
        is_obstacle = 1 if depth_array[120][w//2] < 4 else 0
        
    '''
        #data = read_points(msg, field_names=("x", "y", "z"))
        #print(data.shape)
        data = msg.data
        n = len(data)
        print(data[:])
        """
        global is_obstacle
        is_obstacle = 1 if data[n//2] < 4 else 0"""
        '''
        
def main(args=None):
    rclpy.init(args=args)

    circling = CirclePublisher()

    rclpy.spin(circling)

    #circling.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
