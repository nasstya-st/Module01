import rclpy, sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Empty, "robot_start",
         	    self.listener_callback, 10)
        self.subscription = self.create_subscription(Image, "/color/image",
         	    self.image_callback, 10) 	    
        self.timer_period = 0.5 
        self.error = [0,0]
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.prevpt1 = [110,60]
        self.prevpt2 = [620,60]
        self.curr_time = 0

    def timer_callback(self):
        self.curr_time += self.timer_period
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = float((self.error[-1] * 4) + \
                            3 * self.error[-1]*np.sum(np.linspace(0, self.curr_time, len(self.error))) + \
                            2 * (self.error[-1] - self.error[-2]) / self.timer_period )

        self.publisher.publish(cmd_vel)
        '''
        twist = Twist()
        twist.linear.x = 1.
        twist.angular.z = 0.5         
        self.publisher.publish(twist)
        '''
  
    def listener_callback(self, msg):
        twist = Twist()
        twist.linear.x = 0.5      
        self.publisher.publish(twist)
        
    def image_callback(self, msg):
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        gray = gray + 100 - np.mean(gray)
        _, gray = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)

        dst = gray[int(gray.shape[0]/3*2):, :]
        dst = np.array(dst, dtype=np.uint8)
        
        cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

        if cnt > 1:
            mindistance1 = []
            mindistance2 = []
            for i in range(1, cnt):
                p = centroids[i]
                ptdistance = [abs(p[0] - self.prevpt1[0]), abs(p[0] - self.prevpt2[0])]
                mindistance1.append(ptdistance[0])
                mindistance2.append(ptdistance[1])

            threshdistance = [min(mindistance1), min(mindistance2)]

            minlb = [mindistance1.index(min(mindistance1)), mindistance2.index(min(mindistance2))]

            cpt = [centroids[minlb[0] + 1], centroids[minlb[1] + 1]]

            if threshdistance[0] > 100:
                cpt[0] = self.prevpt1
            if threshdistance[1] > 100:
                cpt[1] = self.prevpt2

            mindistance1.clear()
            mindistance2.clear()
        else:
            cpt = [self.prevpt1, self.prevpt2]

        self.prevpt1 = cpt[0]
        self.prevpt2 = cpt[1]

        fpt = [(cpt[0][0] + cpt[1][0]) / 2, (cpt[0][1] + cpt[1][1]) / 2 + gray.shape[0] / 3 * 2]
        dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

        cv2.circle(frame, tuple(map(int, fpt)), 2, (0, 0, 255), 2)
        cv2.circle(dst, tuple(map(int, cpt[0])), 2, (0, 0, 255), 2)
        cv2.circle(dst, tuple(map(int, cpt[1])), 2, (255, 0, 0), 2)

        self.error.append(dst.shape[1] / 2 - fpt[0])
        
        
def main(args=None):
    rclpy.init(args=args)

    circling = CirclePublisher()

    rclpy.spin(circling)

    #circling.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
