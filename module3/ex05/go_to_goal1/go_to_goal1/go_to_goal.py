import rclpy
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from rclpy.node import Node


class TurtleBot(Node):

     def __init__(self):
         # Creates a node with name 'turtlebot_controller' and make sure it is a
         # unique node (using anonymous=True).
         super().__init__('turtlebot_controller')
 
         # Publisher which will publish to the topic '/turtle1/cmd_vel'.
         self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
 
         # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
         # when a message of type Pose is received.
         self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
 
         self.pose = Pose()
         self.timer = self.create_timer(0.1, self.move2goal)

     def update_pose(self, data):
         """Callback function which is called when a new message of type Pose is
         received by the subscriber."""
         self.pose = data
 
     def steering_angle(self, goal_pose):
         return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

     def move2goal(self):
         """Moves the turtle to the goal."""
         const = 5
         vel_msg = Twist()
         goal_pose = Pose()
 
         # Get the input from the user.
         goal_pose.x = float(sys.argv[1])
         goal_pose.y = float(sys.argv[2])
 
         tolerance =  0.01 
 
         distance_to_goal = sqrt(pow((goal_pose.x - self.pose.x), 2) + 
         			 pow((goal_pose.y - self.pose.y), 2))
         angle_error = self.steering_angle(goal_pose) - self.pose.theta
 
         if abs(angle_error) > tolerance:
             vel_msg.angular.z = const*angle_error  
         elif distance_to_goal >= tolerance:
             # Linear velocity in the x-axis.
             vel_msg.linear.x = const*distance_to_goal 
         else:
             vel_msg.linear.x = 0.0
             vel_msg.angular.z = 0.0
             self.get_logger().info("Goal Reached!! ")
             quit()
 
         # Publishing our vel_msg
         self.velocity_publisher.publish(vel_msg)
 
         
def main(args=None):
    rclpy.init(args=args)
    x = TurtleBot()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()

