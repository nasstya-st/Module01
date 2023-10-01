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
         #rospy.init_node('turtlebot_controller', anonymous=True)
 
         # Publisher which will publish to the topic '/turtle1/cmd_vel'.
         self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
 
         # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
         # when a message of type Pose is received.
         self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose',
                                                 self.update_pose, 10)
 
         self.pose = Pose()
         ####
         self.timer = self.create_timer(0.1, self.move2goal)
         #self.rate = rclpy.timer.Rate(self.timer, context=None)
         #self.rate = node.create_rate(10)
 
     def update_pose(self, data):
         """Callback function which is called when a new message of type Pose is
         received by the subscriber."""
         self.pose = data
         #self.pose.x = round(self.pose.x, 4)
         #self.pose.y = round(self.pose.y, 4)
 
     def euclidean_distance(self, goal_pose):
         """Euclidean distance between current pose and the goal."""
         return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                     pow((goal_pose.y - self.pose.y), 2))
 
     def linear_vel(self, goal_pose, constant=1.5):
         """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
         return constant * self.euclidean_distance(goal_pose)
 
     def steering_angle(self, goal_pose):
         """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
         return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
 
     def angular_vel(self, goal_pose, constant=6):
         """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
         return constant * (self.steering_angle(goal_pose) - self.pose.theta)
 
     def move2goal(self):
         """Moves the turtle to the goal."""
         vel_msg = Twist()
         goal_pose = Pose()
 
         # Get the input from the user.
         goal_pose.x = float(sys.argv[1])
         goal_pose.y = float(sys.argv[2])
 
         # Please, insert a number slightly greater than 0 (e.g. 0.01).
         distance_tolerance =  0.01 # float(input("Set your tolerance: "))
 
         
         distance_to_goal = self.euclidean_distance(goal_pose)
         angle_error = self.steering_angle(goal_pose) - self.pose.theta
 
         if abs(angle_error) > distance_tolerance:
             vel_msg.angular.z = self.angular_vel(goal_pose)
         else:
             if distance_to_goal >= distance_tolerance:
 
             # Porportional controller.
             # https://en.wikipedia.org/wiki/Proportional_control
 
             # Linear velocity in the x-axis.
             	 vel_msg.linear.x = self.linear_vel(goal_pose)
             #vel_msg.linear.y = 0.0
             #vel_msg.linear.z = 0.0
 
             # Angular velocity in the z-axis.
             #vel_msg.angular.x = 0.0
             #vel_msg.angular.y = 0.0
             else:
                 vel_msg.linear.x = 0.0
                 vel_msg.angular.z = 0.0
                 self.get_logger().info("Goal Reached ")
                 quit()
		         #self.velocity_publisher.publish(vel_msg)
 
             # Publishing our vel_msg
         self.velocity_publisher.publish(vel_msg)
 
             # Publish at the desired rate.
             #self.rate.sleep()
 
         # Stopping our robot after the movement is over.
 
         # If we press control + C, the node will stop.
         #rospy.spin()
         
def main(args=None):
    rclpy.init(args=args)
    x = TurtleBot()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()
    #x.move2goal()
 
if __name__ == '__main__':
     #try:
     #    x = TurtleBot()
     #    x.move2goal()
     #except rospy.ROSInterruptException:
     #    pass

    main()

