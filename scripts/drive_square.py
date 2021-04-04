#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveInSquare(object):
    """ This node drives the robot in a square """
    def __init__(self):
        # Start rospy node.
        rospy.init_node("drive_in_square")

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg. Set lin velocity to .2
        lin = Vector3(.2,0,0)
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def run(self):
        # Allow time to initialize
        rospy.sleep(1)

        # Create loop that rotates robot by 90 degrees and moves it
        while not rospy.is_shutdown():
            # Drive in a straight line for 5s
            self.twist_pub.publish(self.twist)
            rospy.sleep(5)

            # Set angular velocity and turn for 1.6s
            self.twist.angular.z = 1
            self.twist_pub.publish(self.twist)
            rospy.sleep(1.6)

            # Set angular velocity back to 0
            self.twist.angular.z = 0
        
if __name__ == '__main__':
    # Declare a node and run it.
    node = DriveInSquare()
    node.run()