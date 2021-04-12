#!/usr/bin/env python3

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to person.
distance = 0.4

class StopAtPerson(object):
    """ This node walks the robot to person and stops """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("walk_to_person")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def process_scan(self, data):
        # Determine closeness to person by ooking at scan data from in front of
        #   the robot, set linear velocity based on that information, and
        #   publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.

        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.


        if data.ranges[0] > 3:
            # If person is out of range, turn
            self.twist.angular.z = .3

        elif data.ranges[0] >= distance:
            # Go forward if not close enough to person.
            # make sure robot goes straight to person
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0
        else:
            # Close enough to person, stop.
            self.twist.linear.x = 0
            self.twist.angular.z = 0

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

        print(data.ranges[0])

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = StopAtPerson()
    node.run()