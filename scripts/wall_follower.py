#!/usr/bin/env python3

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to a wall with our front sensor.
distance = 1.3
# How close we will stay to the walk while moving along it
buffer = .4

# keep track of whether a variable has been seen or not
wall = False

class FollowWall(object):
    """ This node walks the robot to a wall and turns """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("follow_wall")

        wall = False

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
        # If we are far enough away from a wall, go straight, 
        # If we detect we are approaching a wall, start turning fast
        # If we are very close to a wall turn slowly
        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.
        # The entry at 320 represents close to the front right corner of the robot: the one close to the wall

        if data.ranges[0] < .3:
            #If robot is against a wall, backup so it can turn
            self.twist.linear.x = -0.3
            print("1")

        elif (data.ranges[0] < .7 or data.ranges[320] < buffer):
            # If the front or side of the robot is very close to a wall, turn slightly
            self.twist.angular.z = .3
            self.twist.linear.x = .1
            print("2")

        elif data.ranges[0] < distance:
            # If we are starting to get close to a wall, turn fast
            self.twist.angular.z = .8
            self.twist.linear.x = .3
            print("3")

        elif data.ranges[0] >= distance:
            # Go forward if not close enough to a wall.
            if data.ranges[320] > buffer and data.ranges[320] < .8:
                # If we are turning away from a wall, but not at a corner yet, 
                # turn back slightly to maintain an even distance from wall
                self.twist.angular.z = -.1
                self.twist.linear.x = .1
            else:
                # If we are still close enough to wall, continue going straight
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
            print("4")

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

        # Print statements to check that the robot stays a consistent distance from the wall
        # print(data.ranges[0])
        # print(data.ranges[90])
        # print(data.ranges[180])
        # print(data.ranges[270])

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = FollowWall()
    node.run()