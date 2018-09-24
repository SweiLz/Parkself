#!/usr/bin/env python

import rospy
from math import sqrt, atan2, cos, sin
from geometry_msgs.msg import Twist, PoseStamped, Pose2D, PoseWithCovarianceStamped
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
import angles

goal = Pose2D()
cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)


def poseCB(data):
    global goal
    robot = Pose2D()
    robot.x = round(data.pose.pose.position.x, 4)
    robot.y = round(data.pose.pose.position.y, 4)
    quanternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    robot.theta = round(euler_from_quaternion(quanternion)[2], 4)
    cmd_vel_msg = Twist()

    dx = goal.x - robot.x
    dy = goal.y - robot.y
    dist = sqrt(dx**2 + dy**2)

    if dist > 0.02:
        angle = atan2(dy, dx)
        angle = angles.shortest_angular_distance(robot.theta, angle)
        # rospy.loginfo("dx: {}, {}".format(dx, dy))
        # rospy.loginfo("v: {}, {}".format(cos(angle), sin(angle)))
        rospy.loginfo("dist: {}".format(dist))
        cmd_vel_msg.linear.x = 0.09 * cos(angle)
        cmd_vel_msg.linear.y = 0.09 * sin(angle)
        cmd_vel_msg.angular.z = 1 * -robot.theta
    cmd_vel.publish(cmd_vel_msg)


def goalCB(data):
    global goal
    goal.x = round(data.pose.position.x, 4)
    goal.y = round(data.pose.position.y, 4)
    # quanternion = (
    #     data.pose.orientation.x,
    #     data.pose.orientation.y,
    #     data.pose.orientation.z,
    #     data.pose.orientation.w)
    # goal.theta = round(euler_from_quaternion(quanternion)[2], 4)
    goal.theta = 0.0


def main():
    rospy.init_node("parkself_controller")
    rospy.loginfo("Parkself Controller")
    rospy.Subscriber("poseupdate", PoseWithCovarianceStamped, poseCB)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, goalCB)

    rospy.spin()

if __name__ == '__main__':
    main()
