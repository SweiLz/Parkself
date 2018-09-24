#!/usr/bin/env python

import rospy
from math import sqrt
from geometry_msgs.msg import Twist, PoseStamped, Pose2D, PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, String
import Queue

goal_point = rospy.Publisher(
    "move_base_simple/goal", PoseStamped, queue_size=1)

servo = rospy.Publisher("servo", Empty, queue_size=1)

robot_state = "In"
p0 = [(0, 0), (0.53, -0.03), (1.09, -0.07)]
p1 = [(0.00, -0.46), (0.53, -0.50), (1.00, -0.55)]
p2 = [(0, -0.90), (0.53, -0.90), (1.00, -0.90)]
p3 = [(0, -1.30), (0.53, -1.30), (1.00, -1.30)]
p4 = [(0, -1.72), (0.53, -1.70), (1.00, -1.72)]
p5 = [(0, -2.10), (0.53, -2.10), (1.00, -2.10)]
graphN = {
    "In": p0[0],
    "Out": p0[2],
    "B1": p1[0],
    "B2": p2[0],
    "B3": p3[0],
    "B4": p4[0],
    "B5": p5[0],
    "A1": p1[2],
    "A2": p2[2],
    "A3": p3[2],
    "A4": p4[2],
    "A5": p5[2]
}

graph = {
    p0[0]: [p0[1]],
    p0[1]: [p0[0], p0[2], p1[1]],
    p0[2]: [p0[1]],

    p1[0]: [p1[1]],
    p1[1]: [p0[1], p1[0], p1[2], p2[1]],
    p1[2]: [p1[1]],

    p2[0]: [p2[1]],
    p2[1]: [p1[1], p2[0], p2[2], p3[1]],
    p2[2]: [p2[1]],

    p3[0]: [p3[1]],
    p3[1]: [p2[1], p3[0], p3[2], p4[1]],
    p3[2]: [p3[1]],

    p4[0]: [p4[1]],
    p4[1]: [p3[1], p4[0], p4[2], p5[1]],
    p4[2]: [p4[1]],

    p5[0]: [p5[1]],
    p5[1]: [p4[1], p5[0], p5[2]],
    p5[2]: [p5[1]]
}


def bfs(start, goal):
    global graph
    frontier = Queue.Queue()
    frontier.put(start)
    came_from = {start: None}
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            break
        for nieg in graph[current]:
            if nieg not in came_from:
                frontier.put(nieg)
                came_from[nieg] = current
    path = [goal]
    parent = came_from.get(goal)
    while parent != None:
        path.insert(0, parent)
        parent = came_from.get(parent)
    return path


def userCB(msg):
    global graphN, robot_state
    msgStart, msgGoal = msg.data.split(',')
    rospy.loginfo("from {} -> {}".format(msgStart, msgGoal))

    path = bfs(graphN[robot_state], graphN[msgStart])
    rospy.loginfo("path: {}".format(path))
    goal = PoseStamped()
    goal.header.frame_id = "map"
    for i, pat in enumerate(path[1:]):
        goal.pose.position.x = pat[0]
        goal.pose.position.y = pat[1]
        goal_point.publish(goal)
        dx = pat[0] - path[i][0]
        dy = pat[1] - path[i][1]
        duration = sqrt(dx**2 + dy**2) / 0.08
        rospy.sleep(duration)
    servo.publish(Empty())
    rospy.sleep(2.0)

    path = bfs(graphN[msgStart], graphN[msgGoal])
    rospy.loginfo("path: {}".format(path))
    for i, pat in enumerate(path[1:]):
        goal.pose.position.x = pat[0]
        goal.pose.position.y = pat[1]
        goal_point.publish(goal)
        dx = pat[0] - path[i][0]
        dy = pat[1] - path[i][1]
        duration = sqrt(dx**2 + dy**2) / 0.08
        rospy.sleep(duration)
    servo.publish(Empty())
    rospy.sleep(2.0)
    robot_state = msgGoal


def main():
    rospy.init_node("parkself_runner")
    rospy.loginfo("Parkself Runner")
    rospy.Subscriber("user", String, userCB)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal_point.publish(goal)

    rospy.spin()

if __name__ == '__main__':
    main()
