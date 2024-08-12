#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, Quaternion


array_goals = [
  # straight line
  [8.0, 0.0, 0.0],
  [12.0, 0.0, 0.0],
  [20.0, 0.0, 0.0],

  # turn over
  [25.0, 3.0, 0.0],
  [25.0, 7.0, 0.0],
  [19.0, 11.0, 0.0],
]


def move_robot(x, y, angle):
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  client.wait_for_server()

  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = "map"
  goal.target_pose.header.stamp = rospy.Time.now()

  goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, angle, 1.0))

  client.send_goal(goal)
  client.wait_for_result()

def feedback_callback(result):
  if result.status.status == 3:  # Status 3 means the goal was reached
    rospy.loginfo("Goal reached!")
    array_goals.pop(0)
  else:
    rospy.logwarn("Goal not reached. Status: %d", result.status.status)

if __name__ == '__main__':
  try:
    rospy.init_node('move_robot_node', anonymous=True)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, feedback_callback)
    
    while (len(array_goals) > 0):
      move_robot(array_goals[0][0], array_goals[0][1], array_goals[0][2])

  except rospy.ROSInterruptException:
    pass
