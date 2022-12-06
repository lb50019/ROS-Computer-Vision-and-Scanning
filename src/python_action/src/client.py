#!/usr/bin/env python
 
import rospy
import actionlib
from boxor_msgs.msg import idenjeAction, idenjeGoal, idenjeFeedback, idenjeResult
from sensor_msgs.msg import LaserScan
import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

rospy.init_node('python_action_client')
client = actionlib.SimpleActionClient('/cpp_action_server', idenjeAction)
client.wait_for_server()

goal = idenjeGoal()
goal.raz = 1
client.send_goal(goal)

state=client.get_state()
rate = rospy.Rate(1)

while state < DONE:
    print(state)
    print('Pokrenut client')
    state = client.get_state()
    rate.sleep()
