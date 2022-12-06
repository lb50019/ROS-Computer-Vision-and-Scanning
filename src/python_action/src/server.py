#! /usr/bin/env python

import rospy
import actionlib
from boxor_msgs.msg import idenjeFeedback, idenjeResult, idenjeAction
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Stop(object):
    # create messages that are used to publish feedback/result
    _feedback = idenjeFeedback()
    _result = idenjeResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, idenjeAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        
    def laser_callback(self, msg):
        self._feedback.tren_raz = msg.ranges[0]
        self._as.publish_feedback(self._feedback)   
        
    def execute_cb(self, goal):
    
    	success = True
        
        _pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        move = Twist()
        
        _sub = rospy.Subscriber('/robot1/scan', LaserScan, self.laser_callback)
        
        if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                
        while(self._feedback.tren_raz > goal.raz):
            move.linear.x = 0.1
            _pub.publish(move)
            
        move.linear.x = 0
        _pub.publish(move)

        if success:
            self._result.kon_raz = self._feedback.tren_raz
            rospy.loginfo('%s: uspio' % self._action_name)
            self._as.set_succeeded(self._result)
            
        
       
if __name__ == '__main__':
    rospy.init_node('naprijed_node')
    server = Stop('python_action_server')
    rospy.spin()
