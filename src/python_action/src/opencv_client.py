#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

Subscribes to 
    /blob/point_blob
    
Publishes commands to 
    /cmd_vel    

"""

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

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)
    
K_LAT_DIST_TO_STEER = 2.0

class ChaseBall():
    def __init__(self):
        
        self.blob_x         = 0.0
        self.blob_y         = 0.0
        self._time_detected = 0.0
        
        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_ball)
        rospy.loginfo("Subscribers set")
        
        self.pub_twist = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        
        self._message = Twist()
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0
        
        self.client = actionlib.SimpleActionClient('/cpp_action_server', idenjeAction)
        self.client.wait_for_server()
        self.goal = idenjeGoal()
        self.goal.raz = 1
        self.client.send_goal(self.goal)
        
        self.state = self.client.get_state()
#        self.rate = rospy.Rate(60)
        
    @property
    
    def is_detected(self): return(time.time() - self._time_detected < 1.0)
        
    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self._time_detected = time.time()
        # rospy.loginfo("Ball detected: %.1f  %.1f "%(self.blob_x, self.blob_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        
        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action    = 0.0
        throttle_action = 0.0
        
        if self.is_detected:
            #--- Apply steering, proportional to how close is the object
            steer_action   = -K_LAT_DIST_TO_STEER*self.blob_x
            steer_action   = saturate(steer_action, -1.5, 1.5)
            rospy.loginfo("Steering command %.2f"%steer_action) 
            throttle_action = 0.3 
            
        return (steer_action, throttle_action)
        
    def run(self):
    
        while self.state < DONE:
            #-- Get the control action
            steer_action, throttle_action = self.get_control_action() 
            
            rospy.loginfo("Steering = %3.1f"%(steer_action))
            
		    #-- update the message
            #self._message.linear.x  = throttle_action
            self._message.linear.x = 0.3
            self._message.angular.z = steer_action
		        
		    #-- publish it
            self.pub_twist.publish(self._message)

            print(self.state)
            print('Pokrenut client')
            self.state = self.client.get_state()
            
            if self.client.get_result():
                self._message.linear.x = 0
                self.pub_twist.publish(self._message)
                            
if __name__ == "__main__":

    rospy.init_node('python_action_client')
    
    chase_ball = ChaseBall()
    chase_ball.run()            
