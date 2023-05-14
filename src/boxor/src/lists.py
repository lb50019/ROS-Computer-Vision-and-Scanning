#! /usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
import math
import tf
from misonja import udalj, kutovi

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

rospy.init_node('liste')
listener = tf.TransformListener()
listener.waitForTransform('/base_link','/map', rospy.Time(), rospy.Duration(4.0))
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
move = Twist()
(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
rot = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])
rate=rospy.Rate(60)
pol = 0
usp = 0

#vrime = [2.14, 1.7, 1]
#kutovi = [0, 1.57, -1.57]

for i in range(len(kutovi)):
	kutovi[i]-=1.57

for i in range(len(udalj)):

	move.angular.z = 0
	move.linear.x = 0
	pub.publish(move)
	
	rospy.sleep(3)

	while(abs(rot[2] - kutovi[i]) > 0.4):
	    if(rot[2] > 0 and kutovi[i] > 0):
		    pol = 1
		    if(rot[2] < kutovi[i]):
			    usp = 1
		    elif(rot[2] > kutovi[i]):
			    usp = -1
	    elif(rot[2] < 0 and kutovi[i] < 0):
		    pol = -1
		    if(abs(rot[2]) < abs(kutovi[i])):
			    usp = 1
		    elif(abs(rot[2]) > abs(kutovi[i])):
			    usp = -1
	    elif(rot[2] > 0 and kutovi[i] < 0):
		    pol = 1
		    if(rot[2] - kutovi[i] < 3.14):
			    usp = -1
		    elif(rot[2] - kutovi[i] > 3.14):
			    usp = 1
	    elif(rot[2] < 0 and kutovi[i] > 0):
		    pol = -1
		    if(rot[2] - kutovi[i] < 3.14):
			    usp = -1
		    elif(rot[2] - kutovi[i] > 3.14):
			    usp = 1
	    move.angular.z = pol * usp * 0.1
	    pub.publish(move)
	    (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	    rot = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])
	    rate.sleep()
	
	rospy.sleep(3)
	
	move.angular.z = 0
	move.linear.x = 0.7
	pub.publish(move)
	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	rot = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])
	rospy.sleep(udalj[i]/40)
