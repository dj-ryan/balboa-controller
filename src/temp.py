#!/usr/bin/env python
from __future__ import print_function
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
import rospy

pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)
motor = balboaMotorSpeeds()
targetAngle = 0
pVal = 0.001

def callbackIMU(data):
	global targetAngle
	rospy.loginfo("IMU")
	motor.left = pVal*(targetAngle-data.angleZ)
	rospy.loginfo(pVal*(targetAngle-data.angleZ))
	rospy.loginfo(data.angleZ)

def callbackKey(data):
    global targetAngle
    rospy.loginfo("keys")
    if(data.angular.z > 0):
        targetAngle = targetAngle + 85,500

def pidSubscriptions():
    rospy.init_node('pidLab1')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callbackKey)
    rospy.Subscriber('/balboaLL', balboaLL, callbackIMU)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rospy.loginfo("while")
        motor.left = 0
        motor.right = 0
        pubMotor.publish(motor)
        rate.sleep()

if __name__ == "__main__":
	pidSubscriptions()
