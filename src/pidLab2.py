#!/usr/bin/env python
from __future__ import print_function
from re import S
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
import rospy

pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)
motor = balboaMotorSpeeds()

#PID constants
pValDis = 0.07
dValDis = 0.01

oldDisError = 0

def callbackIR(data):
    global oldDisError

	#set the constant buffer distance - robot strives to maintain this distance always
    targetDistance = 80

	#calculate error between target and actual IR values
    disError = (targetDistance - data.linear.x)
    
	#calculate PID value and set to motors
    disPidVal = (pValDis * disError) + (dValDis * (disError - oldDisError)) 
    rospy.loginfo("disPidVal: %d" , disPidVal)
    
    motor.right = disPidVal
    motor.left = disPidVal
    
	#set current error to old error to use in the D term of the next PID loop
    oldDisError = disError
    

def pidSubscriptions():
	#initialize node/subscriber
    rospy.init_node('pidLab1')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/ir_reactive_control/cmd_vel', Twist, callbackIR)
    rate = rospy.Rate(10)
	
    while not rospy.is_shutdown():
		#publish to Balboa motors
        pubMotor.publish(motor)
        motor.left = 0
        motor.right = 0
        rate.sleep()

if __name__ == "__main__":
    pidSubscriptions()

