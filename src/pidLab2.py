#!/usr/bin/env python
from __future__ import print_function
from re import S
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
import rospy

pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)
motor = balboaMotorSpeeds()





pValDis = 0.07
dValDis = 0.01

# pValAng = rospy.get_param('/pidLab1/pValAng')
# dValAng = rospy.get_param('/pidLab1/dValAng')
# pValDis = rospy.get_param('/pidLab1/pValDis')
# dValDis = rospy.get_param('/pidLab1/dValDis')






oldDisError = 0

def callbackIR(data):
    global oldDisError

    targetDistance = 80

 
    disError = (targetDistance - data.linear.x)

    rospy.loginfo("targetDis: %d", targetDistance)

    rospy.loginfo("disError: %d", disError)
    
    disPidVal = (pValDis * disError) + (dValDis * (disError - oldDisError)) 

    rospy.loginfo("disPidVal: %d" , disPidVal)
    
    motor.right = disPidVal
    motor.left = disPidVal
    
    oldDisError = disError
    

def pidSubscriptions():
    rospy.init_node('pidLab1')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/ir_reactive_control/cmd_vel', Twist, callbackIR)
    #rospy.Subscriber('/balboaLL', balboaLL, callbackIR)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #rospy.loginfo("while")
        pubMotor.publish(motor)
        motor.left = 0
        motor.right = 0
        rate.sleep()

if __name__ == "__main__":
    pidSubscriptions()

