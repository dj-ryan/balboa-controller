#!/usr/bin/env python
from __future__ import print_function
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
import rospy

pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)
motor = balboaMotorSpeeds()

targetAngle = 0
targetDistance = 0

#pValAng = 0.0001
#dValAng = 0.00000001
#pValDis = 0.015
#dValDis = 0.0001

pValAng = rospy.get_param('/pidLab1/pValAng')
dValAng = rospy.get_param('/pidLab1/dValAng')
pValDis = rospy.get_param('/pidLab1/pValDis')
dValDis = rospy.get_param('/pidLab1/dValDis')

initFlag = 0
initIMU = balboaLL()

oldAngError = 0
oldDisError = 0

def callbackIMU(data):
    global initFlag
    global targetAngle
    global targetDistance
    global oldAngError
    global oldDisError
    
    if(initFlag == 0):
        initIMU = data
        initFlag = 1
        targetAngle = initIMU.angleX
        targetDistance = initIMU.distanceRight

    angError = (targetAngle - data.angleX)
    disError = (targetDistance - data.distanceRight)
    
    angPidVal = (pValAng * angError) + (dValAng * (angError - oldAngError)) 
    disPidVal = (pValDis * disError) + (dValDis * (disError - oldDisError))
    
    motor.right = angPidVal + disPidVal
    motor.left = -angPidVal + disPidVal
    
    rospy.loginfo("Angle error: %d | old Angle error: %d", angError, oldAngError)
    rospy.loginfo("Distance error: %d | old Distance error: %d", disError, oldDisError)
    
    oldAngError = angError
    oldDisError = disError
    
    rospy.loginfo("motor left: %d | motor right: %d", data.speedLeft, data.speedRight)
    
    #rospy.loginfo("error: %d | old error: %d", error, oldError)
    #rospy.loginfo("IMU updated")
    #rospy.loginfo("current val: %d", data.angleX)
    #rospy.loginfo("targetAngle: %d | currentAngle: %d | error: %d",targetAngle,data.angleX, error)

def callbackKey(data):
    global targetAngle
    global targetDistance
    # rospy.loginfo("keys")
    #rospy.loginfo("new target: %d", targetAngle)

    if(data.angular.z > 0):
        targetAngle = targetAngle + 48000
    elif(data.angular.z < 0):
        targetAngle = targetAngle - 48000
    elif(data.linear.x > 0):
        targetDistance = targetDistance - 750
    elif(data.linear.x < 0):
        targetDistance = targetDistance + 750

def pidSubscriptions():
    rospy.init_node('pidLab1')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/turtle1/cmd_vel', Twist, callbackKey)
    rospy.Subscriber('/balboaLL', balboaLL, callbackIMU)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("while")
        pubMotor.publish(motor)
        motor.left = 0
        motor.right = 0
        rate.sleep()

if __name__ == "__main__":
    pidSubscriptions()

