#include "ros/ros.h"
#include "std_msgs/String.h"
#include "balboa_core/balboaLL.h"

#include "balboa_controller/irArray.h"

//#include "std_msgs/Header.h"


balboa_core::balboaLL robot;
balboa_controller::irArray ir;

void callbackIR(const balboa_core::balboaLL::ConstPtr& data)
{
	robot = *data;
	if(robot.IRsensor1 < 900){
		ir.sensor1 = 0;
	} else {
		ir.sensor1 = 1;
	}
	if(robot.IRsensor2 < 900){
		ir.sensor2 = 0;
	} else {
		ir.sensor2 = 1;
	}
	if(robot.IRsensor4 < 900){
		ir.sensor4 = 0;
	} else {
		ir.sensor4 = 1;
	}
	if(robot.IRsensor5 < 900){
		ir.sensor5 = 0;
	} else {
		ir.sensor5 = 1;
	}
	
	//ir.header.stamp = ros::Time::now();
}

int main(int argc, char **argv)
{
	ROS_INFO("STARTED");
	ros::init(argc, argv, "mapLab2");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("balboaLL", 1000, callbackIR);
	ros::Publisher pub = n.advertise<balboa_controller::irArray>("irData", 10);
	
	ros::Rate loop_rate(10);
	
	while(ros::ok()){
		pub.publish(ir);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}