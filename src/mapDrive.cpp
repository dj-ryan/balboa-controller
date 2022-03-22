#include "ros/ros.h"
#include "balboa_core/balboaLL.h"
#include "std_msgs/Int64.h"

// Declare and initialize variables
balboa_core::balboaLL robotData;

// Store robot IR data when received
void callbackIR(const balboa_core::balboaLL &data)
{
	// Update robot IR data to use later
	robotData = data;
}

int main(int argc, char **argv)
{
	// State node has started on console
	ROS_INFO("STARTED");

	// Initialize node
	ros::init(argc, argv, "mapDrive");
	ros::NodeHandle node;

	// Declare subscribers and publishers
	ros::Subscriber sub = node.subscribe("balboaLL", 1000, callbackIR);

	ros::Publisher pubDrive = node.advertise<std_msgs::Int64>("mapDrive", 1000);
	//ros::Publisher pubTurn = node.advertise<std_msgs::Int64>("mapTurn", 1000);

	// Set standard distance targets and turn targets
	std_msgs::Int64 targetDrive;
	std_msgs::Int64 targetAngle;
	targetDrive.data = 62;
	targetAngle.data = 48000;

	// Initialize map grid variables
	int average;
	int snake[240];

	// Set loop rate
	ros::Rate loop_rate(10);

	// Main node loop
	while (ros::ok())
	{

		// Publish target values
		// pub.publish();

		// for (int i = 1; i < 48; i++)
		// {

		// 	// Store reflectance data point
		// 	average = (((robotData.IRsensor1) + (robotData.IRsensor2) + (robotData.IRsensor3) + (robotData.IRsensor4) + (robotData.IRsensor5)) / 5);
		// 	snake[i - 1] = average;

		// 	// Publish target distance values
		// 	pubDrive.publish(targetDrive);

		// 	// TEST print the reflectance data point
		// 	ROS_INFO("Avg Reflectance: %d", snake[i - 1]);
		// }
		targetDrive.data += 50;
		ROS_INFO("new target: %d", (int)targetDrive.data);

		pubDrive.publish(targetDrive);





		// Spin loop and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}