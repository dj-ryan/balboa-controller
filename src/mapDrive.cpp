#include "ros/ros.h"
#include "balboa_core/balboaLL.h"
#include "std_msgs/Int32.h"
#include <cmath>
#include <map>
#include <string>

// Declare and initialize variables
balboa_core::balboaLL robotData;

// Create variable for storing reflectance sensor data for mapping
struct IrValues
{
	int sensor1;
	int sensor2;
	int sensor4;
	int sensor5;
};

std::map<int, struct IrValues> rowCapture;

// Store ASCII characters corresponding to mapped sensor values
// void printMapRow(std::map<int, struct IrValues> row, int i)
// {
// 	std::string row1Output = "row " + std::to_string(i) + " => ";
// 	std::string row2Output = "row " + std::to_string(i + 1) + " => ";
// 	std::string row4Output = "row " + std::to_string(i + 2) + " => ";
// 	std::string row5Output = "row " + std::to_string(i + 3) + " => ";

// 	for (const auto &dataPoint : row)
// 	{
// 		if (dataPoint.second.sensor1 > 1600)
// 		{
// 			row1Output.append("0");
// 		}
// 		else if (dataPoint.second.sensor1 <= 1600 && dataPoint.second.sensor1 >= 500)
// 		{
// 			row1Output.append("o");
// 		}
// 		else
// 		{
// 			row1Output.append("_");
// 		}
// 		if (dataPoint.second.sensor2 > 1600)
// 		{
// 			row2Output.append("0");
// 		}
// 		else if (dataPoint.second.sensor2 <= 1600 && dataPoint.second.sensor2 >= 500)
// 		{
// 			row2Output.append("o");
// 		}
// 		else
// 		{
// 			row2Output.append("_");
// 		}
// 		if (dataPoint.second.sensor4 > 1600)
// 		{
// 			row4Output.append("0");
// 		}
// 		else if (dataPoint.second <= 1600 && dataPoint.second.sensor4 >= 500)
// 		{
// 			row4Output.append("o");
// 		}
// 		else
// 		{
// 			row4Output.append("_");
// 		}
// 		if (dataPoint.second.sensor5 > 1600)
// 		{
// 			row5Output.append("0");
// 		}
// 		else if (dataPoint.second.sensor5 <= 1600 && dataPoint.second.sensor5 >= 500)
// 		{
// 			row5Output.append("o");
// 		}
// 		else
// 		{
// 			row5Output.append("_");
// 		}
// 	}

	// ROS_INFO(row1Output.c_str())
	// ROS_INFO(row2Output.c_str())
	// ROS_INFO(row4Output.c_str())
	// ROS_INFO(row5Output.c_str())

	// return rowOutput;
//}

// Print ASCII map from stored characters
// void printMap(std::vector<std::map<int, int>> rows)
// {
// 	int i = 0;
// 	for (std::map<int, int> r : rows)
// 	{
// 		ROS_INFO(printMapRow(r, i).c_str());
// 		i++;
// 	}
// }

// Store robot IR data when received
void callbackIMU(const balboa_core::balboaLL &data)
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

	// Example row print
	// ROS_INFO(printMapRow(exampleRow, 0).c_str());

	// Declare subscribers and publishers
	ros::Subscriber sub = node.subscribe("balboaLL", 1000, callbackIMU);

	ros::Publisher pubDrive = node.advertise<std_msgs::Int32>("mapDrive", 1000);
	ros::Publisher pubTurn = node.advertise<std_msgs::Int32>("mapTurn", 1000);

	// Set standard distance targets and turn targets
	std_msgs::Int32 targetDrive;
	std_msgs::Int32 targetAngle;

	// Initialize map grid variables
	int totalRows = 4;
	int rowLength = 6000;
	int doneOnce = 0;
	int rightTurn = 23000;
	int leftTurn = -23000;
	int goToNextRow = 650;
	int finishLine = 0;

	// Set loop rate
	ros::Rate loop_rate(0.5);

	// Main node loop
	while (ros::ok())
	{
		// Drive robot through totalRows, and then stop
		if (doneOnce == 0)
		{
			doneOnce = 1;

			// Drive robot through totalRows
			for (int i = 0; i < totalRows; i++)
			{
				ros::spinOnce();

				// struct IrValues irValues;

				// Drive the first row, 16 steps of approximately 1.5in at a time 
				for (int j = 0; j < 16; j++)
				{

					// Capture reflectance sensor data 
					// irValues.sensor1 = robotData.IRsensor1;
					// irValues.sensor2 = robotData.IRsensor2;
					// irValues.sensor4 = robotData.IRsensor4;
					// irValues.sensor5 = robotData.IRsensor5;

					// rowCapture.insert(j, irValues);

					// Publish next step 
					targetDrive.data = 200;
					pubDrive.publish(targetDrive);

					ROS_INFO("Sent target: %d", targetDrive);

					// Sleep while waiting for robot to hit target 
					ros::spinOnce();
					loop_rate.sleep();
				}

				ROS_INFO(" == FINISHED ROW ==");

				// printMapRow(rowCapture, i);

				// Make first 90 deg turn out of row, alternate left and right turns
				if (i % 2 == 0)
				{
					// Publish 4 steps of approx. 22 degrees
					for (int j = 0; j < 4; j++) 
					{
					targetAngle.data = leftTurn;
					pubTurn.publish(targetAngle);
					loop_rate.sleep();
					}
				}
				else
				{
					// Publish 4 steps of approx. 22 degrees
					for (int j = 0; j < 4; j++)
					{
					targetAngle.data = rightTurn;
					pubTurn.publish(targetAngle);
					loop_rate.sleep();
					}
				}

				ROS_INFO(" == Completed first turn ");

				// Drive to next row, approx. robot width 
				targetDrive.data = goToNextRow;
				pubDrive.publish(targetDrive);
				ros::spinOnce();
				loop_rate.sleep();

				ROS_INFO(" == Arrived at next row == ");

				// Make second 90 deg turn into row, alternate left and right turns
				if (i % 2 == 0)
				{
					// Publish 4 steps of approx. 22 degrees
					for (int j = 0; j < 4; j++) 
					{
					targetAngle.data = leftTurn;
					pubTurn.publish(targetAngle);
					loop_rate.sleep();
					}
				}
				else
				{
					// Publish 4 steps of approx. 22 degrees
					for (int j = 0; j < 4; j++)
					{
					targetAngle.data = rightTurn;
					pubTurn.publish(targetAngle);
					loop_rate.sleep();
					}
				}

				ROS_INFO(" == Completed second turn! == ");
			}

			ros::spinOnce();
		}

		ROS_INFO(" == FINISHED MAPPING PROCEDURE == ");

		// Spin loop and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}