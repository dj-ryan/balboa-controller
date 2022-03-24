#include "ros/ros.h"
#include "balboa_core/balboaLL.h"
#include "std_msgs/Int32.h"
#include <cmath>
#include <map>
#include <string>

// Declare and initialize variables
int IR1;
int IR2;
int IR4;
int IR5;

// Print ASCII characters corresponding to mapped sensor values
void printMap(int IRmap1[20][4], int IRmap2[20][4], int IRmap4[20][4], int IRmap5[20][4])
{
	// Create variable to store ASCII values
	char fullMap[20][4][4];
	
	// Determine which white/gray/black ASCII character to store 
	for (int i = 0; i < 4; i++)
	{
		for(int j = 0; j< 20; j++) {
			
			// Sensor 1
			if (IRmap1[j][i] > 2000) //black, larger because shadows
			{
				fullMap[j][0][i] = '0';
			}
			else if (IRmap1[j][i] <= 2000 && IRmap1[j][i] >= 600) //gray
			{
				fullMap[j][0][i] = 'o';	
			}
			else //white
			{
				fullMap[j][0][i] = '_';
			}

			// Sensor 2
			if (IRmap2[j][i] > 1300) //black
			{
				fullMap[j][1][i] = '0';
			}
			else if (IRmap2[j][i] <= 1300 && IRmap2[j][i] >= 600) //gray
			{
				fullMap[j][1][i] = 'o';
			}
			else //white
			{
				fullMap[j][1][i] = '_';
			}

			// Sensor 4
			if (IRmap4[j][i] > 1300) //black
			{
				fullMap[j][2][i] = '0';
			}
			else if (IRmap4[j][i] <= 1300 && IRmap4[j][i] >= 600) //gray
			{
				fullMap[j][2][i] = 'o';
			}
			else //white
			{
				fullMap[j][2][i] = '_';
			}

			// Sensor 5
			if (IRmap5[j][i] > 2000) //black, larger because shadows
			{
				fullMap[j][3][i] = '0';
			}
			else if (IRmap5[j][i] <= 2000 && IRmap5[j][i] >= 600) //gray
			{
				fullMap[j][3][i] = 'o';
			}
			else //white
			{
				fullMap[j][3][i] = '_';
			}
		}
	}

	// Print ASCII characters to console in map grid  
	for (int i = 0; i < 4; i++)
	{
		for (int k = 0; k < 4; k++) 
		{
			ROS_INFO("%c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c",fullMap[0][k][i],fullMap[1][k][i],fullMap[2][k][i],fullMap[3][k][i],fullMap[4][k][i],fullMap[5][k][i],fullMap[6][k][i],fullMap[7][k][i],fullMap[8][k][i],fullMap[9][k][i],fullMap[10][k][i],fullMap[11][k][i],fullMap[12][k][i],fullMap[13][k][i],fullMap[14][k][i],fullMap[15][k][i],fullMap[16][k][i],fullMap[17][k][i],fullMap[18][k][i],fullMap[19][k][i]);
		}
	}

}

// Store robot IR data when received
void callbackIMU(const balboa_core::balboaLL &msg)
{
	// Update robot IR data to use later, just evenly spaced sensors 
	IR1 = msg.IRsensor1;
	IR2 = msg.IRsensor2;
	IR4 = msg.IRsensor4;
	IR5 = msg.IRsensor5;
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
	int rightTurn = 22000;
	int leftTurn = -22000;

	// Create variable for storing reflectance sensor data for mapping
	int snakeS1[20][4];
	int snakeS2[20][4];
	int snakeS4[20][4];
	int snakeS5[20][4];

	// Set loop rate
	ros::Rate loop_rate(0.75);

	// Main node loop
	while (ros::ok())
	{
		// Drive robot through totalRows, and then stop
		if (doneOnce == 0)
		{
			doneOnce = 1;
			ros::spinOnce();

			// Drive robot through totalRows
			for (int i = 0; i < totalRows; i++)
			{
				ros::spinOnce();
				loop_rate.sleep();

				// Drive the first row, 20 steps of approximately 1.5in at a time 
				for (int j = 0; j < 20; j++)
				{
					// Grab latest sensor data
					ros::spinOnce();
					loop_rate.sleep();

					if (i % 2 == 0) 
					{
						// Capture reflectance sensor data L>R
						snakeS1[j][i] = IR1;
						snakeS2[j][i] = IR2;
						snakeS4[j][i] = IR4;
						snakeS5[j][i] = IR5;
					}
					else
					{
						// Capture reflectance sensor data R>L 
						snakeS1[19-j][i] = IR1;
						snakeS2[19-j][i] = IR2;
						snakeS4[19-j][i] = IR4;
						snakeS5[19-j][i] = IR5;
					}

					// Publish next step 
					targetDrive.data = 200;
					pubDrive.publish(targetDrive);

					ROS_INFO("Sent target: %d", targetDrive);
					ROS_INFO("IR sensor data: %d , %d , %d , %d", snakeS1[j][i], snakeS2[j][i], snakeS4[j][i], snakeS5[j][i]);

				}

				ROS_INFO(" == FINISHED ROW ==");

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
					
					ROS_INFO(" == Completed first turn == ");

					// Drive to next row, approx. robot width w left turn
					targetDrive.data = 100;
					pubDrive.publish(targetDrive);
					loop_rate.sleep();
					
					ROS_INFO(" == Arrived at next row == ");
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

					ROS_INFO(" == Completed first turn == ");

					// Drive to next row, approx. robot width w right turn
					targetDrive.data = 1200;
					pubDrive.publish(targetDrive);
					loop_rate.sleep();
					
					ROS_INFO(" == Arrived at next row == ");					
				}

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

					// Realign with previous edge
					targetDrive.data = 200;
					pubDrive.publish(targetDrive);
					loop_rate.sleep();
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

					// Realign with previous edge
					targetDrive.data = 200;
					pubDrive.publish(targetDrive);
					loop_rate.sleep();
				}

				ROS_INFO(" == Completed second turn! == ");
			}

			ros::spinOnce();
		}

		ROS_INFO(" == FINISHED MAPPING PROCEDURE == ");

		// Send collected sensor data to mapping function 
		printMap(snakeS1, snakeS2, snakeS4, snakeS5);

		// Spin loop and sleep
		// ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}