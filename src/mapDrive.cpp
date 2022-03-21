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
    ros::Publisher pubTurn = node.advertise<std_msgs::Int64>("mapTurn", 1000);
	
	// Set loop rate
    ros::Rate loop_rate(10);
	
    // Main node loop
	while(ros::ok()){

        // Publish target values
		// pub.publish();
		
        // Spin loop and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}