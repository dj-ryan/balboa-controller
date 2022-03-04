#include "ros/ros.h"
#include "std_msgs/String.h"
#include "balboa_core/balboaLL.h"
#include "balboa_controller/irArray.h"

#include <map>


balboa_core::balboaLL robot;
balboa_controller::irArray ir;

int resolution = 50;
int tick = 0;
int rowLength = 30;

std::map<int,balboa_controller::irArray> row;

//void initMap() {
//	balboa_controller::irArray emptyIR;
//	emptyIR.sensor1 = 0;
//	emptyIR.sensor2 = 0;
//	emptyIR.sensor4 = 0;
//	emptyIR.sensor5 = 0;
//	
//	for(int i = 0; i <= rowLength; i++) {
//		int tick = i * resolution;
//		row.insert({tick,emtpyIR});
//	}	
//}


void callbackIR(const balboa_controller::irArray::ConstPtr& data)
{
	ir = *data;
}

void callbackEncoder(const balboa_core::balboaLL::ConstPtr& data)
{
	if((data.distanceRight % resolution) == 0){
		currentTick = data.distance / resolution;
		row.insert_or_assign({currentTick,ir});
		ROS_INFO("inserted: row[%d] = %d, %d, %d, %d",currentTick, ir.sensor1, ir.sensor2, ir.sensor4, ir.sensor5);
	}

}

int main(int argc, char **argv)
{
	
	//initMap();
	
	ROS_INFO("STARTED");
	ros::init(argc, argv, "mapAlgo");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("irData", 1000, callbackIR);
	ros::Subscriber sub = n.subscribe("balboaLL", 1000, callbackEncoder);
	
	ros::Rate loop_rate(1);
	
	return 0;
}
