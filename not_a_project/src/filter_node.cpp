/*
	Jacob Anderson
	ROB599 Not A Project
	Dec 13, 2020
*/

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <not_a_project/filter.h>

#include "filter_function.h"


bool filter_srv(not_a_project::filter::Request &request,
								not_a_project::filter::Response &response)
{
	ROS_INFO("Service Got Mesage");

	sensor_msgs::LaserScan newScan = request.raw;

	response.filtered = filter( newScan );

	return true;

}


int main(int argc, char **argv) {

	// Initialize the node and make a node handle.
	ros::init(argc, argv, "laser_filter");
	ros::NodeHandle node;

	// Set up the service, using NodeHandle.advertiseService.  This takes a service name, and a callback
	// function.
	ros::ServiceServer service = node.advertiseService("laser_filter", filter_srv);
	ROS_INFO("Laser Filter Service Started");

	// Give control over to ROS.
	ros::spin();

	return 0;
}
