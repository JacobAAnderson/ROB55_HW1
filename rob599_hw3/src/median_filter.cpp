/*
	Jacob Anderson
	ROB599 HW3
	Dec 1, 2020
*/
// include the basic ROS stuff, and the Int64 definition.
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"


// This is the callback function.  It should have a void return type, and a single argument that is a reference to a const pointer
// to the message type.  All ROS messages will allow you to use this sort of syntax.

float median(std::vector<float> &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}



class median_filter_node
{

protected:
	ros::NodeHandle node;
	ros::Subscriber sub;
	ros::Publisher  pub;

public:

	median_filter_node(){
		// Set up the subscriber and publisher
		sub = node.subscribe("median_filter/laser_scan_in", 1, &median_filter_node::callback, this);
		pub = node.advertise<sensor_msgs::LaserScan>("median_filter/laser_scan_filterd", 1);
	}


	void callback(const sensor_msgs::LaserScan::ConstPtr& scan) {

		sensor_msgs::LaserScan newScan = *scan;																	//	New Scanline that makes use of an exisiting message

		//	Get filter size from ROS parameter or use the default of 10 if the parameter is not available
		int filterSize;

		if(node.hasParam("/median_filter_size") && node.getParam("/median_filter_size", filterSize));
		else filterSize = 10;


		int size = scan->ranges.size();																					//	Size of the laser scan

		if(filterSize < 2 || filterSize >= size) {															//	Make sure the filter size is reasonable
			pub.publish(newScan);																									//  If not, just publish the raw laserscan
			return;
			}

		ROS_INFO("Filter Size: %d",filterSize);

		// Spcaial Averaging filter
		int a = filterSize/2;
		std::vector<float> ranges;
		std::vector<float> buff;

		for (int i = 0; i < size; i++){																					//	Go through the scanline

			// Sliding window filter
			for(int j = std::max(i-a, 0); j < std::min( i+a, size ); j++ ) buff.push_back( scan->ranges[j] );

			float m = median(buff); 																							//	Get median Value of the sliding window

			buff.clear();																													// 	Clear the buffer fo rthe next pass

			ranges.push_back(m);																									//	Keep Median value
			}

		newScan.ranges = ranges;																								//	Assign the filtered values to the laser scan

		pub.publish(newScan);
	}
};


int main(int argc, char **argv) {
	// Initialize the node and set up the node handle.
	ros::init(argc, argv, "laser_filter");

	median_filter_node mfn;

	ROS_INFO("Laser Filter Node Started");
	// Give control over to ROS.
	ros::spin();

	return 0;
}
