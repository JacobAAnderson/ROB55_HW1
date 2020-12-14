/*
	Jacob Anderson
	ROB599 Not A Project
	Dec 13, 2020
*/

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"


class filter {

protected:
ros::NodeHandle node;
ros::Subscriber sub;
ros::Publisher  pub;


public:

	filter(){
		sub = node.subscribe("base_scan", 1, &filter::callback, this);
		pub = node.advertise<sensor_msgs::LaserScan>("filtered_laserscan/front", 1);
  }

	void callback(const sensor_msgs::LaserScan &scan){

		sensor_msgs::LaserScan newScan = scan;

		float filterSize = 0.5;                                                        //	Size of filter
		int size = scan.ranges.size();                                             //	Size of the laser scan

		std::vector<float> ranges;
		std::vector<float> intens;
		std::vector<float> angles;

		float angle = scan.angle_min;

		for (int i = 0; i < size; i++){																					//	Go through the scanline

			if (std::abs( std::sin(angle) * scan.ranges[i] ) <= filterSize ){
				ranges.push_back(scan.ranges[i]);
				intens.push_back(scan.intensities[i]);
				angles.push_back(angle);
				}

			angle = angle + scan.angle_increment;
			}

//		std::cout << "Angles: " << angle << ",  " << (scan.angle_max + scan.angle_increment) << std::endl;

		float minAngle = *std::min_element(angles.begin(), angles.end());
		float maxAngle = *std::max_element(angles.begin(), angles.end());

	  newScan.ranges = ranges;
		newScan.angle_min = minAngle;
		newScan.angle_max = maxAngle;
		newScan.intensities = intens;

		pub.publish(newScan);

	}

};	// --- End Fiter ---


int main(int argc, char **argv) {

	// Initialize the node and make a node handle.
	ros::init(argc, argv, "laser_front_filter");

	filter f;

	// Give control over to ROS.
	ros::spin();

	return 0;
}
