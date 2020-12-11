/*
	Jacob Anderson
	ROB599 HW3
	Dec 1, 2020
*/
// include the basic ROS stuff, and the Int64 definition.
#include <vector>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <random>



class laser_noiser_node
{

protected:
	ros::NodeHandle node;
	ros::Subscriber sub;
	ros::Publisher  pub;


//  float var;

public:

	laser_noiser_node(){
		// Set up the subscriber and publisher
    sub = node.subscribe("/noiser/laser_scan_in", 1, &laser_noiser_node::callback, this);
		pub = node.advertise<sensor_msgs::LaserScan>("/noiser/laser_scan_noised", 1);
	}


	void callback(const sensor_msgs::LaserScan::ConstPtr& scan) {

		sensor_msgs::LaserScan newScan = *scan;

		float var;

		if(node.hasParam("/laser_noise_variance") && node.getParam("/laser_noise_variance", var));
    else var = 0.01;

    ROS_INFO("Setting Variance: %f", var);

    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0, var);

		std::vector<float> ranges;

    int size = scan->ranges.size();

    for (int i = 0; i < size; i++) ranges.push_back( scan->ranges[i] + distribution(generator) );

		newScan.ranges = ranges;

		pub.publish(newScan);
	}
};


int main(int argc, char **argv) {

	ros::init(argc, argv, "laser_noiser");     // Initialize the node and set up the node handle.

	laser_noiser_node lnn;

  ROS_INFO(" Laser Noiser Node Started");

	ros::spin();

	return 0;
}
