/*
	Jacob Anderson
	ROB599 HW3
	Dec 10, 2020
*/

#include <vector>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <numeric>
#include <fstream>

class timer_node {


protected:
	ros::NodeHandle node;                 //  The Node Handel
	ros::Subscriber sub1;                 //  Subscribe to the base scan as our source of laser scans
  ros::Subscriber sub2;                 //  Subscribe to the returning scan lines
	ros::Publisher  pub;                  //  Publish the scan lines that are being timed

  std::vector<double> trip_times;       //  Duration for the other nodes to do their work
  std::map<int, ros::Time> manifest;    //  Keeps track of when the outgoing scan was sent
  int count;


public:

	timer_node(){

		// Set up the subscriber and publisher
    sub1 = node.subscribe("base_scan", 1, &timer_node::callback1, this);
    sub2 = node.subscribe("timer/clocked_scan", 100, &timer_node::callback2, this);
		pub = node.advertise<sensor_msgs::LaserScan>("timer/queued_scan", 1);

    // Initialize the count
    count = 0;
	}


	void callback1(const sensor_msgs::LaserScan::ConstPtr& scan){             	//  Callback to send out the laser scans

		if(count > 100) return;                                                 	//  Stop after 100 Laser Scans

    count++;                                                                	//  Keep track of how many laser scans have been sent

    manifest.insert(std::pair<int, ros::Time> (count, ros::Time::now()) );  	//  Make note of current time

		pub.publish(*scan);                                                     	//  Send out the scan line
	}


  void callback2(const sensor_msgs::LaserScan::ConstPtr& scan){             	// Callback for the returning laser scans

  	if ( manifest.find(scan->header.seq) == manifest.end() ) return;          //  Make Sure the key exists

  	ros::Duration dt = ros::Time::now()- manifest[scan->header.seq];          //  Elapsed time for nodes to work

  	trip_times.push_back(dt.toSec());                                         //  Hold onto the duration

  	ROS_INFO("Duration: %f", dt.toSec() );

  	if(scan->header.seq == 100){                                              //  After 100 messages, find the average time

    	double average = std::accumulate( trip_times.begin(), trip_times.end(), 0.0) / trip_times.size();

    	ROS_INFO("Average: %f\n\n\n\n", average);

			// Write Avarage to file
			std::ofstream myfile;
  		myfile.open ("/home/ubuntu/rob599_homeWork/cpp_average_time.txt");
  		myfile << "Average Time:" << average << std::endl;
  		myfile.close();

    	}
  }

};




int main(int argc, char **argv) {
	// Initialize the node and set up the node handle.
	ros::init(argc, argv, "timer_noiser");

	timer_node tn;

  ROS_INFO("Timer Node Started");
	// Give control over to ROS.
	ros::spin();

	return 0;
}
