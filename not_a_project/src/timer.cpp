/*
	Jacob Anderson
	ROB599 HW3
	Dec 10, 2020
*/
// General stuff
#include <numeric>
#include <fstream>
#include <stdio.h>  /* defines FILENAME_MAX */
#include <unistd.h>
#include <string>
#include <vector>

// ROS Stuff
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <not_a_project/filter.h>

#include "filter_function.h"


// Define variables
#define GetCurrentDir getcwd


// ----- Timer ------
class timer_node {


protected:
	ros::NodeHandle node;                 //  The Node Handel
	ros::Subscriber sub1;                 //  Subscribe to the base scan as our source of laser scans
  ros::Publisher  pub1;                 //  Publish the scan lines filtered by the service
	ros::Publisher  pub2;                 //  Publish the scan lines filtered localy

	ros::ServiceClient client;

	not_a_project::filter service_data;

  std::vector<double> service_times;    //  Duration for the service call
  std::vector<double> local_times;      //  Duration for the local call
  int count; 														// 	Number of messages that have been sent

	std::string file_path;

public:

	timer_node(){
		// -- Set up the subscriber and publisher
    sub1 = node.subscribe("base_scan", 1, &timer_node::callback, this);
		pub1 = node.advertise<sensor_msgs::LaserScan>("filtered_laserscan/service", 1);
		pub2 = node.advertise<sensor_msgs::LaserScan>("filtered_laserscan/local", 1);


		// --- Client set up
		client = node.serviceClient<not_a_project::filter>("laser_filter");


		// -- Get the file path that results are being saved too
		char cCurrentPath[FILENAME_MAX];
		if (GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))){
	      cCurrentPath[sizeof(cCurrentPath) - 1] = '\0';                        // not really required
			}
		file_path = std::string(cCurrentPath) +"/AverageTimes.txt";               //	File Path to save results

	 	// -- Initialize the count
	 	count = 0;

	}



	void callback(const sensor_msgs::LaserScan::ConstPtr& scan){                //  Callback to send out the laser scans

		if(count >= 100) return;                                                  //  Stop after 100 Laser Scans

    count++;                                                                  //  Keep track of how many laser scans have been sent

	 	service_data.request.raw = *scan;

		std::cout << "Callback " << count << std::endl;


		// --- Time the sevice call ---
		ros::Time now = ros::Time::now();

		if (!client.call(service_data)){
			ROS_ERROR("Service call failed");
			return;
		}

		ros::Duration dt = ros::Time::now()- now;

		service_times.push_back(dt.toSec());                                      //  Hold onto the duration

		sensor_msgs::LaserScan filtered = service_data.response.filtered;
		pub1.publish(filtered);


		// --- Time the local call ---
		now = ros::Time::now();

		filtered = filter( *scan );

		dt = ros::Time::now()- now;

		local_times.push_back(dt.toSec());                                      //  Hold onto the duration

		pub2.publish(filtered);



		//  Less than 100 scans have been sent, dont compute avarage
		if (count < 100 ) return;

		std::cout << "\n\n\n";

		double average = std::accumulate( service_times.begin(), service_times.end(), 0.0) / service_times.size();
		ROS_INFO("Service Average Time: %f", average);
		writeFile("Service", average);


 		average = std::accumulate( local_times.begin(), local_times.end(), 0.0) / local_times.size();
		ROS_INFO("Local Average Time: %f", average);
		writeFile("Local", average);

	}


  void writeFile(const std::string &name, const double &average){        	// Callback to write avarage to file

		ROS_INFO_STREAM("Writing  " << name << " to File: " << file_path);

		std::ofstream myfile;
  	myfile.open (file_path, std::ios_base::app);
  	myfile << name << "Average Time:" << average << " [sec]" << std::endl;
  	myfile.close();

		ROS_INFO("Done writing file");

  }

}; // --- End timer_node class ---




int main(int argc, char **argv) {
	// Initialize the node and set up the node handle.
	ros::init(argc, argv, "timer_srv_node");

	timer_node tn;

  ROS_INFO("Timer srv Node Started");
	// Give control over to ROS.
	ros::spin();

	return 0;
}
