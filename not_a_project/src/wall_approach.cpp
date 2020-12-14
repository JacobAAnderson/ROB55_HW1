/*
	Jacob Anderson
	ROB599 Not A Project
	Dec 13, 2020
*/
// General stuff
#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>
#include <vector>

// ROS Stuff
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <actionlib/server/simple_action_server.h>
#include <not_a_project/stopping_distance.h>
#include <not_a_project/approach_wallAction.h>

void kalmanFilter(float &x, float &sig, const float &u, const float &z,
	 								const float &A, const float &B, const float &Q, const float &R)
									{

	float X_ = A * x  +  B * u;
	float P_ = A * sig * A + Q;

// K = Err in estimate / ( err in eatimate + err in measurment)
	float K = P_/(P_ + R);

	x = X_ + K * (z - X_);

// sigma = (1-K) * Err in estimate
	sig = (1 - K) * P_;

	}


// ----- Timer ------
class Wall_Aproach {


protected:
	ros::NodeHandle node;                 //  The Node Handel
	ros::Subscriber sub;                 //  Subscribe to the base scan as our source of laser scans
  ros::Publisher  pub;                 //  Publish the scan lines filtered by the service
	ros::ServiceServer service;
	ros::Time t_last;

//	actionlib::SimpleActionServer<not_a_project::approach_wallAction> server;

	geometry_msgs::Twist twist;

	float x;
	float sig;
	float speed;
	float is_cal;
	float distance;
	float lidar_STD;

	std::vector<float> cal_set;

public:

	Wall_Aproach(){
		// -- Set up the subscriber and publisher
    sub = node.subscribe("filtered_laserscan/front", 1, &Wall_Aproach::callback, this);
		pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		service = node.advertiseService("/stopping_distance", &Wall_Aproach::sd_callback, this);

//		server(node, "wall_aproach", boost::bind(&Wall_Aproach::action_callback, _1, &server), false)
//		server.start();
//		ROS_INFO("wall_aproach action server started");

		x = 0.0;
		sig = 0.0;
		speed = 0.0;
		is_cal = false;
		distance = 1.0;
		lidar_STD = 0.0;
		t_last = ros::Time::now();

		// -- Initialize Twist Command
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

		ROS_INFO("Lidar Calibrating");

	}



	void callback(const sensor_msgs::LaserScan::ConstPtr& scan){                //  Callback to send out the laser scans

		std::vector<float> ranges = scan->ranges;

		float min_range = *std::min_element(ranges.begin(), ranges.end());

		if(!is_cal && cal_set.size() <10){                                                  //  Get a calibration set of 100
			cal_set.push_back(min_range);
			return;
		}

		if( !is_cal){
			float sum    = std::accumulate(cal_set.begin(), cal_set.end(), 0.0);
			float mean   = sum / cal_set.size();
			float sq_sum = std::inner_product(cal_set.begin(), cal_set.end(), cal_set.begin(), 0.0);

			lidar_STD = std::sqrt(sq_sum / cal_set.size() - mean * mean);
			sig = lidar_STD;
			t_last = ros::Time::now();
			is_cal = true;

			ROS_INFO("Lidar Calibration Finised");
			}

		ros::Duration dt = ros::Time::now()- t_last;
		t_last = ros::Time::now();

		// -- Kalman filter the min range ---
		float A = 1.0;
		float B = -dt.toSec();
		float Q = 0.05;
		float R = lidar_STD;

//		std::cout << "\nBefore: " << x << ", " << sig << std::endl;
		kalmanFilter(x, sig, speed, min_range, A, B, Q, R);
//		std::cout << "After: " << x << ", " << sig << "\n" << std::endl;


		// -- Stoping distance error and speed command--
		float error = x - distance;

		if( std::abs(error) > lidar_STD * 3) speed = std::tanh(error * 2.0);
		else speed = 0.0;

//		std::cout << "Dist: " << distance << ", Min Range: " <<min_range << ", Error: " << error << ", Speed: " << speed << std::endl;

		twist.linear.x = speed;

		pub.publish(twist);


	}


	bool sd_callback(not_a_project::stopping_distance::Request &request,
									not_a_project::stopping_distance::Response &response){

		float dist = request.stopping_distance;

		std::cout << "\n\n\nSet dist: " << dist << "\n\n\n" << std::endl;

		if(std::isnan(dist) || dist < 0.5 ) response.set = false;
		else{
			distance = dist;
			response.set = true;
			}

		return true;
	}


	void action_callback(const not_a_project::approach_wallActionGoalConstPtr &goal,
		                   actionlib::SimpleActionServer<not_a_project::approach_wallAction> *server)
		{
/*
		ROS_INFO("Goal Recived: %f", goal->distance);

		float dist = goal->distance;

		if(std::isnan(dist) || dist < 0.5 ){
			server->setPreempted();
			return;
		}
		else{
			distance = dist;
			response.set = true;
			}
*/
		not_a_project::approach_wallActionResult result;
		}

}; // --- End Wall_Aproach class ---




int main(int argc, char **argv) {
	// Initialize the node and set up the node handle.
	ros::init(argc, argv, "wall_aproach_node");

	Wall_Aproach wa;
  ROS_INFO("staring_at_wall Node Started");

	// Give control over to ROS.
	ros::spin();

	return 0;
}
