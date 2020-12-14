/*
Jacob Anderson
Noat A Project
Dec 13, 2020
*/

#include <cstdlib>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <not_a_project/approach_wallAction.h>


void done_callback(const actionlib::SimpleClientGoalState &state, const not_a_project::approach_wallResultConstPtr &result) {

	if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Succeeded with result: %d", result->arrived);
		}
	else {
		ROS_INFO("Action failed!");
	}
}


void active_callback() {
	ROS_INFO("Action is active");
}


void feedback_callback(const not_a_project::approach_wallFeedbackConstPtr &feedback) {
	ROS_INFO("Feedback: %f", feedback->err);
}


int main(int argc, char **argv) {

	float dist = 1.0;

	if (argc == 2)
		dist = atof(argv[1]);

	// Initialize the node
	ros::init(argc, argv, "Wall_Aproach_action_client");
	ros::NodeHandle node;

	// Create a SimpleActionClient
	actionlib::SimpleActionClient<not_a_project::approach_wallAction> client(node, "wall_aproach", true);
	client.waitForServer();

	// Construct a goal instance and fill in the data fields.
	not_a_project::approach_wallGoal goal;
	goal.distance = dist;

	// Send the goal, registering the appropriate callbacks.
	client.sendGoal(goal, &done_callback, &active_callback, &feedback_callback);

	// Wait until the action is done.
	client.waitForResult();

	return 0;
}
