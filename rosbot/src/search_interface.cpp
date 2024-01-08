/**
search_interface.cpp node
Implements the method concreateCallback that is called by ROSPlan to execute the related action
**/

#include "ros/ros.h"
#include <regex>
#include "../include/action_handler.h"
#include <unistd.h>
#include "rosbot/RobotVision.h"
#include <geometry_msgs/Twist.h>

int cameraId;
ros::Publisher cmdVel_pub;

namespace KCL_rosplan {
	ActionInterfaceExtended::ActionInterfaceExtended(ros::NodeHandle &nh) {}

	/*
		Implements the action search called by ROSPlan
		
		args: msg (const rosplan_dispatch_msgs::ActionDispatch::ConstPtr&) action argument

		returns: bool once finished return true. It is not dependent from the action result.
	*/
	bool ActionInterfaceExtended::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			
			ROS_INFO("search concreteCallback");

			// extract number from string
			std::regex rx("[0-9]+");
			std::smatch m;
			std::string str(msg->parameters[2].value);
			regex_search(str, m, rx);
			int targetId = std::stoi(m[0]);
			
			// command to look for the marker Id
			geometry_msgs::Twist cmd_vel;
			cmd_vel.angular.z = 0.5;

			// look for target marker id
			while (targetId != cameraId)
			{
				ROS_INFO("targetId: %d - cameraId: %d", targetId, cameraId);
				// keep on looking for the marker Id
				cmdVel_pub.publish(cmd_vel);
				ros::Duration(0.05).sleep();
			}

			// stop the robot - marker Id found
			cmd_vel.angular.z = 0.0;
			cmdVel_pub.publish(cmd_vel);

			// reset cameraId for new search
			cameraId = 0;

			ROS_INFO("Marker %d found. Action (%s) performed: completed!", targetId, msg->name.c_str());
			
			return true;
		}
}

void vision_cb(const rosbot::RobotVision::ConstPtr& msg){
	/*
	Callback function listening to the topic /info_vision and updating the
	marker id seen by the camera

	args: msg (const rosbot::RobotVision::ConstPtr&) message from topic /info_vision

	returns: void
	*/

	// update id seen by the camera
	cameraId = msg->id;
}


int main(int argc, char **argv) {
	/*
		main function to:
		- initializes the node
		- defines subscriber to /info_vision topic
		- defines publisher to /cmd_vel
		- extends the interface for ROSPlan to execute the action go-home
	*/
	
	ros::init(argc, argv, "search_interface", ros::init_options::AnonymousName);
	
	ros::NodeHandle nh;
	
	ros::Subscriber robotVision_sub = nh.subscribe("info_vision", 100, vision_cb);
	
	// publisher for topic /cmd_vel to drive the rosbot
	cmdVel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	KCL_rosplan::ActionInterfaceExtended my_aci(nh);
	
	my_aci.runActionInterface();
	
	return 0;
}
