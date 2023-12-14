#include "ros/ros.h"
#include <regex>
#include "../include/action_handler.h"
#include <unistd.h>
#include "assignment_2/RobotVision.h"
#include <geometry_msgs/Twist.h>

int cameraId;
ros::Publisher cmdVel_pub;

namespace KCL_rosplan {
	ActionInterfaceExtended::ActionInterfaceExtended(ros::NodeHandle &nh) {}
	
	bool ActionInterfaceExtended::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			
			ROS_INFO("*********************** search concreteCallback ***********************");
			// // camera info subscriber
			// ros::Subscriber robotVision_sub = nh.subscribe("info_vision", 100, vision_cb);

			// // publisher robot command
			// ros::Publisher cmdVel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

			// extract number form string
			std::regex rx("[0-9]+");
			std::smatch m;
			std::string str(msg->parameters[2].value);
			regex_search(str, m, rx);
			int targetId = std::stoi(m[0]);
			
			// command to look for the marker Id
			geometry_msgs::Twist cmd_vel;
			cmd_vel.angular.z = 0.5;

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

			// // unsubscribe from camera vision
			// robotVision_sub.shutdown();

			ROS_INFO("Marker %d found. Action (%s) performed: completed!", targetId, msg->name.c_str());
			return true;
		}
}

void vision_cb(const assignment_2::RobotVision::ConstPtr& msg){
	
	// ROS_INFO("**************** vision_cb ****************");
	// update id seen by the camera
	cameraId = msg->id;
	// ROS_INFO("vision_cb - cameraId: %d", cameraId);
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "search_interface", ros::init_options::AnonymousName);
	
	ros::NodeHandle nh;
	
	ros::Subscriber robotVision_sub = nh.subscribe("info_vision", 100, vision_cb);
	
	// inizializzare client al rotation server
	cmdVel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	KCL_rosplan::ActionInterfaceExtended my_aci(nh);
	
	my_aci.runActionInterface();
	
	return 0;
}
