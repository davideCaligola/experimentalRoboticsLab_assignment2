#include "../include/action_handler.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string.h>
#include <unordered_map>
#include <tuple>

namespace KCL_rosplan {
	ActionInterfaceExtended::ActionInterfaceExtended(ros::NodeHandle &nh) {}
	
	bool ActionInterfaceExtended::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
			
			ROS_INFO("******************* goto concreteCallback *******************************");

			std::unordered_map<std::string, std::tuple<float,float>> markerPositions = {
				{"wp0", std::make_tuple(0.0, 1.0)},
				{"wp1", std::make_tuple(5.5, 2.5)},	// original (6.0,2.0)
				{"wp2", std::make_tuple(7.0, -5.0)},
				{"wp3", std::make_tuple(-3.0, -8.0)},
				{"wp4", std::make_tuple(-7.0, 1.0)} // original (-7.0, -1.5) it is behind, cannot see the marker
			};
			
			std::tuple<float,float> coordinates = markerPositions.at(msg->parameters[2].value);

			//inizializzare client to movebase
			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebase_client("move_base", true);
			
			// wait for the action server to come up
  			while(!movebase_client.waitForServer(ros::Duration(5.0))){
  				ROS_INFO("Waiting for the move_base action server to come up");
  			}

			// define goal to reach
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = std::get<0>(coordinates);
			goal.target_pose.pose.position.y = std::get<1>(coordinates);
			goal.target_pose.pose.orientation.w = 1.0;

			ROS_INFO("**************************** robot_move sending goal ****************************");
			movebase_client.sendGoal(goal);

			ROS_INFO("**************************** waiting for result ****************************");
			// wait for the action result
			movebase_client.waitForResult();
			ROS_INFO("**************************** result done ****************************");

			if (movebase_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("robot reached target position");
			else
				ROS_INFO("robot missed target position for some reason");
			
			
			// here the implementation of the action
			// std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
			
			ROS_INFO("*********** Action (%s) performed: completed! ********************", msg->name.c_str());
			
			return true;
		}
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "goto_interface", ros::init_options::AnonymousName);
	
	ros::NodeHandle nh("~");
	
	// inizialiizzare client to movebase
	
	KCL_rosplan::ActionInterfaceExtended my_aci(nh);
	
	my_aci.runActionInterface();
	
	return 0;
}
