#! /usr/bin/env python

import rospy
import os
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceRequest
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_dispatch_msgs.srv import ProblemService, ProblemServiceRequest
from rosplan_dispatch_msgs.srv import PlanningService, PlanningServiceRequest
from rosplan_dispatch_msgs.srv import ParsingService, ParsingServiceRequest
from assignment_2.msg import LogicStates

# global variables
pkg_path = ""
planner_path = ""
data_path = ""
state = LogicStates.PROBLEM_GEN

def action_dispatch_callback(action: ActionDispatch):
    rospy.loginfo("Executing action: %s" % action.name)


def action_feedback_callback(feedback: ActionFeedback):
    if (feedback.status == ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE):
        msg = "Action completed successfully"
        rospy.loginfo(msg)
    elif (feedback.status == ActionFeedback.ACTION_FAILED):
        msg = "Action failed"
        rospy.loginfo(msg)


def start():
    
    global state, pkg_path, planner_path, data_path

    rate = rospy.Rate(10)
    
    while (not rospy.is_shutdown()):
        if state == LogicStates.PROBLEM_GEN:
            rospy.loginfo("robot_logic - generating problem")

            # subscribe to the service to generate the problem
            problem_gen = rospy.ServiceProxy(
                                            "/rosplan_problem_interface/problem_generation_server_params",
                                            ProblemService
                                        )

            # set problem generation request
            req_problem_gen = ProblemServiceRequest()
            req_problem_gen.problem_path = os.path.join(pkg_path, "pddl/generated_problem.pddl")
            req_problem_gen.problem_string_response = False

            # request problem generation
            resp_problem_gen = problem_gen(req_problem_gen)

            state = LogicStates.WAITING_PROBLEM

        
        if state == LogicStates.WAITING_PROBLEM:
            if resp_problem_gen.problem_generated:
                state = LogicStates.PLAN_GEN
            else:
                rospy.loginfo("Error in generating the problem")
                state = LogicStates.ERROR


        if state == LogicStates.PLAN_GEN:
            rospy.loginfo("robot_logic - generating plan")

            # subscribe to planner generator service
            planner_gen = rospy.ServiceProxy(
                                            "/rosplan_planner_interface/planning_server_params",
                                            PlanningService
                                        )
            
            # set request for plan generation
            req_planner = PlanningServiceRequest()
            req_planner.use_problem_topic = True
            req_planner.problem_path = os.path.join(pkg_path, "pddl/planner_problem.pddl")
            req_planner.domain_path = os.path.join(pkg_path, "pddl/domain.pddl")
            req_planner.data_path = os.path.join(pkg_path, data_path)
            req_planner.planner_command = planner_path + " DOMAIN PROBLEM"
            
            # request plan generation
            resp_planner = planner_gen(req_planner)

            state = LogicStates.WAITING_PLAN


        if state == LogicStates.WAITING_PLAN:
            if resp_planner.plan_found:
                rospy.loginfo("Plan found")
                state = LogicStates.PLAN_PARSING
            else:
                rospy.loginfo("Error in generating the plan")
                state = LogicStates.ERROR


        if state == LogicStates.PLAN_PARSING:
            rospy.loginfo("robot_logic - parsing plan")
            # subscribe to parse service
            parser = rospy.ServiceProxy(
                "/rosplan_parsing_interface/parse_plan_from_file",
                ParsingService
            )

            # set request for plan parsing
            req_parse_plan = ParsingServiceRequest()
            # this file name is hard-coded in the rosplan_planning_system package
            req_parse_plan.plan_path = os.path.join(pkg_path, data_path, "plan.pddl")
            
            # request plan parsing
            resp_parse_plan = parser(req_parse_plan)

            state = LogicStates.WAITING_PARSING

        
        if state == LogicStates.WAITING_PARSING:
            if (resp_parse_plan.plan_parsed):
                rospy.loginfo("robot_logic - plan parsed successfully")
                state = LogicStates.DISPATCH_ACTION
            else:
                rospy.loginfo("Error in parsing the plan")
                state = LogicStates.ERROR


        if state == LogicStates.DISPATCH_ACTION:
            rospy.loginfo("robot_logic - dispatching actions")
            # listen to action_dispatch
            action_dispatch_sub = rospy.Subscriber(
                "/rosplan_plan_dispatcher/action_dispatch",
                ActionDispatch,
                action_dispatch_callback
            )

            # listen to action_feedback
            action_feedback_sub = rospy.Subscriber(
                "/rosplan_plan_dispatcher/action_feedback",
                ActionFeedback,
                action_feedback_callback
            )

            # subscribe to action feedback topic
            dispatcher = rospy.ServiceProxy(
                "/rosplan_plan_dispatcher/dispatch_plan",
                DispatchService
            )

            # trigger dispatcher
            req_dispatcher = DispatchServiceRequest()
            resp_dispatcher = dispatcher(req_dispatcher)

            state = LogicStates.WAITING_REACHING_GOAL

        
        if state == LogicStates.WAITING_REACHING_GOAL:
            # success,goal_achieved
            if (resp_dispatcher.goal_achieved):
                rospy.loginfo("Goal achieved!")
                state = LogicStates.EXIT
            else:
                rospy.loginfo("Error while dispatching the plan actions")
                state = LogicStates.ERROR

        if state == LogicStates.EXIT:
            rospy.loginfo("robot_logic - shutting down")
            rospy.signal_shutdown("\nShutting down node on user request...")
            pass

        if state == LogicStates.ERROR:
            rospy.loginfo("error")

        rate.sleep()


def main():

    global state, pkg_path, planner_path, data_path

    # initialize node
    rospy.init_node("robot_logic")

    # wait for service to generate the problem
    rospy.loginfo("waiting for service /rosplan_problem_interface/problem_generation_server_params...")
    rospy.wait_for_service("/rosplan_problem_interface/problem_generation_server_params")
    rospy.loginfo("service found")

    # wait for service to generate the plan
    rospy.loginfo("waiting for service /rosplan_planner_interface/planning_server_params")
    rospy.wait_for_service("/rosplan_planner_interface/planning_server_params")
    rospy.loginfo("service found")
    
    # wait for service to parse the plan
    rospy.loginfo("waiting for service /rosplan_parsing_interface/parse_plan_from_file")
    rospy.wait_for_service("/rosplan_parsing_interface/parse_plan_from_file")
    rospy.loginfo("service found")

    # wait for service dispatch the plan actions
    rospy.loginfo("waiting for service /rosplan_plan_dispatcher/dispatch_plan")
    rospy.wait_for_service("/rosplan_plan_dispatcher/dispatch_plan")
    rospy.loginfo("service found")

    # get package and planner path
    pkg_path = rospy.get_param("pkg_path")
    planner_path = rospy.get_param("planner_path")
    data_path = rospy.get_param("data_path")

    # starting state for the state machine
    state = LogicStates.PROBLEM_GEN
    # state = LogicStates.BRAKE

    rospy.loginfo("robot_logic node initialized")
    
    start()

    rospy.spin()


if __name__ == "__main__":
    main()