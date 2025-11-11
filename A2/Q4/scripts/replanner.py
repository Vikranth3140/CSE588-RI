#!/usr/bin/env python3
import rospy
import subprocess
from monitor_motion import check_motion_feasibility

def replan():
    rospy.loginfo("Triggering replan due to motion failure...")
    subprocess.call(["rosservice", "call", "/rosplan_planner_interface/planning_server"])

if __name__ == "__main__":
    rospy.init_node("replanner")
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        feasible = check_motion_feasibility()
        if not feasible:
            replan()
        rate.sleep()
