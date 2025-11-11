#!/usr/bin/env python3
import random
import rospy

def check_motion_feasibility():
    return random.random() > 0.3

if __name__ == "__main__":
    rospy.init_node("motion_monitor")
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        feasible = check_motion_feasibility()
        rospy.loginfo(f"Motion feasibility: {'OK' if feasible else 'FAILED'}")
        rate.sleep()
