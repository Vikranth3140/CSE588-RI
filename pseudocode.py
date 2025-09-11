#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import time

def draw_line(distance, speed=1.0):
    vel = Twist()
    vel.linear.x = speed
    duration = distance / speed
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < duration:
        pub.publish(vel)
        rate.sleep()
    vel.linear.x = 0
    pub.publish(vel)

def turn(angle_deg, speed=1.0):
    vel = Twist()
    angular_speed = speed
    angle_rad = angle_deg * 3.14159 / 180.0
    duration = angle_rad / angular_speed
    vel.angular.z = angular_speed
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < duration:
        pub.publish(vel)
        rate.sleep()
    vel.angular.z = 0
    pub.publish(vel)

def teleport(x, y, theta=0):
    rospy.wait_for_service('/turtle1/teleport_absolute')
    teleport_srv = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
    teleport_srv(x, y, theta)

def draw_rectangle(x, y, width, height):
    teleport(x, y, 0)
    for _ in range(2):
        draw_line(width)
        turn(90)
        draw_line(height)
        turn(90)

def draw_triangle(x, y, width, height):
    teleport(x, y, 0)
    draw_line(width)     # base
    turn(150)
    draw_line((width**2 + height**2)**0.5)
    turn(60)
    draw_line((width**2 + height**2)**0.5)
    turn(150)

def draw_house(x, y):
    draw_rectangle(x, y, 5, 4.5)
    draw_triangle(x, y+4.5, 5, 4.0)
    draw_rectangle(x+0.5, y+2.0, 1, 1)   # window
    draw_rectangle(x+4.0, y, 1, 2.5)     # door

if __name__ == "__main__":
    rospy.init_node("draw_house")
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Read coordinates from file
    with open("assignment1Input.txt") as f:
        coords = [tuple(map(float, line.split())) for line in f]

    for (x, y) in coords:
        draw_house(x, y)
        input("Press Enter for next house...")
        rospy.wait_for_service('/reset')
        rospy.ServiceProxy('/reset', Empty)()
