#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen, Empty
import time
import math

class TurtleHouseDrawer:
    def __init__(self):
        rospy.init_node("draw_house", anonymous=True)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(50)  # control loop frequency

        # Service clients
        rospy.wait_for_service('/turtle1/teleport_absolute')
        self.teleport_srv = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        rospy.wait_for_service('/turtle1/set_pen')
        self.setpen_srv = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        rospy.wait_for_service('/reset')
        self.reset_srv = rospy.ServiceProxy('/reset', Empty)

    def move_straight(self, distance, speed=1.0):
        vel = Twist()
        vel.linear.x = speed
        duration = distance / speed
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < duration:
            self.pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0
        self.pub.publish(vel)

    def rotate(self, angle_deg, speed=1.0):
        vel = Twist()
        angle_rad = math.radians(angle_deg)
        vel.angular.z = speed if angle_rad > 0 else -speed
        duration = abs(angle_rad) / speed
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < duration:
            self.pub.publish(vel)
            self.rate.sleep()
        vel.angular.z = 0
        self.pub.publish(vel)

    def teleport(self, x, y, theta=0.0):
        self.setpen_srv(0, 0, 0, 0, 1)  # lift pen
        self.teleport_srv(x, y, theta)
        self.setpen_srv(255, 255, 255, 2, 0)  # put pen down

    def draw_rectangle(self, x, y, width, height):
        self.teleport(x, y, 0)
        for _ in range(2):
            self.move_straight(width)
            self.rotate(90)
            self.move_straight(height)
            self.rotate(90)

    def draw_triangle(self, x, y, width, height):
        self.teleport(x, y, 0)
        # base
        self.move_straight(width)
        # left roof side
        self.rotate(150)
        slant = math.sqrt((width/2)**2 + height**2)
        self.move_straight(slant)
        # right roof side
        self.rotate(60)
        self.move_straight(slant)
        self.rotate(150)

    def draw_house(self, x, y):
        # Main wall
        self.draw_rectangle(x, y, 5.0, 4.5)
        # Roof
        self.draw_triangle(x, y+4.5, 5.0, 4.0)
        # Window
        self.draw_rectangle(x+0.5, y+2.0, 1.0, 1.0)
        # Door
        self.draw_rectangle(x+4.0, y, 1.0, 2.5)

if __name__ == "__main__":
    drawer = TurtleHouseDrawer()

    # Read coordinates from file
    with open("assignment1Input.txt") as f:
        coords = [tuple(map(float, line.split())) for line in f]

    for (x, y) in coords:
        drawer.draw_house(x, y)
        input("✅ House drawn at ({}, {}). Press Enter to continue...".format(x, y))
        drawer.reset_srv()
