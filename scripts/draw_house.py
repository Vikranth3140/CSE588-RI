#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty
import time

class TurtleHouseDrawer(Node):
    def __init__(self):
        super().__init__("draw_house")
        self.current_orientation_rad = 0.0

        # Publisher for movement
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.rate = self.create_rate(50)

        # Create service clients
        self.teleport_cli = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        self.setpen_cli = self.create_client(SetPen, "/turtle1/set_pen")
        self.reset_cli = self.create_client(Empty, "/reset")

        # Wait for services
        for cli, name in [
            (self.teleport_cli, "teleport_absolute"),
            (self.setpen_cli, "set_pen"),
            (self.reset_cli, "reset"),
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"{name} service not available, waiting...")

    # ---- Movement Primitives ----
    def move_straight(self, distance, speed=1.0):
        vel = Twist()
        vel.linear.x = speed
        duration = distance / speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(vel)
            time.sleep(0.02)  # 50Hz update rate
        
        vel.linear.x = 0.0
        self.pub.publish(vel)

    def rotate(self, angle_deg, speed=1.0):
        vel = Twist()
        angle_rad = math.radians(angle_deg)
        vel.angular.z = speed if angle_rad > 0 else -speed
        duration = abs(angle_rad) / speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(vel)
            time.sleep(0.02)  # 50Hz update rate
        
        vel.angular.z = 0.0
        self.pub.publish(vel)
        self.current_orientation_rad += angle_rad

    def set_pen(self, on, r=255, g=255, b=255, width=2):
        pen_req = SetPen.Request()
        if on:
            pen_req.r, pen_req.g, pen_req.b, pen_req.width, pen_req.off = r, g, b, width, 0
        else:
            pen_req.r, pen_req.g, pen_req.b, pen_req.width, pen_req.off = 0, 0, 0, 0, 1
        
        future = self.setpen_cli.call_async(pen_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().warn("Set pen service call timed out")

    def move_to(self, x, y):
        self.set_pen(False) # Pen up

        # Use teleport for repositioning without drawing
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), 0.0
        future = self.teleport_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().warn("Teleport service call timed out")
            return
        self.current_orientation_rad = 0.0

        self.set_pen(True) # Pen down

    def teleport(self, x, y, theta=0.0):
        # Lift pen
        self.set_pen(False)

        # Teleport
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        future = self.teleport_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().warn("Teleport service call timed out")
            return
        self.current_orientation_rad = theta

        # Put pen down
        self.set_pen(True)

    # ---- Shape Drawing ----
    def draw_rectangle(self, x, y, width, height):
        self.move_to(x, y)
        for _ in range(2):
            self.move_straight(width)
            self.rotate(90)
            self.move_straight(height)
            self.rotate(90)

    def draw_triangle(self, x, y, width, height):
        # This function is now only used for the roof.
        # The base of the roof is the top of the wall, which is already drawn.
        # We start at the top-left of the wall, go to the peak, then to the top-right.
        self.move_to(x, y)

        # Go to the peak
        peak_x = x + width / 2
        peak_y = y + height
        angle_to_peak = math.atan2(peak_y - y, peak_x - x)
        dist_to_peak = math.sqrt((peak_x - x)**2 + (peak_y - y)**2)
        
        rotation_angle = angle_to_peak - self.current_orientation_rad
        self.rotate(math.degrees(rotation_angle))
        self.move_straight(dist_to_peak)

        # Go to the top-right corner
        top_right_x = x + width
        top_right_y = y
        angle_to_right = math.atan2(top_right_y - peak_y, top_right_x - peak_x)
        dist_to_right = math.sqrt((top_right_x - peak_x)**2 + (top_right_y - peak_y)**2)

        rotation_angle = angle_to_right - self.current_orientation_rad
        self.rotate(math.degrees(rotation_angle))
        self.move_straight(dist_to_right)

    def draw_house(self, x, y):
        # Wall
        wall_width = 5.0
        wall_height = 4.5
        self.draw_rectangle(x, y, wall_width, wall_height)

        # Roof
        roof_base = 5.0
        roof_height = 4.0
        self.draw_triangle(x, y + wall_height, roof_base, roof_height)

        # Window
        window_size = 1.0
        window_offset_left = 0.5
        window_offset_bottom = 2.0
        self.draw_rectangle(x + window_offset_left, y + window_offset_bottom, window_size, window_size)

        # Door
        door_width = 1.0
        door_height = 2.5
        door_offset_right = 1.0
        door_x = x + wall_width - door_offset_right - door_width
        self.draw_rectangle(door_x, y, door_width, door_height)

    def reset_screen(self):
        req = Empty.Request()
        future = self.reset_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    drawer = TurtleHouseDrawer()

    # Read coordinates from input file
    with open("/home/vikranth/ros2_ws/src/house_drawer/assignment1Input.txt") as f:
        coords = [tuple(map(float, line.split())) for line in f]

    for (x, y) in coords:
        drawer.get_logger().info(f"Drawing house at ({x}, {y})...")
        drawer.draw_house(x, y)
        input("✅ House drawn. Press Enter to continue...")
        drawer.reset_screen()

    drawer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
