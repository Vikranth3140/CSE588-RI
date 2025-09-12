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
        self.get_logger().info(f"Moving straight: distance={distance}, speed={speed}")
        vel = Twist()
        vel.linear.x = speed
        duration = distance / speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(vel)
            time.sleep(0.02)  # 50Hz update rate
            
        vel.linear.x = 0.0
        self.pub.publish(vel)
        self.get_logger().info(f"Finished moving straight")

    def rotate(self, angle_deg, speed=1.0):
        self.get_logger().info(f"Rotating: angle={angle_deg} degrees, speed={speed}")
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
        self.get_logger().info(f"Finished rotating")

    def teleport(self, x, y, theta=0.0):
        self.get_logger().info(f"Teleporting to ({x}, {y}, {theta})")
        
        # Lift pen
        pen_req = SetPen.Request()
        pen_req.r = pen_req.g = pen_req.b = 0
        pen_req.width = 0
        pen_req.off = 1
        future = self.setpen_cli.call_async(pen_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().warn("Set pen (lift) service call timed out")
            return

        # Teleport
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        future = self.teleport_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().warn("Teleport service call timed out")
            return

        # Put pen down
        pen_req = SetPen.Request()
        pen_req.r = pen_req.g = pen_req.b = 255
        pen_req.width = 2
        pen_req.off = 0
        future = self.setpen_cli.call_async(pen_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().warn("Set pen (down) service call timed out")
            return
            
        self.get_logger().info("Teleport completed successfully")

    # ---- Shape Drawing ----
    def draw_rectangle(self, x, y, width, height):
        self.teleport(x, y, 0.0)
        for _ in range(2):
            self.move_straight(width)
            self.rotate(90)
            self.move_straight(height)
            self.rotate(90)

    def draw_triangle(self, x, y, width, height):
        self.teleport(x, y, 0.0)
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
