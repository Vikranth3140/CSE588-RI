import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time



class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_nav_node')

        # Goal (x, y)
        self.target_position = (3.5, 2.0)
        self.goal_tolerance = 0.3  # meters
        self.max_duration = 180.0  # seconds

        # Metrics
        self.start_timestamp = time.time()
        self.total_distance = 0.0
        self.collision_warnings = 0
        self.last_position = None
        self.reached_goal = False

        # ROS setup
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.current_cmd = Twist()

        self.get_logger().info('Reactive node started.')

    def odom_callback(self, msg):
        """Handle odometry updates and check for goal or timeout."""
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        self._update_distance(pos_x, pos_y)
        self._check_goal_reached(pos_x, pos_y)
        self._check_timeout()

    def _update_distance(self, x, y):
        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            self.total_distance += math.hypot(dx, dy)
        self.last_position = (x, y)

    def _check_goal_reached(self, x, y):
        dist = math.hypot(x - self.target_position[0], y - self.target_position[1])
        if dist < self.goal_tolerance and not self.reached_goal:
            self.reached_goal = True
            elapsed = time.time() - self.start_timestamp
            self.get_logger().info('Goal reached!')
            self._print_metrics(elapsed)
            self._halt_motion()

    def _check_timeout(self):
        elapsed = time.time() - self.start_timestamp
        if elapsed > self.max_duration and not self.reached_goal:
            self.get_logger().warn('Navigation timed out.')
            self._print_metrics(elapsed)
            self._halt_motion()

    def lidar_callback(self, msg):
        """Process LIDAR data and perform reactive obstacle avoidance."""
        scan = msg.ranges
        if scan and min(scan) < 0.22:
            self.collision_warnings += 1

        # Check for obstacles in front
        front_arcs = list(scan[:8]) + list(scan[-8:])
        min_front = min(front_arcs) if front_arcs else float('inf')

        if min_front < 1.0:
            self._set_motion(0.0, 0.5)  # Turn in place
        else:
            self._set_motion(0.2, 0.0)  # Move forward

        if not self.reached_goal and (time.time() - self.start_timestamp) < self.max_duration:
            self.cmd_vel_pub.publish(self.current_cmd)

    def _set_motion(self, linear, angular):
        self.current_cmd.linear.x = linear
        self.current_cmd.angular.z = angular

    def _halt_motion(self):
        stop = Twist()
        self.cmd_vel_pub.publish(stop)

    def _print_metrics(self, elapsed):
        self.get_logger().info('Navigation Summary:')
        self.get_logger().info(f'  Success         : {"Yes" if self.reached_goal else "No"}')
        self.get_logger().info(f'  Time Elapsed    : {elapsed:.2f} seconds')
        self.get_logger().info(f'  Distance Travelled: {self.total_distance:.2f} meters')
        self.get_logger().info(f'  Near Collisions : {self.collision_warnings}')


def main(args=None):
    """Entry point for the reactive navigation node."""
    rclpy.init(args=args)
    nav_node = ReactiveNavigator()
    rclpy.spin(nav_node)
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()