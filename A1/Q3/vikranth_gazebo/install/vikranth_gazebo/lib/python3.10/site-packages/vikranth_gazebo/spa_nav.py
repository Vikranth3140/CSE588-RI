import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
import math
import time

class SPANavigator(Node):
    def __init__(self):
        super().__init__('spa_navigator')
        self._nav_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_pose = self._make_goal()
        self._goal_sent = False
        self._start_time = time.time()
        self._success = False
        self._collisions = 0
        self._distance = 0.0
        self._last_pos = None
        self._goal_radius = 0.25
        self._timeout = 120
        self._done = False
        self._stuck_time = None
        self._stuck_warned = False
        self._stuck_threshold = 7.0  # seconds

        self.create_subscription(Odometry, 'odom', self._odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)

        # Try to send goal immediately if server is ready
        self._send_goal_timer = self.create_timer(0.5, self._send_goal_if_ready)

    def _make_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 3.5
        goal.pose.position.y = -1.0
        goal.pose.orientation.z = 0.707
        goal.pose.orientation.w = 0.707
        return goal

    def _send_goal_if_ready(self):
        if not self._goal_sent and self._nav_action.wait_for_server(timeout_sec=0.2):
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = self._goal_pose
            self._nav_action.send_goal_async(nav_goal)
            self._goal_sent = True
            self.get_logger().info('SPA Navigator: Goal dispatched.')
            self._send_goal_timer.cancel()

    def _odom_callback(self, msg):
        pos = msg.pose.pose.position
        # Distance calculation
        if self._last_pos is not None:
            dx = pos.x - self._last_pos.x
            dy = pos.y - self._last_pos.y
            step = math.hypot(dx, dy)
            self._distance += step
            # Stuck detection
            if step < 0.01:
                if self._stuck_time is None:
                    self._stuck_time = time.time()
                elif not self._stuck_warned and (time.time() - self._stuck_time) > self._stuck_threshold:
                    self.get_logger().warn('Robot may be stuck!')
                    self._stuck_warned = True
            else:
                self._stuck_time = None
                self._stuck_warned = False
        self._last_pos = pos

        # Goal check
        dist = math.hypot(pos.x - self._goal_pose.pose.position.x, pos.y - self._goal_pose.pose.position.y)
        if not self._done and dist < self._goal_radius:
            self._done = True
            self._success = True
            self.get_logger().info('Goal reached!')
            self._log_stats()
            self._shutdown()
            return

        # Timeout
        elapsed = time.time() - self._start_time
        if elapsed > self._timeout and not self._done:
            self.get_logger().error('Timeout reached!')
            self._log_stats()
            self._shutdown()

    def _scan_callback(self, msg):
        clean_ranges = [r if not (math.isinf(r) or math.isnan(r)) else msg.range_max for r in msg.ranges]
        if min(clean_ranges) < 0.20:
            self._collisions += 1

    def _log_stats(self):
        elapsed = time.time() - self._start_time
        self.get_logger().info(f"=== SPA Navigator Stats ===")
        self.get_logger().info(f"Success: {int(self._success)}")
        self.get_logger().info(f"Elapsed Time (s): {min(elapsed, self._timeout):.2f}")
        self.get_logger().info(f"Near-collisions: {self._collisions}")
        self.get_logger().info(f"Total Path (m): {self._distance:.2f}")
        self.get_logger().info(f"=============================")

    def _shutdown(self):
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SPANavigator()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()