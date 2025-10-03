import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
import math
import time


class HybridNavigator(Node):
    def __init__(self):
        super().__init__('hybrid_navigator')
        self.nav_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.target_pose = self._build_target_pose()
        self.goal_dispatched = False
        self.active_goal_handle = None

        # Performance metrics
        self.run_start = time.time()
        self.is_successful = 0
        self.collision_warnings = 0
        self.total_distance = 0.0
        self.prev_odom_pos = None
        self.reach_tolerance = 0.3
        self.max_duration = 180
        self.has_arrived = False
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Hybrid navigation state
        self.last_move_time = time.time()
        self.last_move_pos = None
        self.min_progress = 0.05  # meters
        self.stuck_limit = 5.0  # seconds
        self.reactive_mode = False
        self.reactive_mode_start = None
        self.reactive_mode_time = 5.0  # seconds

        self.odom_listener = self.create_subscription(Odometry, 'odom', self.odom_handler, 10)
        self.laser_listener = self.create_subscription(LaserScan, 'scan', self.laser_handler, 10)
        self.goal_timer = self.create_timer(1.0, self._dispatch_goal_once)

    def _build_target_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 2.0
        pose.pose.position.y = 2.0
        pose.pose.orientation.w = 1.0
        return pose

    def _dispatch_goal_once(self):
        if not self.goal_dispatched and self.nav_action.wait_for_server(timeout_sec=2.0):
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = self.target_pose
            self.nav_action.send_goal_async(nav_goal)
            self.goal_dispatched = True
            self.get_logger().info('HybridNavigator: Goal sent to Nav2 action server.')
            self.goal_timer.cancel()

    def odom_handler(self, msg):
        if self.has_arrived:
            return  # Stop updating metrics after arrival

        position = msg.pose.pose.position
        # Track total distance
        if self.prev_odom_pos is not None:
            dx = position.x - self.prev_odom_pos.x
            dy = position.y - self.prev_odom_pos.y
            self.total_distance += math.hypot(dx, dy)
        self.prev_odom_pos = position

        # Check for goal reached
        dist_to_target = math.hypot(position.x - self.target_pose.pose.position.x, position.y - self.target_pose.pose.position.y)
        if dist_to_target < self.reach_tolerance:
            self.has_arrived = True
            self.is_successful = 1
            self.get_logger().info('Target reached!')
            self.display_metrics()
            self.terminate()
            return

        # Timeout: print metrics and stop
        elapsed = time.time() - self.run_start
        if elapsed > self.max_duration:
            self.display_metrics()
            self.terminate()

        # Hybrid: monitor progress
        if not self.reactive_mode and self.goal_dispatched and not self.has_arrived:
            if self.last_move_pos is None:
                self.last_move_pos = (position.x, position.y)
                self.last_move_time = time.time()
            else:
                progress = math.hypot(position.x - self.last_move_pos[0], position.y - self.last_move_pos[1])
                if progress > self.min_progress:
                    self.last_move_time = time.time()
                    self.last_move_pos = (position.x, position.y)
                elif time.time() - self.last_move_time > self.stuck_limit:
                    self.get_logger().warn('No progress! Switching to reactive navigation.')
                    self.abort_goal()
                    self.reactive_mode = True
                    self.reactive_mode_start = time.time()

        # After reactive mode, replan
        if self.reactive_mode and self.reactive_mode_start is not None:
            if (time.time() - self.reactive_mode_start > self.reactive_mode_time):
                self.get_logger().info('Reactive navigation complete, replanning...')
                self.reactive_mode = False
                self.goal_dispatched = False
                self.goal_timer = self.create_timer(1.0, self._dispatch_goal_once)

    def abort_goal(self):
        if self.active_goal_handle is not None:
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Goal aborted.'))

    def laser_handler(self, msg):
        if self.has_arrived:
            return  # Stop updating metrics after arrival

        filtered_ranges = [r if not (math.isinf(r) or math.isnan(r)) else msg.range_max for r in msg.ranges]
        if min(filtered_ranges) < 0.22:
            self.collision_warnings += 1

        if self.reactive_mode:
            # --- Purely reactive wall-following ---
            safe_dist = 0.5
            danger_dist = 0.28
            n = len(filtered_ranges)
            front = list(range(0, 15)) + list(range(n - 15, n))
            left = list(range(int(n/4), int(n/4 + 30)))
            right = list(range(int(3*n/4 - 30), int(3*n/4)))

            front_val = min([filtered_ranges[i] for i in front])
            left_val = min([filtered_ranges[i] for i in left])
            right_val = min([filtered_ranges[i] for i in right])

            twist = Twist()
            if front_val < danger_dist:
                twist.linear.x = 0.0
                twist.angular.z = 0.7  # sharp left
            elif front_val < safe_dist:
                twist.linear.x = 0.08
                if left_val < safe_dist:
                    twist.angular.z = -0.6  # steer right
                elif left_val > safe_dist + 0.15:
                    twist.angular.z = 0.6   # steer left
                else:
                    twist.angular.z = 0.2   # gentle left
            else:
                twist.linear.x = 0.18
                twist.angular.z = 0.0

            self.velocity_publisher.publish(twist)

    def display_metrics(self):
        elapsed = time.time() - self.run_start
        print("\n=== HybridNavigator Metrics ===")
        print(f"Success: {self.is_successful}")
        print(f"Elapsed Time (sec): {min(elapsed, self.max_duration):.2f}")
        print(f"Collision Warnings: {self.collision_warnings}")
        print(f"Distance Traveled (m): {self.total_distance:.2f}")
        print("==============================\n")

    def terminate(self):
        self.velocity_publisher.publish(Twist())  # stop the robot
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    nav_node = HybridNavigator()
    rclpy.spin(nav_node)
    nav_node.destroy_node()

if __name__ == '__main__':
    main()