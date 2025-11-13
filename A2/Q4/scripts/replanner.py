#!/usr/bin/env python3
import rospy
import subprocess
import json
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Empty, EmptyRequest
from datetime import datetime
import time


class AdaptiveReplanner:
    def __init__(self):
        rospy.init_node("replanner")
        
        # Parameters
        self.check_rate = rospy.get_param('~check_rate', 0.5)  # Hz
        self.failure_threshold = rospy.get_param('~failure_threshold', 2)
        self.collision_risk_threshold = rospy.get_param('~collision_risk_threshold', 0.7)
        self.battery_threshold = rospy.get_param('~battery_threshold', 0.15)
        self.replan_cooldown = rospy.get_param('~replan_cooldown', 5.0)  # seconds
        
        # State tracking
        self.consecutive_failures = 0
        self.total_replans = 0
        self.replan_events = []
        self.last_replan_time = 0
        self.current_battery = 1.0
        self.current_collision_risk = 0.0
        self.current_action = None
        self.plan_start_time = datetime.now()
        
        # Feasibility history for adaptive thresholding
        self.feasibility_window = []
        self.window_size = 10
        
        # Subscribe to monitoring topics
        rospy.Subscriber('/motion_feasibility', Bool, self.feasibility_callback)
        rospy.Subscriber('/collision_risk', Float32, self.collision_risk_callback)
        rospy.Subscriber('/motion_metrics', String, self.metrics_callback)
        rospy.Subscriber('/battery_level', Float32, self.battery_callback)
        rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch', 
                        String, self.action_callback)
        
        # Publishers
        self.replan_event_pub = rospy.Publisher(
            '/replanning_events',
            String,
            queue_size=10
        )
        
        # Service proxies
        self.setup_service_proxies()
        
        rospy.loginfo("Adaptive Replanner initialized")
        rospy.loginfo(f"Failure threshold: {self.failure_threshold}")
        rospy.loginfo(f"Collision risk threshold: {self.collision_risk_threshold}")
        rospy.loginfo(f"Battery threshold: {self.battery_threshold}")
    
    def setup_service_proxies(self):
        """Setup service proxies for replanning"""
        try:
            rospy.loginfo("Waiting for ROSPlan services...")
            
            # Wait for planning service
            rospy.wait_for_service('/rosplan_planner_interface/planning_server', 
                                  timeout=10.0)
            self.planning_service = rospy.ServiceProxy(
                '/rosplan_planner_interface/planning_server',
                Empty
            )
            
            # Try to connect to problem generation service
            try:
                rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server',
                                      timeout=5.0)
                self.problem_service = rospy.ServiceProxy(
                    '/rosplan_problem_interface/problem_generation_server',
                    Empty
                )
            except:
                rospy.logwarn("Problem generation service not available")
                self.problem_service = None
            
            rospy.loginfo("Connected to ROSPlan services")
            
        except rospy.ROSException as e:
            rospy.logwarn(f"Could not connect to all ROSPlan services: {e}")
            self.planning_service = None
            self.problem_service = None
    
    def feasibility_callback(self, msg):
        """Track feasibility status"""
        self.feasibility_window.append(msg.data)
        if len(self.feasibility_window) > self.window_size:
            self.feasibility_window.pop(0)
        
        if not msg.data:
            self.consecutive_failures += 1
        else:
            self.consecutive_failures = 0
    
    def collision_risk_callback(self, msg):
        """Track collision risk"""
        self.current_collision_risk = msg.data
    
    def battery_callback(self, msg):
        """Track battery level"""
        self.current_battery = msg.data
    
    def action_callback(self, msg):
        """Track current action"""
        try:
            self.current_action = json.loads(msg.data)
        except:
            self.current_action = msg.data
    
    def metrics_callback(self, msg):
        """Process monitoring metrics"""
        try:
            metrics = json.loads(msg.data)
            # Could use metrics for more sophisticated decision making
            rospy.logdebug(f"Metrics: {metrics}")
        except Exception as e:
            rospy.logdebug(f"Error parsing metrics: {e}")
    
    def should_replan(self):
        """
        Determine if replanning should be triggered based on multiple criteria.
        Returns (should_replan: bool, reason: str)
        """
        # Check cooldown period
        if time.time() - self.last_replan_time < self.replan_cooldown:
            return False, "In cooldown period"
        
        # Criterion 1: Consecutive failures
        if self.consecutive_failures >= self.failure_threshold:
            return True, f"Consecutive failures: {self.consecutive_failures}"
        
        # Criterion 2: High collision risk
        if self.current_collision_risk > self.collision_risk_threshold:
            return True, f"High collision risk: {self.current_collision_risk:.2f}"
        
        # Criterion 3: Low battery
        if self.current_battery < self.battery_threshold:
            return True, f"Low battery: {self.current_battery:.2f}"
        
        # Criterion 4: Sustained poor performance
        if len(self.feasibility_window) >= self.window_size:
            success_rate = sum(self.feasibility_window) / len(self.feasibility_window)
            if success_rate < 0.3:
                return True, f"Poor success rate: {success_rate:.2%}"
        
        # Criterion 5: Action-specific heuristics
        if self.current_action:
            action_str = str(self.current_action).lower()
            
            # If attempting risky paths, be more conservative
            if ('shelf' in action_str or 'storage' in action_str):
                if self.current_collision_risk > 0.5:
                    return True, f"Risky path with collision risk: {self.current_collision_risk:.2f}"
        
        return False, "All criteria passed"
    
    def replan(self, reason):
        """
        Trigger replanning and record event.
        """
        rospy.logwarn("=" * 60)
        rospy.logwarn(f"TRIGGERING REPLAN #{self.total_replans + 1}")
        rospy.logwarn(f"Reason: {reason}")
        rospy.logwarn(f"Current collision risk: {self.current_collision_risk:.2f}")
        rospy.logwarn(f"Current battery: {self.current_battery:.2f}")
        rospy.logwarn(f"Consecutive failures: {self.consecutive_failures}")
        rospy.logwarn("=" * 60)
        
        # Record event
        event = {
            'replan_number': self.total_replans + 1,
            'timestamp': datetime.now().isoformat(),
            'reason': reason,
            'collision_risk': self.current_collision_risk,
            'battery_level': self.current_battery,
            'consecutive_failures': self.consecutive_failures,
            'current_action': str(self.current_action) if self.current_action else 'None',
            'runtime_seconds': (datetime.now() - self.plan_start_time).total_seconds()
        }
        
        self.replan_events.append(event)
        
        # Publish event
        self.replan_event_pub.publish(String(data=json.dumps(event)))
        
        # Call replanning services
        success = False
        try:
            # First regenerate problem if service available
            if self.problem_service:
                rospy.loginfo("Regenerating problem...")
                self.problem_service(EmptyRequest())
                rospy.sleep(0.5)
            
            # Then trigger planning
            if self.planning_service:
                rospy.loginfo("Calling planner...")
                self.planning_service(EmptyRequest())
                success = True
                rospy.loginfo("Replan request sent successfully")
            else:
                # Fallback to rosservice command
                rospy.loginfo("Using rosservice fallback...")
                result = subprocess.call([
                    "rosservice", "call", 
                    "/rosplan_planner_interface/planning_server"
                ])
                success = (result == 0)
        
        except Exception as e:
            rospy.logerr(f"Replan failed: {e}")
        
        if success:
            self.total_replans += 1
            self.last_replan_time = time.time()
            self.consecutive_failures = 0  # Reset after replan
            rospy.loginfo(f"Total replans so far: {self.total_replans}")
        
        return success
    
    def print_statistics(self):
        """Print replanning statistics"""
        if self.total_replans > 0:
            rospy.loginfo("=" * 60)
            rospy.loginfo("REPLANNING STATISTICS")
            rospy.loginfo(f"Total replans: {self.total_replans}")
            rospy.loginfo(f"Runtime: {(datetime.now() - self.plan_start_time).total_seconds():.1f}s")
            
            # Summarize reasons
            reasons = {}
            for event in self.replan_events:
                reason_key = event['reason'].split(':')[0]
                reasons[reason_key] = reasons.get(reason_key, 0) + 1
            
            rospy.loginfo("Replan reasons:")
            for reason, count in reasons.items():
                rospy.loginfo(f"  - {reason}: {count}")
            
            rospy.loginfo("=" * 60)
    
    def run(self):
        """Main replanning loop"""
        rate = rospy.Rate(self.check_rate)
        
        rospy.loginfo(f"Starting adaptive replanner at {self.check_rate} Hz")
        
        # Print statistics periodically
        last_stats_time = time.time()
        stats_interval = 30.0  # seconds
        
        while not rospy.is_shutdown():
            # Check if replanning needed
            should_replan, reason = self.should_replan()
            
            if should_replan:
                self.replan(reason)
            
            # Print statistics periodically
            if time.time() - last_stats_time > stats_interval:
                self.print_statistics()
                last_stats_time = time.time()
            
            rate.sleep()
        
        # Print final statistics
        self.print_statistics()


# # Legacy function for backward compatibility
# def check_motion_feasibility():
#     """Legacy function that imports from monitor_motion module"""
#     try:
#         from monitor_motion import check_motion_feasibility as cmf
#         return cmf()
#     except:
#         import random
#         return random.random() > 0.3


if __name__ == "__main__":
    try:
        replanner = AdaptiveReplanner()
        replanner.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Replanner crashed: {e}")
        import traceback
        traceback.print_exc()
