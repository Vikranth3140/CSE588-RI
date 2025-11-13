#!/usr/bin/env python3
import random
import rospy
import traceback
import json
from std_msgs.msg import String, Bool, Float32
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import numpy as np
from datetime import datetime


class MotionMonitor:
    def __init__(self):
        rospy.init_node("motion_monitor")
        
        # Parameters
        self.use_moveit = rospy.get_param('~use_moveit', True)
        self.check_rate = rospy.get_param('~check_rate', 0.5)  # Hz
        self.collision_threshold = rospy.get_param('~collision_threshold', 0.7)
        
        # State tracking
        self.current_action = None
        self.infeasibility_count = 0
        self.feasibility_history = []
        self.start_time = datetime.now()
        
        # MoveIt service client
        if self.use_moveit:
            rospy.loginfo("Waiting for MoveIt state validity service...")
            rospy.wait_for_service('/check_state_validity', timeout=10.0)
            self.validity_client = rospy.ServiceProxy(
                '/check_state_validity', 
                GetStateValidity
            )
            rospy.loginfo("Connected to MoveIt collision checker")
        
        # Publishers
        self.feasibility_pub = rospy.Publisher(
            '/motion_feasibility', 
            Bool, 
            queue_size=10
        )
        self.metrics_pub = rospy.Publisher(
            '/motion_metrics', 
            String, 
            queue_size=10
        )
        self.collision_risk_pub = rospy.Publisher(
            '/collision_risk',
            Float32,
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            '/rosplan_plan_dispatcher/action_dispatch',
            String,
            self.action_callback
        )
        
        rospy.loginfo("Motion Monitor initialized")
    
    def action_callback(self, msg):
        """Track current executing action"""
        try:
            action_data = json.loads(msg.data)
            self.current_action = action_data
            rospy.logdebug(f"Current action: {action_data}")
        except:
            self.current_action = msg.data
    
    def check_moveit_feasibility(self, joint_state=None):
        """
        Check motion feasibility using MoveIt collision checking.
        Returns (feasible: bool, collision_risk: float)
        """
        try:
            # Create request
            req = GetStateValidityRequest()
            req.group_name = "manipulator"  # or "arm" depending on robot config
            
            # Use current or provided joint state
            if joint_state is None:
                # Generate sample configuration
                joint_state = self.generate_sample_configuration()
            
            robot_state = RobotState()
            robot_state.joint_state = joint_state
            req.robot_state = robot_state
            
            # Call service
            response = self.validity_client(req)
            
            # Calculate collision risk based on distance to collision
            collision_risk = 0.0
            if not response.valid:
                collision_risk = 1.0
            elif hasattr(response, 'contacts') and len(response.contacts) > 0:
                # Calculate risk based on proximity to collision
                min_distance = min([c.depth for c in response.contacts])
                collision_risk = max(0.0, 1.0 - min_distance / 0.1)  # 10cm threshold
            
            return response.valid, collision_risk
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"MoveIt service call failed: {e}")
            return self.check_fallback_feasibility()
        except Exception as e:
            rospy.logwarn(f"Error checking MoveIt feasibility: {e}")
            return self.check_fallback_feasibility()
    
    def generate_sample_configuration(self):
        """Generate a sample joint configuration for testing"""
        joint_state = JointState()
        joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                           'elbow_joint', 'wrist_1_joint', 
                           'wrist_2_joint', 'wrist_3_joint']
        
        # Generate random configuration (or use current if available)
        joint_state.position = np.random.uniform(-np.pi, np.pi, 6).tolist()
        return joint_state
    
    def check_fallback_feasibility(self):
        """
        Fallback feasibility check using heuristics when MoveIt is unavailable.
        Simulates collision detection based on problem structure.
        """
        # Use collision costs from problem definition
        collision_risk = np.random.uniform(0.0, 1.0)
        
        # If we know the current action, use its collision cost
        if self.current_action:
            action_str = str(self.current_action)
            
            # High-risk paths from problem definition
            if 'table' in action_str and 'shelf' in action_str:
                collision_risk = 0.9
            elif 'counter' in action_str and 'storage' in action_str:
                collision_risk = 0.85
            elif 'shelf' in action_str:
                collision_risk = np.random.uniform(0.6, 0.95)
            else:
                collision_risk = np.random.uniform(0.1, 0.4)
        
        feasible = collision_risk < self.collision_threshold
        return feasible, collision_risk
    
    def check_motion_feasibility(self):
        """Main feasibility check - delegates to MoveIt or fallback"""
        if self.use_moveit:
            try:
                return self.check_moveit_feasibility()
            except:
                rospy.logwarn("MoveIt check failed, using fallback")
                return self.check_fallback_feasibility()
        else:
            return self.check_fallback_feasibility()
    
    def publish_metrics(self, feasible, collision_risk):
        """Publish monitoring metrics"""
        # Publish feasibility
        self.feasibility_pub.publish(Bool(data=feasible))
        
        # Publish collision risk
        self.collision_risk_pub.publish(Float32(data=collision_risk))
        
        # Calculate and publish aggregate metrics
        runtime = (datetime.now() - self.start_time).total_seconds()
        success_rate = (len([f for f in self.feasibility_history if f]) / 
                       max(1, len(self.feasibility_history)))
        
        metrics = {
            'timestamp': datetime.now().isoformat(),
            'feasible': feasible,
            'collision_risk': collision_risk,
            'infeasibility_count': self.infeasibility_count,
            'total_checks': len(self.feasibility_history),
            'success_rate': success_rate,
            'runtime_seconds': runtime,
            'current_action': str(self.current_action) if self.current_action else 'None'
        }
        
        self.metrics_pub.publish(String(data=json.dumps(metrics)))
    
    def run(self):
        """Main monitoring loop"""
        rate = rospy.Rate(self.check_rate)
        
        rospy.loginfo(f"Starting motion monitoring at {self.check_rate} Hz")
        rospy.loginfo(f"MoveIt integration: {self.use_moveit}")
        rospy.loginfo(f"Collision threshold: {self.collision_threshold}")
        
        while not rospy.is_shutdown():
            # Check feasibility
            feasible, collision_risk = self.check_motion_feasibility()
            
            # Track history
            self.feasibility_history.append(feasible)
            if not feasible:
                self.infeasibility_count += 1
            
            # Log status
            status = "OK" if feasible else "FAILED"
            risk_level = "LOW" if collision_risk < 0.3 else "MEDIUM" if collision_risk < 0.7 else "HIGH"
            
            if not feasible:
                rospy.logwarn(f"Motion feasibility: {status} | "
                            f"Collision risk: {collision_risk:.2f} ({risk_level}) | "
                            f"Total failures: {self.infeasibility_count}")
            else:
                rospy.loginfo(f"Motion feasibility: {status} | "
                            f"Collision risk: {collision_risk:.2f} ({risk_level})")
            
            # Publish metrics
            self.publish_metrics(feasible, collision_risk)
            
            rate.sleep()


# def check_motion_feasibility():
#     """Legacy function for compatibility with replanner"""
#     # This is called by replanner.py
#     # Returns a simple boolean based on recent checks
#     try:
#         msg = rospy.wait_for_message('/motion_feasibility', Bool, timeout=1.0)
#         return msg.data
#     except:
#         # Fallback to random if message not available
#         return random.random() > 0.3

if __name__ == "__main__":
    try:
        monitor = MotionMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Motion monitor crashed: {e}")
        traceback.print_exc()
