#!/usr/bin/env python3
"""
Experiment Runner for Hybrid Planning Evaluation
Automates testing of different planning modes and collects metrics.
"""
import rospy
import json
import time
from datetime import datetime
from std_msgs.msg import String
import subprocess
import os


class ExperimentRunner:
    def __init__(self):
        rospy.init_node("experiment_runner", anonymous=True)
        
        self.results = []
        self.current_experiment = None
        
        # Subscribers for metrics
        rospy.Subscriber('/motion_metrics', String, self.metrics_callback)
        rospy.Subscriber('/replanning_events', String, self.replan_callback)
        
        self.replan_events = []
        self.metrics_history = []
    
    def metrics_callback(self, msg):
        """Collect metrics during experiment"""
        try:
            data = json.loads(msg.data)
            self.metrics_history.append(data)
        except:
            pass
    
    def replan_callback(self, msg):
        """Record replanning events"""
        try:
            data = json.loads(msg.data)
            self.replan_events.append(data)
            print(f"  [REPLAN #{len(self.replan_events)}] {data['reason']}")
        except:
            pass
    
    def run_experiment(self, mode, duration=60):
        """
        Run a single experiment.
        mode: 'symbolic', 'sequential', or 'interleaved'
        """
        print("=" * 70)
        print(f"EXPERIMENT: {mode.upper()} MODE")
        print(f"Duration: {duration}s")
        print("=" * 70)
        
        # Reset tracking
        self.replan_events = []
        self.metrics_history = []
        start_time = time.time()
        
        # Configure system based on mode
        self.configure_mode(mode)
        
        # Trigger initial planning
        print("\nStarting plan execution...")
        self.trigger_planning()
        
        # Monitor execution
        rospy.sleep(1.0)
        print("\nMonitoring execution...\n")
        
        rate = rospy.Rate(1)  # 1 Hz
        elapsed = 0
        
        while elapsed < duration and not rospy.is_shutdown():
            elapsed = time.time() - start_time
            
            # Print progress
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                print(f"[{int(elapsed)}s] Replans: {len(self.replan_events)}, "
                      f"Metrics: {len(self.metrics_history)}")
            
            rate.sleep()
        
        # Collect results
        results = self.analyze_results(mode, elapsed)
        self.results.append(results)
        
        print("\n" + "=" * 70)
        print("EXPERIMENT COMPLETE")
        print("=" * 70)
        self.print_results(results)
        print()
        
        return results
    
    def configure_mode(self, mode):
        """Configure system parameters for experiment mode"""
        if mode == 'symbolic':
            # Disable motion validation
            rospy.set_param('/motion_monitor/use_moveit', False)
            rospy.set_param('/motion_monitor/check_rate', 0.0)
            rospy.set_param('/replanner/check_rate', 0.0)
            print("✓ Configured: Pure symbolic planning (no validation)")
            
        elif mode == 'sequential':
            # Enable validation but limited replanning
            rospy.set_param('/motion_monitor/use_moveit', True)
            rospy.set_param('/motion_monitor/check_rate', 0.1)
            rospy.set_param('/replanner/failure_threshold', 5)
            rospy.set_param('/replanner/replan_cooldown', 30.0)
            print("✓ Configured: Sequential validation (limited replanning)")
            
        elif mode == 'interleaved':
            # Full adaptive replanning
            rospy.set_param('/motion_monitor/use_moveit', True)
            rospy.set_param('/motion_monitor/check_rate', 0.5)
            rospy.set_param('/replanner/failure_threshold', 2)
            rospy.set_param('/replanner/collision_risk_threshold', 0.7)
            rospy.set_param('/replanner/battery_threshold', 0.15)
            rospy.set_param('/replanner/replan_cooldown', 5.0)
            print("✓ Configured: Interleaved planning (full adaptive replanning)")
    
    def trigger_planning(self):
        """Trigger ROSPlan to generate and execute plan"""
        try:
            # Call planning service
            subprocess.call([
                "rosservice", "call",
                "/rosplan_planner_interface/planning_server"
            ], timeout=10)
            print("✓ Planning service called")
        except Exception as e:
            print(f"✗ Failed to trigger planning: {e}")
    
    def analyze_results(self, mode, duration):
        """Analyze collected data and compute metrics"""
        # Count replans
        num_replans = len(self.replan_events)
        
        # Analyze replan reasons
        replan_reasons = {}
        for event in self.replan_events:
            reason = event.get('reason', 'unknown').split(':')[0]
            replan_reasons[reason] = replan_reasons.get(reason, 0) + 1
        
        # Compute metrics from history
        if self.metrics_history:
            feasibility_checks = len(self.metrics_history)
            successful_checks = sum(1 for m in self.metrics_history 
                                   if m.get('feasible', False))
            success_rate = successful_checks / max(1, feasibility_checks)
            
            # Get final metrics
            last_metric = self.metrics_history[-1]
            infeasibility_count = last_metric.get('infeasibility_count', 0)
            
            # Collision attempts (infeasibilities in symbolic mode)
            collision_attempts = infeasibility_count if mode == 'symbolic' else 0
        else:
            feasibility_checks = 0
            success_rate = 0.0
            infeasibility_count = 0
            collision_attempts = 0
        
        # Estimate battery consumption (simplified)
        battery_consumed = 0.5 - max(0.1, 0.5 - num_replans * 0.05)
        
        # Determine success
        if mode == 'symbolic':
            success = (collision_attempts == 0)
        else:
            success = (success_rate > 0.5)
        
        return {
            'mode': mode,
            'duration': duration,
            'success': success,
            'num_replans': num_replans,
            'replan_reasons': replan_reasons,
            'feasibility_checks': feasibility_checks,
            'success_rate': success_rate,
            'infeasibility_count': infeasibility_count,
            'collision_attempts': collision_attempts,
            'battery_consumed': battery_consumed,
            'timestamp': datetime.now().isoformat()
        }
    
    def print_results(self, results):
        """Print formatted results"""
        print(f"\nMode: {results['mode'].upper()}")
        print(f"Success: {'✓ YES' if results['success'] else '✗ NO'}")
        print(f"Duration: {results['duration']:.1f}s")
        print(f"Replans: {results['num_replans']}")
        
        if results['replan_reasons']:
            print("Replan Reasons:")
            for reason, count in results['replan_reasons'].items():
                print(f"  - {reason}: {count}")
        
        print(f"Feasibility Checks: {results['feasibility_checks']}")
        print(f"Success Rate: {results['success_rate']:.1%}")
        print(f"Infeasibilities: {results['infeasibility_count']}")
        print(f"Collision Attempts: {results['collision_attempts']}")
        print(f"Battery Consumed: {results['battery_consumed']:.2f}")
    
    def run_full_suite(self):
        """Run complete experiment suite"""
        print("\n" + "=" * 70)
        print("HYBRID PLANNING EXPERIMENT SUITE")
        print("=" * 70 + "\n")
        
        # Run experiments
        modes = ['symbolic', 'sequential', 'interleaved']
        durations = [30, 45, 60]  # Different durations for each mode
        
        for mode, duration in zip(modes, durations):
            self.run_experiment(mode, duration)
            
            # Pause between experiments
            if mode != modes[-1]:
                print("\nPausing 10s before next experiment...\n")
                rospy.sleep(10.0)
        
        # Generate comparison report
        self.generate_comparison_report()
    
    def generate_comparison_report(self):
        """Generate markdown report comparing all modes"""
        print("\n" + "=" * 70)
        print("COMPARISON REPORT")
        print("=" * 70 + "\n")
        
        # Create table
        print("| Metric | Symbolic | Sequential | Interleaved |")
        print("|--------|----------|------------|-------------|")
        
        metrics_to_compare = [
            ('success', 'Success', lambda x: '✓' if x else '✗'),
            ('num_replans', 'Replans', lambda x: str(x)),
            ('success_rate', 'Success Rate', lambda x: f"{x:.1%}"),
            ('collision_attempts', 'Collision Attempts', lambda x: str(x)),
            ('battery_consumed', 'Battery Used', lambda x: f"{x:.2f}"),
        ]
        
        for key, label, formatter in metrics_to_compare:
            row = f"| {label} |"
            for result in self.results:
                value = result.get(key, 'N/A')
                formatted = formatter(value) if value != 'N/A' else 'N/A'
                row += f" {formatted} |"
            print(row)
        
        print("\n" + "=" * 70)
        
        # Save to file
        self.save_results_to_file()
    
    def save_results_to_file(self):
        """Save results to JSON file"""
        filename = f"experiment_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        data = {
            'timestamp': datetime.now().isoformat(),
            'results': self.results,
            'summary': {
                'total_experiments': len(self.results),
                'successful_experiments': sum(1 for r in self.results if r['success']),
            }
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"\n✓ Results saved to: {filename}")
        except Exception as e:
            print(f"\n✗ Failed to save results: {e}")


def main():
    """Main entry point"""
    print("\n" + "=" * 70)
    print("HYBRID PLANNING EXPERIMENT RUNNER")
    print("=" * 70)
    
    runner = ExperimentRunner()
    
    print("\nOptions:")
    print("1. Run single experiment")
    print("2. Run full suite (all modes)")
    print("3. Quick test (interleaved only)")
    
    try:
        choice = input("\nSelect option (1-3): ").strip()
        
        if choice == '1':
            mode = input("Mode (symbolic/sequential/interleaved): ").strip().lower()
            duration = int(input("Duration (seconds): ").strip())
            runner.run_experiment(mode, duration)
            
        elif choice == '2':
            runner.run_full_suite()
            
        elif choice == '3':
            print("\nRunning quick test with interleaved mode...")
            runner.run_experiment('interleaved', 30)
        
        else:
            print("Invalid option. Running quick test...")
            runner.run_experiment('interleaved', 30)
    
    except KeyboardInterrupt:
        print("\n\nExperiment interrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
    
    print("\nExperiment runner shutting down.")


if __name__ == "__main__":
    main()
