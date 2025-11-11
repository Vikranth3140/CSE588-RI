# Hybrid Planning Assignment — ROSPlan + MoveIt

## Overview
Minimal setup for interleaved symbolic and geometric planning using PDDL⁺, ROSPlan, and MoveIt.

## Usage
1. Start MoveIt:
   ```
   roslaunch rosplan_moveit moveit.launch
   ```
2. Start ROSPlan + monitors:
   ```
   roslaunch rosplan_moveit rosplan.launch
   ```
3. Observe feasibility and replanning logs.

## Modify
- `pddl/hybrid_problem.pddl`: edit objects and goals
- `monitor_motion.py`: integrate MoveIt collision check
- `replanner.py`: adjust replan trigger

Deliverables: modified PDDL files + 1-page REPORT.md summarizing replans, metrics, and insights.
