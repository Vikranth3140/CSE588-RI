# Hybrid Planning with MoveIt and ROSPlan

## Overview

This project implements an **interleaved Task and Motion Planning (TAMP)** system that combines:
- **ROSPlan**: Symbolic/discrete planning using PDDL
- **MoveIt**: Motion planning and collision checking
- **Adaptive Replanning**: Real-time monitoring and plan adjustment

The system demonstrates how purely symbolic planning fails when geometric constraints exist, and how hybrid planning with motion validation resolves these failures.

## Key Features

### 1. Enhanced PDDL Domain
- **Geometric predicates**: `collision-free`, `motion-validated`, `grasp-validated`
- **Continuous fluents**: Battery level, collision cost, path clearance
- **Resource modeling**: Battery consumption based on distance/effort
- **New actions**: `validate-motion`, `recharge`

### 2. MoveIt Integration
- Real collision detection via `/check_state_validity` service
- Collision risk calculation from contact depth
- Fallback heuristics when MoveIt unavailable
- Publishes detailed metrics and feasibility status

### 3. Adaptive Replanner
- **Multi-criteria triggering**: Collision risk, battery, consecutive failures
- **Adaptive thresholds**: Different risk tolerance per action type
- **Cooldown mechanism**: Prevents replan thrashing
- **Event logging**: Detailed tracking of all replans with reasons

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   ROSPlan                           │
│  ┌───────────────┐         ┌──────────────────┐    │
│  │ PDDL Planner  │────────>│  Plan Dispatcher │    │
│  │  (Symbolic)   │         │                  │    │
│  └───────────────┘         └──────────────────┘    │
│         ▲                           │               │
│         │                           ▼               │
│         │                  ┌─────────────────┐     │
│         │                  │  Action Exec    │     │
│         │                  └─────────────────┘     │
└─────────┼───────────────────────────┬───────────────┘
          │                           │
          │                           ▼
    ┌─────┴────────┐         ┌──────────────────┐
    │  Replanner   │<────────│ Motion Monitor   │
    │              │         │  (MoveIt Check)  │
    └──────────────┘         └──────────────────┘
          │                           │
          │                           ▼
          │                  ┌──────────────────┐
          │                  │  MoveIt          │
          └─────────────────>│  Collision Check │
                             └──────────────────┘
```

## File Structure

```
A2/Q4/
├── pddl/
│   ├── hybrid_domain.pddl      # Enhanced domain with geometric predicates
│   └── hybrid_problem.pddl     # Problem with collision-prone paths
├── scripts/
│   ├── monitor_motion.py       # Motion validation with MoveIt
│   ├── replanner.py            # Adaptive replanning logic
│   └── run_experiments.py      # Automated experiment runner
├── launch/
│   ├── moveit.launch           # MoveIt configuration
│   └── rosplan.launch          # ROSPlan + monitors
├── REPORT.md                   # Comprehensive experimental report
└── README.md                   # This file
```

## Installation & Setup

### Prerequisites

```bash
# ROS Noetic (or appropriate ROS version)
sudo apt-get install ros-noetic-desktop-full

# ROSPlan
sudo apt-get install ros-noetic-rosplan

# MoveIt
sudo apt-get install ros-noetic-moveit

# UR5 MoveIt config (or your robot)
sudo apt-get install ros-noetic-ur5-moveit-config
```

### Workspace Setup

```bash
# Create workspace if not exists
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone or copy this package
cp -r /path/to/A2/Q4 ~/catkin_ws/src/rosplan_moveit

# Build
cd ~/catkin_ws
catkin_make

# Source
source devel/setup.bash
```

## Usage

### Quick Start

```bash
# Terminal 1: Start MoveIt
roslaunch rosplan_moveit moveit.launch

# Terminal 2: Start ROSPlan + Monitors
roslaunch rosplan_moveit rosplan.launch

# Terminal 3: Monitor output
rostopic echo /replanning_events
```

### Run Experiments

```bash
# Automated experiment runner
cd scripts
chmod +x run_experiments.py
./run_experiments.py

# Manual single experiment
rosrun rosplan_moveit run_experiments.py
# Then select: 1 (single experiment)
# Mode: interleaved
# Duration: 60
```

### Monitor Topics

```bash
# Motion feasibility
rostopic echo /motion_feasibility

# Collision risk
rostopic echo /collision_risk

# Detailed metrics
rostopic echo /motion_metrics

# Replanning events
rostopic echo /replanning_events
```

## Experimental Modes

### 1. Pure Symbolic (Baseline)
- **No motion validation**
- Demonstrates failure when geometric constraints exist
- Use: Understanding why symbolic planning alone fails

### 2. Sequential Validation
- **Validate after planning, before execution**
- Limited replanning (high threshold)
- Use: Comparing batch vs interleaved validation

### 3. Interleaved (Full System)
- **Continuous monitoring during execution**
- Adaptive replanning with multiple criteria
- Use: Full hybrid TAMP demonstration

## Problem Design

### The Challenge

The problem is designed to **guarantee symbolic planning failure**:

```
Locations: table, counter, sink, shelf, storage
Objects: pot, mug, kettle
Goal: Pour pot into mug

Symbolic Graph Says:
  table → shelf → storage → counter (appears optimal)

Geometric Reality:
  table → shelf: COLLISION (narrow passage)
  counter → storage: COLLISION (blocked path)
  
Pure symbolic planner chooses infeasible path!
```

### Key Constraints

1. **Collision-prone paths**: 
   - `table → shelf` has collision risk 0.9
   - `counter → storage` has collision risk 0.85

2. **Limited battery** (0.5 initial):
   - Forces careful path selection
   - Triggers battery-aware replanning

3. **Grasp constraints**:
   - Not all object grasps are validated
   - Tests grasp feasibility checking

## Expected Results

From our experiments (see REPORT.md for details):

| Metric | Pure Symbolic | Sequential | Interleaved |
|--------|--------------|------------|-------------|
| Success Rate | 0% | 80% | 100% |
| Avg Replans | 0 | 1.2 | 2.3 |
| Avg Time (s) | 8.1* | 18.4 | 15.6 |
| Collision Attempts | 1.0 | 0.2 | 0.0 |

*Time until failure

**Key Findings:**
- Pure symbolic planning **always fails** on this problem
- Interleaved system successfully recovers via replanning
- 44% of replans due to collision detection
- 19% of replans due to battery management

## Configuration

### Motion Monitor Parameters

Edit `launch/rosplan.launch`:

```xml
<node pkg="rosplan_moveit" type="monitor_motion.py" name="motion_monitor">
  <param name="use_moveit" value="true"/>
  <param name="check_rate" value="0.5"/>
  <param name="collision_threshold" value="0.7"/>
</node>
```

### Replanner Parameters

```xml
<node pkg="rosplan_moveit" type="replanner.py" name="replanner">
  <param name="failure_threshold" value="2"/>
  <param name="collision_risk_threshold" value="0.7"/>
  <param name="battery_threshold" value="0.15"/>
  <param name="replan_cooldown" value="5.0"/>
</node>
```

## Troubleshooting

### MoveIt Service Not Available

**Solution:**
1. Check MoveIt is running: `rosnode list | grep move_group`
2. Check service: `rosservice list | grep check_state_validity`
3. Use fallback mode: `use_moveit: false`

### No Replanning Occurs

**Solution:**
1. Check thresholds are not too lenient
2. Verify motion monitor is publishing: `rostopic echo /motion_feasibility`
3. Lower `collision_risk_threshold` or `failure_threshold`

### Replan Thrashing

**Solution:**
1. Increase `replan_cooldown` parameter
2. Increase `failure_threshold`
3. Check if problem is actually solvable with geometric constraints

## Deliverables Summary

- ✅ Modified `hybrid_domain.pddl` with geometric predicates
- ✅ Modified `hybrid_problem.pddl` with challenging scenario
- ✅ Enhanced `monitor_motion.py` with MoveIt integration
- ✅ Enhanced `replanner.py` with adaptive logic
- ✅ Comprehensive `REPORT.md` with metrics and analysis
- ✅ Automated experiment runner

**Key Achievement:** Demonstrated that interleaved TAMP is necessary when geometric constraints exist, as pure symbolic planning has no mechanism to detect or recover from geometric infeasibility.