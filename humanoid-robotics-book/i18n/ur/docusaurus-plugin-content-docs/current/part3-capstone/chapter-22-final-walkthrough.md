# Chapter 22: Final Simulation Walkthrough

## Learning Objectives

By the end of this chapter, you will:

1. See a complete end-to-end demonstration of an autonomous humanoid system
2. Understand how all components work together in real-time
3. Learn how to observe and debug system behavior during execution
4. Explore common failure modes and recovery strategies
5. Understand performance metrics and system evaluation

## Prerequisites

- Chapters 20-21 (Architecture and Integration)
- All four modules completed

## Introduction

This chapter provides a complete walkthrough of the humanoid robot system executing the task: **"Bring me a glass of water from the kitchen"**

We'll observe:
- Real-time data flowing through the system
- How modules collaborate
- Failure detection and recovery
- Performance metrics

## Scenario Setup

**Environment**: Simulated home (Gazebo + Isaac Sim)
- Kitchen with glass on counter
- Living room with user
- Obstacles (furniture, walls)

**Robot**: Humanoid with mobile base, 7-DOF arms, RGB-D camera, LiDAR

**System State** (t=0):
- Location: Living room
- Battery: 85%
- All systems operational

## Step-by-Step Walkthrough

### T+0s: User Command

```
User: "Bring me a glass of water from the kitchen"
```

**Layer 6 (Interface)**: Whisper Node
```
[whisper_node] Audio captured: 3.2s
[whisper_node] Transcribing...
[whisper_node] Transcription (latency: 287ms): "bring me a glass of water from the kitchen"
[whisper_node] Publishing to /speech/transcription
```

### T+0.3s: LLM Planning

**Layer 5 (Executive)**: LLM Planner
```
[llm_planner] Received command: "bring me a glass of water from the kitchen"
[llm_planner] Calling GPT-4 API...
[llm_planner] Response (latency: 1.8s):
{
  "plan": [
    {"action": "navigate", "params": {"location": "kitchen"}},
    {"action": "detect_objects", "params": {"query": "glass"}},
    {"action": "grasp", "params": {"object_id": "auto"}},
    {"action": "navigate", "params": {"location": "sink"}},
    {"action": "fill_container", "params": {"duration": 3}},
    {"action": "navigate", "params": {"location": "user"}},
    {"action": "handover", "params": {}}
  ]
}
[llm_planner] Publishing to /robot/action_plan
```

### T+2.1s: Execute Action 1 - Navigate to Kitchen

**Layer 5 (Executive)**: Task Executor
```
[task_executor] Starting action 1/7: navigate(kitchen)
[task_executor] Calling /navigate_to_pose action
```

**Layer 4 (Deliberative)**: Nav2 Planner
```
[nav2_planner] Received goal: kitchen (x: 5.0, y: 2.0)
[nav2_planner] Current pose: (x: 0.5, y: 0.3)
[nav2_planner] Computing global path... (A* search)
[nav2_planner] Path found: 15 waypoints, distance: 6.2m
[nav2_planner] Publishing velocity commands to /cmd_vel
```

**Layer 3 (Perceptive)**: SLAM & Obstacle Detection
```
[slam_node] LiDAR scan received
[slam_node] Updating pose estimate: (x: 0.52, y: 0.31, yaw: 0.05)
[obstacle_detector] Obstacle detected at (x: 2.5, y: 1.0) - chair
[nav2_controller] Adjusting path to avoid obstacle
```

**Layer 2 (Reactive)**: Motor Controllers
```
[base_controller] Target velocity: linear=0.3 m/s, angular=0.1 rad/s
[base_controller] Sending motor commands: left_wheel=0.25, right_wheel=0.35
```

**Layer 1 (Hardware)**: Simulated Robot
```
[gazebo] Wheel velocities applied
[gazebo] Robot moving... current pose (x: 1.2, y: 0.8)
```

*[12 seconds elapse...]*

```
[nav2_controller] Goal reached: kitchen
[task_executor] Action 1 complete (duration: 12.3s)
```

### T+14.4s: Execute Action 2 - Detect Glass

**Layer 5 (Executive)**:
```
[task_executor] Starting action 2/7: detect_objects(glass)
[task_executor] Calling /detect_objects service
```

**Layer 3 (Perceptive)**: Isaac ROS Object Detector
```
[object_detector] Camera image received: 640x480 RGB-D
[object_detector] Running inference (DetectNet V2)...
[object_detector] Detections (latency: 45ms):
  - glass (confidence: 0.92, bbox: [320, 180, 120, 200], pose: [5.2, 2.3, 0.9])
  - plate (confidence: 0.88, bbox: [150, 220, 100, 80], pose: [5.0, 2.5, 0.9])
[object_detector] Returning detection: glass_0
```

**Layer 5 (Executive)**:
```
[task_executor] Object detected: glass_0 at pose (5.2, 2.3, 0.9)
[task_executor] Action 2 complete (duration: 0.15s)
```

### T+14.6s: Execute Action 3 - Grasp Glass

**Layer 5 (Executive)**:
```
[task_executor] Starting action 3/7: grasp(glass_0)
[task_executor] Calling /move_arm action
```

**Layer 4 (Deliberative)**: MoveIt2 Motion Planner
```
[moveit2_planner] Target pose: (5.2, 2.3, 0.9)
[moveit2_planner] Planning grasp approach...
[grasp_planner] Computing top-down grasp
[moveit2_planner] IK solution found: [0.2, -0.5, 0.3, 1.2, 0.0, 0.8, 0.0] (joint angles)
[moveit2_planner] Planning collision-free trajectory...
[moveit2_planner] Trajectory generated: 50 waypoints, duration: 3.2s
[moveit2_planner] Executing trajectory...
```

**Layer 2 (Reactive)**: Arm Controllers
```
[arm_controller] Following trajectory...
[arm_controller] Joint 1: target=0.2, current=0.19
[arm_controller] Joint 2: target=-0.5, current=-0.48
... [continued for 7 joints]
```

**Layer 1 (Hardware)**: Force/Torque Sensor
```
[ft_sensor] Contact detected (force: 2.3N)
[gripper_controller] Closing gripper...
[ft_sensor] Grasp force: 8.5N (stable)
```

```
[task_executor] Action 3 complete (duration: 3.8s)
```

### T+18.4s: Execute Actions 4-6

*Actions 4-6 proceed similarly:*
- **Navigate to sink** (8.5s)
- **Fill container** (3.0s) - simulated water flow
- **Navigate to user** (11.2s)

### T+41.1s: Execute Action 7 - Handover

**Layer 5 (Executive)**:
```
[task_executor] Starting action 7/7: handover()
[task_executor] Detecting human pose...
```

**Layer 3 (Perceptive)**: Human Detection
```
[human_detector] Human detected at (0.8, 0.4, 1.0)
[hand_tracker] Hand pose: (0.85, 0.45, 1.1)
```

**Layer 4 (Deliberative)**:
```
[moveit2_planner] Planning handover trajectory to (0.85, 0.45, 1.1)
[moveit2_planner] Trajectory generated, executing...
```

**Layer 1 (Hardware)**:
```
[ft_sensor] Human hand contact detected (force decrease: -8.5N → 0N)
[gripper_controller] Opening gripper...
```

**Layer 5 (Executive)**:
```
[task_executor] Action 7 complete (duration: 2.1s)
[task_executor] All actions complete. Total duration: 41.2s
```

**Layer 6 (Interface)**: Text-to-Speech
```
[tts_node] Speaking: "Here is your water"
```

## System Metrics

**Performance Summary**:
```
Task: Bring glass of water
Total duration: 41.2 seconds
Success: ✓

Breakdown:
- Planning (LLM): 1.8s (4.4%)
- Navigation: 31.8s (77.2%)
- Perception: 0.2s (0.5%)
- Manipulation: 7.4s (18.0%)

Resources:
- CPU usage (avg): 42%
- GPU usage (avg): 68% (perception spikes)
- Memory: 4.2 GB
- Network: 15 MB (LLM API calls)
- Battery consumed: 3% (82% remaining)
```

## Failure Scenario & Recovery

**Alternative Timeline** (with failure):

### T+14.4s: Grasp Attempt Fails

```
[grasp_planner] Attempting grasp...
[ft_sensor] No contact detected (expected: 8.5N, actual: 0.2N)
[task_executor] ERROR: Grasp failed
```

**Recovery Strategy**:
```
[task_executor] Triggering failure recovery
[failure_recovery] Analyzing failure: grasp_failed
[failure_recovery] Retry strategy: adjust_grasp_pose
[grasp_planner] Computing alternative grasp (side grasp)
[moveit2_planner] Replanning trajectory...
[moveit2_planner] New trajectory executing...
[ft_sensor] Contact detected! Grasp successful (force: 9.1N)
[task_executor] Recovery successful, continuing plan
```

**Total duration with recovery**: 44.8s (3.6s overhead)

## Observing the System in RViz

**RViz panels to watch**:
1. **Robot Model**: See arm motion, gripper state
2. **Camera Feed**: Object detections with bounding boxes
3. **TF Tree**: Coordinate frame transformations
4. **Planned Path**: Nav2 global planner output
5. **Point Cloud**: Depth sensor data
6. **Diagnostics**: Battery, CPU, errors

**Key visualizations**:
- Red path: Global plan from Nav2
- Green path: Local trajectory (obstacle avoidance)
- Blue boxes: Detected objects
- Yellow sphere: Current navigation goal

## Common Debugging Patterns

### Issue: Robot doesn't move after command

**Diagnosis**:
```bash
# Check if command received
ros2 topic echo /speech/transcription

# Check if plan generated
ros2 topic echo /robot/action_plan

# Check navigation goal
ros2 action send_goal /navigate_to_pose ...
```

**Common causes**:
- Microphone not configured
- OpenAI API key missing
- Nav2 not launched

### Issue: Object detection fails

**Diagnosis**:
```bash
# Check camera feed
ros2 topic echo /camera/image_raw

# Test detector directly
ros2 service call /detect_objects "{query: 'glass'}"

# Check model path
ros2 param get /object_detector model_path
```

## Summary

This walkthrough demonstrated:
1. **Complete system integration**: Voice → Planning → Navigation → Manipulation → Handover
2. **Layer collaboration**: Each layer contributes specialized functionality
3. **Real-time performance**: 41.2s end-to-end execution
4. **Failure recovery**: Automatic retry on grasp failure
5. **Observability**: RViz visualization and ROS topic inspection

**Key insight**: The complexity is hidden behind clean interfaces. Each layer does one job well, and integration patterns glue them together seamlessly.

## Connections

- **Chapter 19**: Implementation details
- **Chapter 20**: Architectural view
- **Chapter 21**: Integration patterns used here
- **Chapters 23-24**: Where to go from here
