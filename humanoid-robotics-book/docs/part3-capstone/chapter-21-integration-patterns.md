# Chapter 21: Integration Patterns and Best Practices

## Learning Objectives

By the end of this chapter, you will:

1. Understand common integration patterns for connecting robotics modules
2. Learn best practices for ROS 2 system integration
3. Explore error handling and fault tolerance strategies
4. Understand testing and validation approaches for integrated systems
5. Learn deployment and DevOps patterns for robotics

## Prerequisites

- Chapter 20 (Complete Architecture Overview)
- Understanding of ROS 2 (Module 1)
- Familiarity with all four modules

## Introduction

Integration is where theory meets practice. This chapter provides battle-tested patterns for connecting modules into reliable, maintainable systems.

## Core Integration Patterns

### Pattern 1: Service-Action Coordination

**Problem**: How do modules coordinate synchronous (wait for result) vs. asynchronous (fire-and-forget) operations?

**Solution**: Use ROS 2 services for queries, actions for long-running tasks

```python
# Example: Coordinate perception (service) with navigation (action)
class TaskCoordinator(Node):
    def __init__(self):
        # Service for synchronous queries
        self.detect_client = self.create_client(DetectObjects, '/detect_objects')

        # Action for long-running tasks
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    async def execute_task(self):
        # Synchronous: detect objects
        request = DetectObjects.Request()
        request.query = "glass"
        response = await self.detect_client.call_async(request)

        # Asynchronous: navigate to object
        goal = NavigateToPose.Goal()
        goal.pose = response.detections[0].pose
        nav_future = await self.nav_client.send_goal_async(goal)
        # Continue other work while navigating...
```

### Pattern 2: Transform Management

**Problem**: How do we keep track of coordinate frames across sensors, arms, and base?

**Solution**: Use tf2 transform tree

```python
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

class TfManager(Node):
    def __init__(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

    def get_object_in_base_frame(self, object_pose_camera):
        try:
            # Transform from camera frame to robot base frame
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link', rclpy.time.Time()
            )
            object_pose_base = do_transform_pose(object_pose_camera, transform)
            return object_pose_base
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            return None
```

### Pattern 3: State Machine for Task Execution

**Problem**: How do we manage complex multi-step tasks with conditional branching?

**Solution**: Use behavior trees or state machines

```python
from enum import Enum

class TaskState(Enum):
    IDLE = 0
    NAVIGATING = 1
    DETECTING = 2
    GRASPING = 3
    DELIVERING = 4
    ERROR = 5

class StateMachineExecutor(Node):
    def __init__(self):
        self.state = TaskState.IDLE
        self.timer = self.create_timer(0.1, self.update)  # 10Hz

    def update(self):
        if self.state == TaskState.IDLE:
            # Wait for new task
            pass
        elif self.state == TaskState.NAVIGATING:
            if self.navigation_complete():
                self.state = TaskState.DETECTING
            elif self.navigation_failed():
                self.state = TaskState.ERROR
        elif self.state == TaskState.DETECTING:
            # ... transition logic
            pass
```

### Pattern 4: Graceful Degradation

**Problem**: What if a sensor fails or a model is unavailable?

**Solution**: Fallback mechanisms

```python
class RobustPerception(Node):
    def __init__(self):
        self.primary_detector = IsaacROSDetector()
        self.fallback_detector = SimpleColorDetector()

    def detect_object(self, image):
        try:
            # Try primary (Isaac ROS)
            detections = self.primary_detector.detect(image)
            if len(detections) > 0:
                return detections
        except Exception as e:
            self.get_logger().warn(f'Primary detector failed: {e}')

        # Fallback to simple detector
        try:
            detections = self.fallback_detector.detect(image)
            return detections
        except Exception as e:
            self.get_logger().error(f'All detectors failed: {e}')
            return []
```

## Best Practices

### 1. Namespace Isolation

Use ROS 2 namespaces to organize nodes:
```bash
# Launch with namespace
ros2 run my_package my_node --ros-args -r __ns:=/robot1

# Multi-robot system
/robot1/perception_node
/robot1/navigation_node
/robot2/perception_node
/robot2/navigation_node
```

### 2. Parameter Management

Centralized configuration:
```yaml
# config/robot_params.yaml
/**:
  ros__parameters:
    robot_name: "humanoid_01"
    max_speed: 0.5

/perception_node:
  ros__parameters:
    model_path: "/models/detector.onnx"
    confidence_threshold: 0.7
```

Load in launch file:
```python
Node(
    package='my_package',
    executable='perception_node',
    parameters=[config_file]
)
```

### 3. Lifecycle Management

Use ROS 2 lifecycle nodes for controlled startup/shutdown:
```python
from rclpy.lifecycle import LifecycleNode, LifecycleState

class ManagedNode(LifecycleNode):
    def on_configure(self, state: LifecycleState):
        # Load configurations, initialize resources
        self.load_model()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        # Start processing
        self.create_subscriptions()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState):
        # Stop processing, cleanup
        return TransitionCallbackReturn.SUCCESS
```

### 4. Error Propagation

Structured error handling:
```python
class ErrorAwareExecutor(Node):
    def execute_action(self, action):
        try:
            result = self.action_executor.execute(action)
            return (True, result)
        except HardwareException as e:
            self.get_logger().error(f'Hardware error: {e}')
            self.request_maintenance()
            return (False, str(e))
        except PlanningException as e:
            self.get_logger().warn(f'Planning failed: {e}')
            self.replan()
            return (False, str(e))
        except Exception as e:
            self.get_logger().fatal(f'Unknown error: {e}')
            self.emergency_stop()
            return (False, str(e))
```

## Testing Strategies

### Unit Testing
Test individual nodes in isolation:
```python
import unittest
from my_package.perception_node import PerceptionNode

class TestPerceptionNode(unittest.TestCase):
    def test_object_detection(self):
        node = PerceptionNode()
        image = load_test_image('glass.png')
        detections = node.detect_objects(image)
        self.assertGreater(len(detections), 0)
        self.assertEqual(detections[0].label, 'glass')
```

### Integration Testing
Test module interactions:
```bash
# Launch minimal system
ros2 launch my_package integration_test.launch.py

# Run integration test
ros2 run my_package integration_test
```

### Simulation Testing
Validate in Gazebo before hardware:
```bash
# Launch Gazebo simulation
ros2 launch humanoid_gazebo full_system.launch.py use_sim_time:=true

# Run test scenario
ros2 run test_suite scenario_bring_water.py
```

## Deployment Patterns

### Docker Containerization

```dockerfile
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-moveit \
    python3-pip

# Install Python packages
RUN pip3 install torch transformers whisper

# Copy workspace
COPY ./humanoid_ws /workspace/humanoid_ws
WORKDIR /workspace/humanoid_ws

# Build
RUN . /opt/ros/humble/setup.sh && colcon build

# Entrypoint
CMD ["ros2", "launch", "humanoid_bringup", "full_system.launch.py"]
```

### CI/CD Pipeline

```yaml
# .github/workflows/ci.yml
name: ROS 2 CI

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v2
      - name: Build ROS workspace
        run: |
          source /opt/ros/humble/setup.bash
          colcon build
      - name: Run tests
        run: |
          source install/setup.bash
          colcon test
```

## Summary

Key integration patterns:
1. **Service-Action Coordination**: Right tool for the right job
2. **Transform Management**: tf2 for spatial reasoning
3. **State Machines**: Manage task complexity
4. **Graceful Degradation**: Fallback mechanisms

Best practices:
- Namespace isolation
- Centralized parameters
- Lifecycle management
- Structured error handling

Testing: Unit → Integration → Simulation → Hardware

Deployment: Docker containers + CI/CD automation

## Connections

- **Chapter 19**: Implementation details
- **Chapter 20**: Architectural context
- **Chapter 22**: See patterns in action
