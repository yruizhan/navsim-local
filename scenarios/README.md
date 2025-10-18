# Test Scenarios

This directory contains JSON scenario files for testing the navsim_local_debug tool.

## Available Scenarios

### 1. simple_corridor.json
**Description**: Simple corridor with static circular and rectangular obstacles.

**Features**:
- Start: (0, 0, 0)
- Goal: (20, 0, 0)
- 3 static obstacles (2 circles, 1 rectangle)
- Suitable for testing basic path planning algorithms

**Usage**:
```bash
./navsim_local_debug --scenario scenarios/simple_corridor.json --planner JpsPlanner
```

---

### 2. parking_scenario.json
**Description**: Parking scenario with tight space constraints.

**Features**:
- Start: (0, 0, 0)
- Goal: (10, 5, π/2) - 90° rotation required
- Tight parking space with walls
- Suitable for testing kinematic planning and parking algorithms

**Usage**:
```bash
./navsim_local_debug --scenario scenarios/parking_scenario.json --planner JpsPlanner
```

---

### 3. dynamic_obstacles.json
**Description**: Scenario with moving vehicles and predicted trajectories.

**Features**:
- Start: (0, 0, 0) with initial velocity 5 m/s
- Goal: (30, 0, 0)
- 2 dynamic obstacles with predicted trajectories
- Suitable for testing interaction-aware planning

**Usage**:
```bash
./navsim_local_debug --scenario scenarios/dynamic_obstacles.json --planner JpsPlanner
```

---

## JSON Format

All scenarios follow this structure:

```json
{
  "scenario_name": "string",
  "description": "string",
  "timestamp": 0.0,
  "planning_horizon": 5.0,
  "ego": {
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
    "chassis_model": "differential|ackermann|tracked|four_wheel",
    "kinematics": { ... },
    "limits": { ... }
  },
  "task": {
    "goal_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "type": "GOTO_GOAL|LANE_FOLLOWING|LANE_CHANGE|PARKING|EMERGENCY_STOP",
    "tolerance": {"position": 0.5, "yaw": 0.2}
  },
  "obstacles": [
    {
      "type": "circle|rectangle|polygon",
      "center": {"x": 0.0, "y": 0.0},  // for circle
      "radius": 1.0,                    // for circle
      "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},  // for rectangle
      "width": 2.0,                     // for rectangle
      "height": 1.0,                    // for rectangle
      "vertices": [...]                 // for polygon
    }
  ],
  "dynamic_obstacles": [
    {
      "id": 1,
      "type": "vehicle|pedestrian|cyclist",
      "shape_type": "circle|rectangle",
      "current_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
      "current_twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
      "length": 4.5,
      "width": 2.0,
      "predicted_trajectories": [
        {
          "probability": 1.0,
          "poses": [...],
          "timestamps": [...]
        }
      ]
    }
  ]
}
```

---

## Creating Custom Scenarios

You can create your own scenarios by:

1. **Copy an existing scenario**:
   ```bash
   cp scenarios/simple_corridor.json scenarios/my_scenario.json
   ```

2. **Edit the JSON file**:
   - Modify ego start pose and velocity
   - Change goal pose
   - Add/remove/modify obstacles
   - Add dynamic obstacles with predicted trajectories

3. **Test your scenario**:
   ```bash
   ./navsim_local_debug --scenario scenarios/my_scenario.json --planner JpsPlanner
   ```

---

## Tips

- **Coordinate System**: X-axis points forward, Y-axis points left, yaw is counter-clockwise from X-axis
- **Units**: Distances in meters, angles in radians, velocities in m/s
- **Obstacle Placement**: Make sure obstacles don't overlap with start or goal positions
- **Dynamic Obstacles**: Predicted trajectories should have matching lengths for poses and timestamps

---

## Troubleshooting

**Scenario fails to load**:
- Check JSON syntax (use `jq` or online JSON validator)
- Verify all required fields are present
- Check that numeric values are valid (no NaN or Inf)

**Planner fails**:
- Check that goal is reachable (not inside obstacles)
- Verify ego vehicle constraints are reasonable
- Try a different planner (e.g., StraightLinePlanner for debugging)

**No trajectory generated**:
- Increase planning_horizon
- Relax goal tolerance
- Simplify obstacle configuration

