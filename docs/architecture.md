# Architecture

This document describes the high-level design of the **Unified Drone AI Framework**.

---

## Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        UNIFIED DRONE AI FRAMEWORK                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌─────────────────────────┐    ┌─────────────────────────────────────┐    │
│   │   INSPECTION PLANNING   │───▶│         MISSION ORCHESTRATOR        │    │
│   │       (planning/)       │    │        (mission/orchestrator.py)    │    │
│   │                         │    │                                     │    │
│   │  • boustrophedon_grid   │    │  • Loads plan → control mode switch │    │
│   │  • spiral_grid          │    │  • Runs plan → control → sim loop   │    │
│   │  • WaypointList         │    │  • Telemetry logging                │    │
│   └──────────┬──────────────┘    └───────────────┬─────────────────────┘    │
│              │                                   │                          │
│              ▼                                   ▼                          │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                         CONTROL LAYER                                │   │
│   │   ┌─────────────────────────┐    ┌─────────────────────────────┐    │   │
│   │   │   CLASSICAL (PID)       │    │   RL (Policy)               │    │   │
│   │   │   (control/controller)  │    │   (policy callable)         │    │   │
│   │   │                         │    │                             │    │   │
│   │   │ • Cascaded pos→att PID  │    │ • External policy fn        │    │   │
│   │   │ • CF2X/CF2P gains       │    │ • Stable-baselines3 hook    │    │   │
│   │   │ • UnifiedController     │    │ • JAX policy hook           │    │   │
│   │   └──────────┬──────────────┘    └──────────────┬──────────────┘    │   │
│   │              │                                  │                    │   │
│   │              └──────────────┬───────────────────┘                    │   │
│   │                             ▼                                        │   │
│   │                    Action: [roll, pitch, yaw, thrust]                │   │
│   └─────────────────────────────┬────────────────────────────────────────┘   │
│                                 │                                            │
│                                 ▼                                            │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                       SIMULATION LAYER                               │   │
│   │   ┌─────────────────────────────────────────────────────────────┐   │   │
│   │   │              SimulationBackend (ABC)                        │   │   │
│   │   │                                                             │   │   │
│   │   │  • reset() → obs dict                                       │   │   │
│   │   │  • step(action) → (obs, reward, done, info)                 │   │   │
│   │   │  • get_obs() → dict                                         │   │   │
│   │   └─────────────────────────────────────────────────────────────┘   │   │
│   │                                                                     │   │
│   │   ┌───────────────────────────┐  ┌─────────────────────────────┐   │   │
│   │   │   MockBackend (default)   │  │   Real (future)             │   │   │
│   │   │   • Pure numpy            │  │   • crazyflow (JAX/MuJoCo)  │   │   │
│   │   │   • For testing           │  │   • gym-pybullet-drones     │   │   │
│   │   └───────────────────────────┘  └─────────────────────────────┘   │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Layers

### 1. Simulation Layer (`unified/simulation/`)

| Class | Purpose |
|-------|---------|
| `SimulationBackend` | Abstract base class defining `reset()`, `step()`, `get_obs()`, `close()` |
| `MockBackend` | Pure-numpy mock for unit testing (no physics engine) |
| `Backend` (Enum) | `CRAZYFLOW` / `PYBULLET` — selection for future real backends |

**Obs dict keys** (matches crazyflow `DroneEnv`):

```python
{
    "pos": (N, 3),      # position [m]
    "quat": (N, 4),     # quaternion [x, y, z, w]
    "vel": (N, 3),      # velocity [m/s]
    "ang_vel": (N, 3),  # angular velocity [rad/s]
}
```

**Action**: `(N, 4)` — `[roll, pitch, yaw, thrust]` or `[P0, P1, P2, P3]` RPMs.

---

### 2. Control Layer (`unified/control/`)

| Class | Purpose |
|-------|---------|
| `ControlMode` (Enum) | `PID` / `RL` / `HYBRID` |
| `UnifiedController` | Computes action from obs + target; PID by default |

**PID Controller**:
- Cascaded position → attitude PID
- Gains from `DSLPIDControl` (CF2X): `kp_pos=[0.4, 0.4, 1.25]`, `kp_att=[70000, 70000, 60000]`
- Gravity compensation on Z axis
- Output: `[roll, pitch, yaw, thrust]` in radians / Newtons

**RL Mode**:
- Accepts `policy: Callable[[obs], action]`
- Raises if no policy provided

---

### 3. Planning Layer (`unified/planning/`)

| Function | Purpose |
|----------|---------|
| `boustrophedon_grid(...)` | Lawnmower coverage pattern over rectangle |
| `spiral_grid(...)` | Outward spiral from center |

**Output**: `List[Tuple[float, float, float]]` — `(x, y, z)` waypoints.

---

### 4. Mission Layer (`unified/mission/`)

| Class | Purpose |
|-------|---------|
| `MissionOrchestrator` | Wires plan → controller → sim → next waypoint |

**Flow**:

```
for each step:
    target = current_waypoint
    obs = sim.get_obs()
    action = controller.compute(obs, target_pos)
    sim.step(action)
    if dist(pos, target) < tolerance:
        advance to next waypoint
```

**Telemetry log**: `[{step, wp_idx, pos, target, dist}, ...]`

---

### 5. Adapters (`unified/adapters/`)

| Module | Purpose |
|--------|---------|
| `state_adapter.py` | `(20,)` gym-pybullet obs ↔ crazyflow `{"pos", "quat", "vel", "ang_vel"}` |
| `action_adapter.py` | RPM `(4,)` ↔ attitude `(4,)` using mixer matrix |

These bridge the two underlying repos' data formats.

---

## Dependency Graph

```
mission.orchestrator
    ├── simulation.backend (SimulationBackend ABC)
    ├── control.controller (UnifiedController)
    └── (planning.coverage for waypoint generation — external)

control.controller
    └── numpy only (pure PID math)

planning.coverage
    └── numpy only (pure geometry)

simulation.backend
    └── numpy only (MockBackend has trivial physics)

adapters
    └── numpy only
```

No cross-repo imports at runtime (except optional user wiring).

---

## Data Flow

```
┌─────────────┐     ┌───────────────┐     ┌───────────────┐
│   Planner   │────▶│  Waypoints    │────▶│  Orchestrator │
│ (coverage)  │     │  [(x,y,z),..] │     │  (mission)    │
└─────────────┘     └───────────────┘     └───────┬───────┘
                                                  │
                          ┌───────────────────────┴───────────────────────┐
                          ▼                                               ▼
                  ┌───────────────┐                               ┌───────────────┐
                  │   Controller  │◀──────────────────────────────│   Simulator   │
                  │  (PID / RL)   │     obs dict                  │ (Backend ABC) │
                  └───────┬───────┘                               └───────┬───────┘
                          │                                               │
                          │  action (4,)                                  │ step()
                          └───────────────────────────────────────────────┘
```

---

## Extending to Real Sim

To use real physics (crazyflow / PyBullet):

1. Create a new `RealBackend(SimulationBackend)` subclass
2. In `step()`: call `crazyflow.Sim.step()` or `p.stepSimulation()`
3. In `get_obs()`: convert sim state to the dict format
4. Pass to `MissionOrchestrator(sim=RealBackend(...))`

The `SimulationBackend` ABC ensures all backends are drop-in compatible.

---

## Testing Philosophy

| Test Type | What | Where |
|-----------|------|-------|
| Unit | Pure numpy, no sim | `tests/unit/test_*.py` |
| Integration | Import sanity only | `tests/integration/test_sim_bridge.py` |
| No simulation runs | Fast, CI-friendly | All tests < 1s |

**Rule**: Every module has a corresponding test file. Tests pass with only `numpy` + `pytest`.

---

## Future Work

| Item | Priority |
|------|----------|
| Real `crazyflow` backend wrapper | Medium |
| Real `gym-pybullet-drones` backend wrapper | Medium |
| RL policy training examples | Low |
| Visualization / plotting | Low |
| ROS2 bridge | Low |

---

## License

MIT
