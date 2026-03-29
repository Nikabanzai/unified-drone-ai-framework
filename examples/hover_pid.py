#!/usr/bin/env python3
"""
Example: Hover with PID control using MockBackend.

This demonstrates the unified framework:
    - MockBackend for simulation (no crazyflow/PyBullet needed)
    - UnifiedController with PID mode
    - MissionOrchestrator to run the mission

Run:
    PYTHONPATH=. python examples/hover_pid.py
"""

import numpy as np

from unified.simulation.backend import MockBackend
from unified.control.controller import ControlMode, UnifiedController
from unified.mission.orchestrator import MissionOrchestrator


def main():
    print("=" * 60)
    print("Unified Drone AI Framework — Hover PID Example")
    print("=" * 60)

    # Create mock simulation (1 drone, 50Hz)
    sim = MockBackend(num_envs=1, dt=0.02)

    # Create PID controller
    controller = UnifiedController(mode=ControlMode.PID)

    # Create orchestrator
    orch = MissionOrchestrator(sim=sim, controller=controller)

    # Define a simple hover mission: stay at (0,0,1)
    plan = [
        (0.0, 0.0, 1.0),  # Hover at 1 meter
    ]
    orch.load_plan(plan)

    print(f"\nLoaded plan with {len(plan)} waypoint(s)")
    print("Running mission (max 200 steps)...")

    # Run mission
    log = orch.run(max_steps=200, waypoint_tolerance=0.1)

    print(f"\nCompleted in {len(log)} steps")
    print(f"Progress: {orch.progress*100:.1f}%")

    # Show final state
    if log:
        last = log[-1]
        print(f"\nFinal state:")
        print(f"  Position: {last['pos']}")
        print(f"  Target:   {last['target']}")
        print(f"  Distance: {last['dist']:.4f} m")

    print("\n" + "=" * 60)
    print("Done! ✅")
    print("=" * 60)


if __name__ == "__main__":
    main()
