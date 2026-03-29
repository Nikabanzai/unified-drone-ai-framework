#!/usr/bin/env python3
"""
Example: Full inspection mission using coverage planning.

This demonstrates:
    - Boustrophedon grid coverage planner
    - MockBackend simulation
    - UnifiedController (PID)
    - MissionOrchestrator

Run:
    PYTHONPATH=. python examples/inspect_mission.py
"""

from unified.planning.coverage import boustrophedon_grid
from unified.simulation.backend import MockBackend
from unified.control.controller import ControlMode, UnifiedController
from unified.mission.orchestrator import MissionOrchestrator


def main():
    print("=" * 60)
    print("Unified Drone AI Framework — Inspection Mission Example")
    print("=" * 60)

    # 1. Generate coverage waypoints (10m x 10m area, 5m altitude, 2m spacing)
    print("\n[1] Generating coverage plan...")
    waypoints = boustrophedon_grid(
        x_min=0, x_max=10,
        y_min=0, y_max=10,
        altitude=5.0,
        spacing=2.0,
        start_corner="bottom_left",
    )
    print(f"    Generated {len(waypoints)} waypoints for 10×10m area")

    # 2. Create simulation + controller + orchestrator
    print("\n[2] Initializing mission components...")
    sim = MockBackend(num_envs=1, dt=0.02)
    controller = UnifiedController(mode=ControlMode.PID)
    orch = MissionOrchestrator(sim=sim, controller=controller)

    # 3. Load plan
    orch.load_plan(waypoints)
    print(f"    Loaded plan into orchestrator")

    # 4. Run mission
    print("\n[3] Running inspection mission...")
    print("    (This uses MockBackend — no real physics)")
    log = orch.run(max_steps=2000, waypoint_tolerance=0.5)

    # 5. Summary
    print("\n" + "-" * 40)
    print("Mission Summary")
    print("-" * 40)
    print(f"  Waypoints:    {len(waypoints)}")
    print(f"  Steps taken:  {len(log)}")
    print(f"  Progress:     {orch.progress*100:.1f}%")
    print(f"  Final dist:   {log[-1]['dist']:.3f} m" if log else "  (no steps)")

    # Show first few and last waypoint positions
    if waypoints:
        print(f"\n  First waypoint: {waypoints[0]}")
        print(f"  Last waypoint:  {waypoints[-1]}")

    print("\n" + "=" * 60)
    print("Done! ✅")
    print("=" * 60)


if __name__ == "__main__":
    main()
