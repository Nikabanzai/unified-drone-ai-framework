"""
Mission orchestrator.

Coordinates planning, control, and simulation to execute autonomous missions.

Usage:
    from unified.mission.orchestrator import MissionOrchestrator
    from unified.simulation.backend import MockBackend

    orch = MissionOrchestrator(sim=MockBackend())
    orch.load_plan([(0,0,1), (1,0,1)])
    orch.run(max_steps=100)
"""

from .orchestrator import MissionOrchestrator

__all__ = ["MissionOrchestrator"]
