# Unified Drone AI Framework

**A unified framework for drone simulation, control, and planning — combining RL, classical PID, and inspection planning.**

Built on the shoulders of:
- **[crazyflow](https://github.com/utiasDSL/crazyflow)** — JAX-based differentiable drone simulation
- **[gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones)** — Gymnasium environments + PID controllers

Made together with [@caglar28](https://github.com/caglar28). 

---

## 🎯 Goals

| Goal | Status |
|------|--------|
| Python-first, WSL/Docker friendly | ✅ |
| No Isaac Sim / ROS / PX4 heavy deps | ✅ |
| Fast unit tests (no simulation runs) | ✅ |
| Swap PID ↔ RL control mid-mission | 🚧 |
| Inspection planning module | 🚧 |

---

## 📁 Repository Structure

```
unified-drone-ai-framework/
├── unified/
│   ├── adapters/
│   │   ├── state_adapter.py      # (20,) ↔ {"pos","quat","vel","ang_vel"}
│   │   └── action_adapter.py     # RPM ↔ attitude [roll,pitch,yaw,thrust]
│   ├── control/
│   │   └── controller.py         # UnifiedController(PID|RL|HYBRID)
│   ├── simulation/
│   │   └── backend.py            # UnifiedSimulator(crazyflow|pybullet)
│   ├── planning/
│   │   ├── waypoints.py          # WaypointList, generate_from_bbox()
│   │   ├── coverage.py           # boustrophedon_grid(), spiral()
│   │   └── validator.py          # collision_check()
│   ├── mission/
│   │   └── orchestrator.py       # MissionOrchestrator
│   └── __init__.py
├── tests/
│   ├── unit/
│   │   └── test_adapters.py      # 32 tests — pure numpy
│   └── integration/
│   │   └── test_sim_bridge.py    # import sanity (no sim runs)
│   └── conftest.py
├── examples/
│   ├── hover_pid.py
│   └── inspect_mission.py
├── docs/
│   └── architecture.md
├── scripts/
│   └── quick_install.sh
├── .github/
│   └── workflows/
│       └── ci.yml
├── pyproject.toml
├── requirements.txt
└── README.md
```

---

## 🗺️ Execution Plan — Start to Finish

| # | Phase | Task | Deliverable | Dependencies | Est. Effort |
|---|-------|------|-------------|--------------|-------------|
| **1** | **Bootstrap** | Create repo structure | `unified-drone-ai-framework/` tree | — | 30 min |
| **2** | **Bootstrap** | Pin Python 3.12, add `pyproject.toml` | Build system works | — | 15 min |
| **3** | **Bootstrap** | Add `requirements.txt` (numpy, pytest) | `pip install -r requirements.txt` works | — | 10 min |
| **4** | **Adapters (Done)** | ✅ `state_adapter.py` — (20,) ↔ crazyflow dict | File exists, 4 functions | numpy | ✅ Complete |
| **5** | **Adapters (Done)** | ✅ `action_adapter.py` — RPM ↔ attitude | File exists, mixer matrices | numpy | ✅ Complete |
| **6** | **Tests (Done)** | ✅ `test_adapters.py` — 32 unit tests | `pytest tests/unit/test_adapters.py -v` passes | pytest | ✅ Complete |
| **7** | **Simulation** | Create `simulation/backend.py` | `UnifiedSimulator` class stub | numpy | 2 hrs |
| **8** | **Simulation** | Add `crazyflow` backend wrapper | `Backend.CRAZYFLOW` works | crazyflow installed | 4 hrs |
| **9** | **Simulation** | Add `pybullet` backend wrapper | `Backend.PYBULLET` works | gym-pybullet-drones installed | 3 hrs |
| **10** | **Control** | Create `control/controller.py` | `UnifiedController(mode)` class | numpy | 3 hrs |
| **11** | **Control** | Wire PID via `DSLPIDControl` | `mode=PID` calls gym-pb-drones PID | gym-pybullet-drones | 2 hrs |
| **12** | **Control** | Wire RL hook (stable-baselines3) | `mode=RL` loads PPO policy | stable-baselines3 (opt) | 2 hrs |
| **13** | **Planning** | Create `planning/waypoints.py` | `WaypointList`, `generate_from_bbox()` | numpy | 3 hrs |
| **14** | **Planning** | Create `planning/coverage.py` | `boustrophedon_grid()`, `spiral()` | numpy | 4 hrs |
| **15** | **Planning** | Create `planning/validator.py` | `collision_check(waypoints, sim)` | numpy (opt: crazyflow) | 2 hrs |
| **16** | **Mission** | Create `mission/orchestrator.py` | `MissionOrchestrator` class | numpy | 4 hrs |
| **17** | **Mission** | Wire plan → control mode switch | PID for hover, RL for complex | control + planning | 3 hrs |
| **18** | **Integration** | Create `tests/integration/test_sim_bridge.py` | Import sanity (no sim) | — | 1 hr |
| **19** | **CI** | Add `.github/workflows/ci.yml` | GitHub Actions: pytest on push | — | 1 hr |
| **20** | **Docs** | Write `docs/architecture.md` | Architecture diagram + layers | — | 2 hrs |
| **21** | **Examples** | Create `examples/hover_pid.py` | Minimal PID hover demo | sim + control | 2 hrs |
| **22** | **Examples** | Create `examples/inspect_mission.py` | Full mission: plan → PID → done | all modules | 3 hrs |
| **23** | **Polish** | Add type hints, docstrings | `mypy --ignore-missing-imports` clean | — | 2 hrs |
| **24** | **Release** | Tag v0.1.0, write CHANGELOG | GitHub release | — | 1 hr |

**Total estimated effort:** ~45–50 hours (≈ 1–2 weeks part-time)

---

## 🔗 Module Dependency Graph

```
                    ┌─────────────────────────────────────┐
                    │        Mission Orchestrator         │
                    │   (mission/orchestrator.py)         │
                    └──────────────┬──────────────────────┘
                                   │
           ┌───────────────────────┼───────────────────────┐
           ▼                       ▼                       ▼
┌─────────────────────┐ ┌──────────────────┐ ┌──────────────────────┐
│   Planning Module   │ │  Control Module  │ │   Simulation Module  │
│   (planning/)       │ │  (control/)      │ │   (simulation/)      │
│ • waypoints.py      │ │ • controller.py  │ │ • backend.py         │
│ • coverage.py       │ │   PID | RL       │ │   CRAZYFLOW|PYBULLET │
│ • validator.py      │ └────────┬─────────┘ └──────────┬───────────┘
└─────────────────────┘          │                      │
                                 │                      │
                    ┌────────────┴────────────┐         │
                    ▼                         ▼         │
           ┌─────────────────┐      ┌─────────────────┐ │
           │  State Adapter  │◄────▶│  Action Adapter │◄┘
           │ (adapters/)     │      │  (adapters/)    │
           └─────────────────┘      └─────────────────┘
```

---

## 🧪 Running Tests

```bash
# All unit tests (pure numpy, no sim)
pytest tests/unit/ -v

# Integration tests (import sanity)
pytest tests/integration/ -v

# Full test suite
pytest -v

# With coverage
pytest --cov=unified tests/
```

---

## 🚀 Quick Start (Future)

```bash
# 1. Clone
git clone <this-repo>
cd unified-drone-ai-framework

# 2. Install deps
pip install -r requirements.txt
pip install -e /path/to/crazyflow
pip install -e /path/to/gym-pybullet-drones

# 3. Run tests
pytest -v

# 4. Run example
python examples/hover_pid.py
```

---

## 📝 License

MIT

---

## 🙏 Credits

- **crazyflow** — UTIAS DSL (Martin Schuck et al.)
- **gym-pybullet-drones** — UTIAS DSL (Jacopo Panerati et al.)
