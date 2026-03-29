# Contributing to Unified Drone AI Framework

Thank you for your interest in contributing! This document provides guidelines and instructions.

## Development Setup

1. **Fork and clone the repo**
   ```bash
   git clone https://github.com/<your-username>/unified-drone-ai-framework.git
   cd unified-drone-ai-framework
   ```

2. **Create a virtual environment**
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # or `.venv\Scripts\activate` on Windows
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   pip install pytest ruff
   ```

4. **Run tests**
   ```bash
   PYTHONPATH=. pytest -v
   ```

## Code Style

- Follow PEP 8. We use `ruff` for linting:
  ```bash
  ruff check .
  ```
- Type hints are encouraged but not enforced.
- Keep functions focused and small.

## Adding a New Module

1. Create `unified/<module>/__init__.py` with exports.
2. Implement the module logic (pure numpy preferred).
3. Create `tests/unit/test_<module>.py` with ≥3 test cases.
4. Ensure `pytest -v` passes.

## Pull Request Process

1. Create a feature branch: `git checkout -b feature/my-feature`
2. Commit your changes with clear messages.
3. Push to your fork and open a PR.
4. CI runs `pytest -v` automatically; all tests must pass.
5. A maintainer will review and merge.

## Reporting Issues

- Use GitHub Issues with a clear title and description.
- Include steps to reproduce, expected vs actual behavior.
- Tag with `bug`, `enhancement`, or `question`.

## License

By contributing, you agree that your contributions are licensed under the MIT License.
