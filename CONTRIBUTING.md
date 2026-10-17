# Contributing to Sowbot

Thank you for your interest in contributing to Sowbot/ feldfreund_devkit_ros!
This document provides guidelines to help you get started.

## Reporting Issues

If you encounter a bug or have a feature request:
1. Check if the issue already exists in the issue tracker
2. Provide a clear description of the problem or suggestion
3. Include steps to reproduce (for bugs)
4. Add relevant code snippets or error messages

## Code of Conduct

We are committed to providing a welcoming and inclusive environment.
Please be respectful and constructive in all interactions.

## Background

We welcome volunteers of all levels to contribute.
If you're new to ROS, [this course](https://github.com/henki-robotics/robotics_essentials_ros2) may be useful.

## Development Setup

### Local Setup

Before diving into contributing, see the [Quick Start](README.md#quick-start) section of the main README to get a working build running first.

Then create a virtual environment and install the development dependencies.
Use Python 3.11 to 3.13 (`rosys` does not support 3.14), for example 3.12:

```bash
python3.12 -m venv .venv
. .venv/bin/activate
make sync
make test
```

## Coding Style

### General Principles

- **Simplicity** - Always prefer simple, straightforward solutions
- **Readability** - Code should be self-explanatory
- **Maintainability** - Keep files under 200-300 lines; refactor if larger
- **No duplication** - Check for similar code before adding new functionality

### Python Style

- Naming conventions:
  - Python files: lowercase with underscores (e.g., `demo_class.py`)
  - Python classes: CamelCase (e.g., `DemoClass`)
- Use single quotes for strings (except docstrings)
- Use f-strings for string formatting
- Follow PEP 8 with 120 character line length
- Type hints are encouraged for public APIs
- Minimal comments - only for non-obvious logic (start with `NOTE:`)

### Code Organization

- High-level code first, helpers below
- Each file should have a clear, single purpose
- Group related functionality together
- Use early returns to reduce nesting

### Async Programming

- **Never block the event loop** - All async handlers must stay non-blocking
- **Never use `asyncio.create_task()`** - The garbage collector might remove unfinished tasks
- Use appropriate methods based on operation type:
  - `run.cpu_bound()` for CPU-intensive operations
  - `run.io_bound()` for blocking I/O operations
  - `background_tasks.create()` for background tasks that need proper lifecycle management
- Clean up background tasks on teardown
- Handle resource cleanup properly in async contexts

## ROS2 Best Practices

We work towards the [Henki ROS 2 Best Practices](https://github.com/henki-robotics/henki_ros2_best_practices).

## Workflow Guidelines

### Before You Start

1. **Understand the requirements** - Ensure you know what needs to be done
2. **Check existing code** - Look for similar patterns or functionality
3. **Discuss significant changes** - Open an issue or discussion for large changes, or raise it on [Discord](https://discord.com/invite/dmX4sxfjfd) first

### Making Changes

1. Create a feature branch from `caatinga-dev`
2. Make focused commits with clear messages
3. Write or update tests for your changes
4. Ensure all tests pass
5. Format code according to style guidelines
6. Update documentation if needed

### Testing

Run before pushing.
This runs the CI checks (ruff, mypy, pylint, unit tests) plus the tests in `tests/`:

```bash
make pre-commit ruff mypy pylint test && pytest
```

- pylint must score 10.0
- Pre-commit hooks may modify files and exit non-zero; re-stage and re-run
- CI auto-commits ruff fixes to pushed branches; pull before pushing again
- Industrial CI (ROS 2 Jazzy build) runs in CI only

Write tests for:
- New features
- Bug fixes
- Edge cases

## Pull Requests

### Submitting

1. Push your changes to your fork
2. Open a pull request against `caatinga-dev` branch
3. Provide a clear description of the changes
4. Reference any related issues
5. Ensure CI checks pass

### PR Checklist

- [ ] Code follows project style guidelines
- [ ] Tests added/updated and passing
- [ ] Documentation updated if needed
- [ ] No unrelated changes included
- [ ] Commit messages are clear

## AI-Assisted Contributions

We welcome AI-assisted development! When using AI coding assistants:

### For Contributors

- Review [AGENTS.md](AGENTS.md) for AI-specific guidelines
- Ensure AI-generated code follows project principles
- Always review and understand AI suggestions before committing
- Test AI-generated code thoroughly

## Documentation

- Add docstrings for public APIs
- Update README.md for user-facing changes
- Each sentence should be on a new line in Markdown files
- Keep documentation clear and concise

## Questions?

If you have questions or need help:
- Join our [Discord server](https://discord.com/invite/dmX4sxfjfd) for the quickest response
- Check the [Quick Start](README.md#quick-start) section of the main README
- Open a discussion in the issue tracker
- Reach out to maintainers
- Check existing issues and pull requests

Thank you for contributing to feldfreund_devkit_ros!
