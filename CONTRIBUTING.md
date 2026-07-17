# Contributing to feldfreund_devkit_ros

Thank you for your interest in contributing to feldfreund_devkit_ros!
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

## Development Setup

### Prerequisites

- Python 3.11+ and pip installed
- Git for version control

### Local Setup

1. Fork and clone the repository
2. Create a virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

3. Install dependencies:

```bash
make sync
```

4. Install pre-commit hooks:

```bash
pre-commit install
```

### Dependencies

`requirements.txt` (runtime) and `requirements-dev.txt` (development and testing) are the single source of truth for dependencies.
`pyproject.toml` only holds tool configuration (ruff, mypy, pylint, pytest) and does not declare dependencies.
The project is installed via `pip` (see `make sync` and the Docker image); it does not use `uv`, so there is no `uv.lock`.

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

## Workflow Guidelines

### Before You Start

1. **Understand the requirements** - Ensure you know what needs to be done
2. **Check existing code** - Look for similar patterns or functionality
3. **Discuss significant changes** - Open an issue or discussion for large changes

### Making Changes

1. Create a feature branch from `main`
2. Make focused commits with clear messages
3. Write or update tests for your changes
4. Ensure all tests pass
5. Format code according to style guidelines
6. Update documentation if needed

### Testing

Run tests before submitting:

```bash
pytest
```

Write tests for:
- New features
- Bug fixes
- Edge cases

### Code Quality

Check your code with:

```bash
make check # which runs:
# ruff check .
# mypy .
# pylint ./feldfreund_devkit_ros
```

Or let pre-commit handle it:

```bash
make pre-commit # which runs:
# pre-commit run --all-files
```

## Pull Requests

### Submitting

1. Push your changes to your fork
2. Open a pull request against `main`
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

### For Reviewers

AI can assist with code reviews according to [.github/copilot-instructions.md](.github/copilot-instructions.md).

**Before submitting a PR**, you should review your changes with an AI assistant:

In **Cursor** or **VS Code with GitHub Copilot Chat**:

Select Agent Mode with claude-sonnet-4 and write:
```
Review my current branch according to @.github/copilot-instructions.md
```

Or in **Cursor**, use these custom commands for easy access:
1. Type `/` in Cursor chat
2. Select commands:
   - `/review-uncommitted` - Review your local uncommitted changes
   - `/review-branch` - Review your current branch vs main
   - `/simplify` - Suggest ways to simplify code
   - `/explain` - Explain what code does

Ensure to address any valid feedback. This will make your life and that of the maintainers much easier.

## Documentation

- Add docstrings for public APIs
- Update README.md for user-facing changes
- Each sentence should be on a new line in Markdown files
- Keep documentation clear and concise

## Questions?

If you have questions or need help:
- Open a discussion in the issue tracker
- Reach out to maintainers
- Check existing issues and pull requests

Thank you for contributing to feldfreund_devkit_ros!
