# Python Coding

Use this skill whenever you write or modify Python code in this repository.

## Environment setup

1. Run `source skills/python-coding/activate-venv.sh` before running Python commands.
2. Use the virtual environment in `.venv` for all Python tooling and tests.

## How to write Python code here

- Follow project standards in `CONTRIBUTING.md` and `AGENTS.md`.
- Keep code simple, focused, and readable.
- Prefer small functions with clear names and explicit error handling.
- Add or update tests when behavior changes.
- Reuse existing patterns in the codebase before introducing new abstractions.

## Required checks before commit

1. Run `make lint`.
2. Run `make test`.
3. Only commit when both commands succeed.
