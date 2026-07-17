.PHONY: help sync install-ci mypy pylint ruff test pre-commit

default: help

## Available commands for this Makefile, use 'make <command>' to execute:

##
## ---------------

## help		Print commands help.
help: Makefile
	@sed -n 's/^##//p' $<

## sync		Install all dependencies for development and testing.
sync:
	pip install -r requirements-dev.txt

## install-ci	Install all dependencies for CI testing.
install-ci:
	pip install -r requirements-dev.txt

## mypy		Run mypy type checks.
mypy:
	mypy ./devkit_driver ./devkit_launch ./devkit_ui --exclude 'setup\.py$$' --non-interactive --install-types

## pylint		Run pylint code analysis.
pylint:
	pylint ./devkit_driver/devkit_driver ./devkit_launch/launch ./devkit_ui/devkit_ui

## ruff		Run ruff code analysis.
ruff:
	ruff check ./devkit_driver/devkit_driver ./devkit_launch/launch ./devkit_ui/devkit_ui

## test		Run the pytest suite (requires a sourced ROS environment).
# NOTE: the ROS launch_testing pytest plugins are incompatible with modern pytest, so we
# disable plugin autoloading and explicitly load only pytest-asyncio (for asyncio_mode).
test:
	PYTHONPATH=$(CURDIR)/devkit_driver:$(CURDIR)/devkit_ui:$$PYTHONPATH \
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -p pytest_asyncio.plugin

## pre-commit	Run pre-commit hooks on all files.
pre-commit:
	pre-commit run --all-files

## check		Run all code checks (mypy, pre-commit, pylint).
check: mypy pre-commit pylint
