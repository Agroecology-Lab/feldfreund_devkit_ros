.PHONY: help sync install-ci mypy pylint pre-commit

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
	mypy ./src/devkit_driver ./src/devkit_launch ./src/devkit_ui --exclude 'setup\.py$$' --non-interactive --install-types

## pylint		Run pylint code analysis.
pylint:
	pylint \
		--disable=duplicate-code \
		./src/devkit_driver/devkit_driver \
		./src/devkit_launch/launch \
		./src/devkit_ui/devkit_ui

## ruff		Run ruff code analysis.
ruff:
	find ./src/devkit_driver/devkit_driver -name '*.py' | xargs ruff check
	find ./src/devkit_launch/launch -name '*.py' | xargs ruff check
	find ./src/devkit_ui/devkit_ui -name '*.py' | xargs ruff check

## pre-commit	Run pre-commit hooks on all files.
pre-commit:
	pre-commit run --all-files

## check		Run all code checks (mypy, pre-commit, pylint).
check: mypy pre-commit pylint
