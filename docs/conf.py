import os
import sys

# ------------------------------------------------------------------------------
# Project Information
# ------------------------------------------------------------------------------
project = 'Feldfreund DevKit ROS'
copyright = '2026, Agroecology Lab'
author = 'Agroecology Lab'
release = 'caatinga-dev'

# ------------------------------------------------------------------------------
# General Configuration
# ------------------------------------------------------------------------------
# Add Sphinx extensions:
# - breathe: Bridge for Doxygen C++ XML
# - myst_parser: Allows writing documentation in Markdown (.md) alongside reST (.rst)
# - sphinx.ext.autodoc: Python auto-documentation support
# - sphinx.ext.napoleon: Parses Google/NumPy style docstrings in Python
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'breathe',
    'myst_parser',
]

# Source file suffixes
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

master_doc = 'index'
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# ------------------------------------------------------------------------------
# Breathe Configuration (Doxygen Integration)
# ------------------------------------------------------------------------------
# Point Breathe to the Doxygen XML output directory relative to docs/conf.py
breathe_projects = {
    "feldfreund_devkit_ros": os.path.abspath("../xml")
}
breathe_default_project = "feldfreund_devkit_ros"

# ------------------------------------------------------------------------------
# HTML Output Options (Read the Docs Theme)
# ------------------------------------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'navigation_depth': 4,
    'collapse_navigation': False,
    'sticky_navigation': True,
}
