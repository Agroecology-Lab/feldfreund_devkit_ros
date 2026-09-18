import os
import sys

# ------------------------------------------------------------------------------
# Path Setup
# ------------------------------------------------------------------------------
# Add the project root and src directories to sys.path so sphinx.ext.autodoc
# can discover and import Python packages and modules inside src/
sys.path.insert(0, os.path.abspath(".."))
sys.path.insert(0, os.path.abspath("../src"))

# ------------------------------------------------------------------------------
# Project Information
# ------------------------------------------------------------------------------
project = "Feldfreund DevKit ROS"
copyright = "2026, Agroecology Lab"
author = "Agroecology Lab"
release = "caatinga-dev"

# ------------------------------------------------------------------------------
# General Configuration
# ------------------------------------------------------------------------------
# Sphinx Extensions
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "breathe",
    "myst_parser",
]

# Generate autosummary pages automatically
autosummary_generate = True

# Docstring configuration (Napoleon for Google & NumPy docstrings)
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True

# Autodoc default options
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}

# Source file suffixes
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

# MyST-Parser Configuration for Markdown extensions
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "fieldlist",
]
myst_heading_anchors = 3

master_doc = "index"
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "xml"]

# ------------------------------------------------------------------------------
# Breathe Configuration (Doxygen C++ / ROS Integration)
# ------------------------------------------------------------------------------
# Points directly to the xml folder generated inside docs/ by Doxygen (OUTPUT_DIRECTORY = docs)
breathe_projects = {
    "feldfreund_devkit_ros": os.path.abspath("xml")
}
breathe_default_project = "feldfreund_devkit_ros"

# ------------------------------------------------------------------------------
# HTML Output Options (Read the Docs Theme)
# ------------------------------------------------------------------------------
html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "navigation_depth": 4,
    "collapse_navigation": False,
    "sticky_navigation": True,
}
