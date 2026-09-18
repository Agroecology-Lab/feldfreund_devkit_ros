import os
import sys

# ------------------------------------------------------------------------------
# Path Setup
# ------------------------------------------------------------------------------
# Each ROS 2 ament_python package here nests its importable module one level
# below its colcon package folder (src/<pkg>/<pkg>/...), so adding "../src"
# alone only exposes namespace packages like "devkit_driver" with no
# submodules underneath. Add each package's own folder to sys.path instead,
# so "import devkit_driver" resolves straight to src/devkit_driver/devkit_driver/.
sys.path.insert(0, os.path.abspath(".."))
_SRC_ROOT = os.path.abspath("../src")
for _pkg in [
    "devkit_driver",
    "devkit_f2c_planner",
    "devkit_mavlink_bridge",
    "devkit_ui",
]:
    sys.path.insert(0, os.path.join(_SRC_ROOT, _pkg))

# autodoc actually imports these modules to pull docstrings. The CI runner has
# no ROS 2 install and no access to the hardware-specific / simulation deps
# below, so autodoc's import will fail without mocking them out.
autodoc_mock_imports = [
    "rclpy",
    "ament_index_python",
    "launch",
    "launch_ros",
    "sensor_msgs",
    "geometry_msgs",
    "nav_msgs",
    "std_msgs",
    "std_srvs",
    "nav2_msgs",
    "nav2_simple_commander",
    "tf2_ros",
    "lifecycle_msgs",
    "rosgraph_msgs",
    "topological_navigation_msgs",
    "ublox_ubx_msgs",
    "pymavlink",
    "rosys",
    "nicegui",
    "fields2cover",
    "feldfreund_devkit",
    "sowbot_sim",
    "topological_nav_simulator",
]

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
