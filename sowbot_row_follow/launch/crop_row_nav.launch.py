#!/usr/bin/env python3

# Copyright 2022 Agricultural-Robotics-Bonn
# Copyright 2026 Agroecology Lab Ltd
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# crop_row_nav.launch.py
# ======================
# Derived from:
#   visual-multi-crop-row-navigation
#   https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation
#   Authors: Alireza Ahmadi, Michael Halstead, Chris McCool (Agricultural-Robotics-Bonn)
#   ROS2 port: Agroecology Lab CIC
#   Reference: Ahmadi et al., "Towards Autonomous Crop-Agnostic Visual Navigation
#              in Arable Fields", arXiv:2109.11936, 2021.
#              Ahmadi et al., ICRA 2020.
#
# Adapted for caatinga_vision / Sowbot by Agroecology Lab Ltd
#
# Launches crop_row_node with its own dedicated camera (standalone, no shared camera).
# The crop_row_node opens the camera directly via OpenCV VideoCapture.
# Run independently or alongside ia.launch.py using a different camera_index.
#
# Topics published:
#   /row_nav/image_raw                  Raw camera frames
#   /row_nav/debug_image                Annotated frames (view in Foxglove)
#   /aoc/conditions/row_offset          Normalised lateral offset [-1,1]
#   /aoc/conditions/row_heading_error   Heading error in radians
#   /aoc/heartbeat/neo_vision           Perception heartbeat for M10
#   /cmd_vel                            Always published; gated by /row_follow/enable service
#
# Usage:
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py camera_index:=2
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py camera_height_m:=0.8 linear_vel:=0.05

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Base params from file — hardware-specific values live here, not in this launch file.
    # Edit caatinga_vision/config/crop_row_params.yaml to set camera_height_m, camera_tilt_deg etc.
    params_file = PathJoinSubstitution(
        [FindPackageShare("sowbot_row_follow"), "config", "crop_row_params.yaml"]
    )

    return LaunchDescription([

        # Command-line overrides — these take precedence over crop_row_params.yaml.
        # Useful for quick experiments without editing the YAML.
        DeclareLaunchArgument("camera_index", default_value="0"),

        Node(
            package="sowbot_row_follow",
            executable="crop_row_node",
            name="crop_row_node",
            output="screen",
            parameters=[
                # 1. Load all defaults from the hardware params file
                params_file,
                # 2. Allow launch-arg override for camera index
                {"camera_index": LaunchConfiguration("camera_index")},
            ],
        ),
    ])
