#!/usr/bin/env python3
import os
import subprocess
import re
import sys
import curses

VERSION = "6.1-SOWBOT"

def get_env_config():
    """Reads hardware ports and mode dynamically from .env on the host."""
    config = {"GPS": "NOT_SET", "MCU": "NOT_SET", "MODE": "unknown", "IS_JETSON": "false"}
    env_path = ".env"
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            content = f.read()
            gps = re.search(r"GPS_PORT_ROVER=(.*)", content)
            mcu = re.search(r"MCU_PORT=(.*)", content)
            jetson = re.search(r"IS_JETSON=(.*)", content)
            if gps: config["GPS"] = gps.group(1).strip()
            if mcu: config["MCU"] = mcu.group(1).strip()
            if jetson: config["IS_JETSON"] = jetson.group(1).strip()
    return config

# --- CONFIGURATION (Aligned with your actual launch files) ---
EXPECTED_NODES = [
    "/controller",       # Matches src/devkit_launch/launch/devkit_driver.launch.py
    "/usb_cam",          # Active in your diagnostic
    "/UI_NODE",          # Active in your diagnostic
    "/foxglove_bridge", 
    "/septentrio_gnss"
]

TARGET_TOPICS = {
    "GNSS PVT": "/pvtgeodetic",
    "Battery": "/battery_state",
    "Odometry": "/odom",
    "Motor Cmds": "/cmd_vel",
    "Camera": "/devkit/camera/image_raw/compressed"
}

def run_cmd(cmd):
    try:
        source_cmd = "source /opt/ros/jazzy/setup.bash && [ -f install/setup.bash ] && source install/setup.bash"
        full_cmd = f"{source_cmd}; {cmd}"
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return ""

def draw(stdscr):
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_RED, -1)
    curses.init_pair(3, curses.COLOR_CYAN, -1)
    curses.init_pair(4, curses.COLOR_YELLOW, -1)
    stdscr.nodelay(True)
    stdscr.timeout(1000)
    cfg = get_env_config()

    while True:
        stdscr.clear()
        nodes_list = run_cmd("ros2 node list")
        topics_list = run_cmd("ros2 topic list")
        
        stdscr.addstr(0, 0, f"🚀 AGROLOGY LAB MISSION CONTROL - {VERSION}", curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, f"Host Device: {'Jetson' if cfg['IS_JETSON'] == 'true' else 'PC/ThinkPad'}", curses.color_pair(4))
        stdscr.addstr(2, 0, "="*65, curses.color_pair(3))

        # Hardware Section
        stdscr.addstr(4, 0, "HARDWARE MAPPING (.env)", curses.A_UNDERLINE)
        hw_labels = [("GPS", cfg["GPS"]), ("MCU", cfg["MCU"])]
        for i, (label, port) in enumerate(hw_labels):
            status = "[VIRTUAL]" if "virtual" in port.lower() else "[PHYSICAL]"
            col = curses.color_pair(4) if "virtual" in port.lower() else curses.color_pair(1)
            stdscr.addstr(5+i, 2, f"{label:<4} {port:<18}: {status}", col)

        # Nodes Section
        stdscr.addstr(8, 0, "ACTIVE ROS 2 NODES", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in nodes_list
            status = "[ACTIVE]" if exists else "[OFFLINE]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(9+i, 2, f"{node:<28}: {status}", col)

        # Topic Stream Section
        stdscr.addstr(15, 0, "DATA STREAM VERIFICATION", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            exists = topic in topics_list
            status = "[FLOWING]" if exists else "[NO DATA]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(16+i, 2, f"{label:<14}: {status}", col)

        stdscr.addstr(22, 0, "Press 'q' to EXIT", curses.color_pair(4))
        stdscr.refresh()
        if stdscr.getch() == ord('q'):
            break

if __name__ == "__main__":
    curses.wrapper(draw)
