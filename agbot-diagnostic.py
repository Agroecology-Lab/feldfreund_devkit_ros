#!/usr/bin/env python3
import os
import subprocess
import time
import curses
import re
import sys

# --- VERSIONING ---
VERSION = "6.0-JAZZY"

def get_env_config():
    """Reads hardware ports and mode dynamically from .env on the host."""
    config = {"GPS": "NOT_SET", "MCU": "NOT_SET", "MODE": "unknown"}
    env_path = ".env"  # Assumes execution from workspace root
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            content = f.read()
            gps = re.search(r"GPS_PORT=(.*)", content)
            mcu = re.search(r"MCU_PORT=(.*)", content)
            mode = re.search(r"MODE=(.*)", content)
            if gps: config["GPS"] = gps.group(1).strip()
            if mcu: config["MCU"] = mcu.group(1).strip()
            if mode: config["MODE"] = mode.group(1).strip()
    return config

# --- CONFIGURATION (Updated for Feldfreund Devkit) ---
EXPECTED_NODES = [
    "/devkit_node", 
    "/septentrio_gnss", 
    "/usb_cam", 
    "/foxglove_bridge", 
    "/devkit_ui"
]
TARGET_TOPICS = {
    "GNSS PVT": "/pvtgeodetic",
    "Battery": "/battery_state",
    "Odometry": "/odom",
    "Motor Cmds": "/cmd_vel",
    "Camera": "/devkit/camera/image_raw/compressed"
}

def run_cmd(cmd):
    """Safe ROS 2 Jazzy command execution."""
    try:
        # Targeting Jazzy setup
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
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(1000)
    cfg = get_env_config()

    while True:
        stdscr.clear()
        nodes_list = run_cmd("ros2 node list")
        topics_list = run_cmd("ros2 topic list")
        
        stdscr.addstr(0, 0, f"🚀 AGROLOGY LAB MISSION CONTROL - JAZZY V{VERSION}", curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, f"Current Mode: {cfg['MODE'].upper()}", curses.color_pair(4))
        stdscr.addstr(2, 0, "="*65, curses.color_pair(3))

        # Hardware Section
        stdscr.addstr(4, 0, "HARDWARE STATUS", curses.A_UNDERLINE)
        fixusb_exists = os.path.exists("fixusb.py")
        fixusb_status = "[FOUND]" if fixusb_exists else "[MISSING]"
        stdscr.addstr(5, 2, f"fixusb.py logic   : {fixusb_status}", curses.color_pair(1 if fixusb_exists else 2))

        hw_labels = [("GPS", cfg["GPS"]), ("MCU", cfg["MCU"])]
        for i, (label, port) in enumerate(hw_labels):
            exists = os.path.exists(port) if port != "NOT_SET" else False
            status = "[ONLINE]" if exists else "[OFFLINE/SIM]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(6+i, 2, f"{label:<4} {port:<18}: {status}", col)

        # Nodes Section
        stdscr.addstr(9, 0, "ACTIVE ROS 2 NODES", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in nodes_list
            status = "[ACTIVE]" if exists else "[OFFLINE]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(10+i, 2, f"{node:<28}: {status}", col)

        # Topic Presence
        stdscr.addstr(16, 0, "DATA STREAM VERIFICATION", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            exists = topic in topics_list
            status = "[FLOWING]" if exists else "[NO DATA]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(17+i, 2, f"{label:<14}:", curses.A_DIM)
            stdscr.addstr(17+i, 20, status, col)

        stdscr.addstr(23, 0, "Press 'q' for POST-FLIGHT AUDIT", curses.color_pair(4))
        stdscr.refresh()
        if stdscr.getch() == ord('q'):
            break

if __name__ == "__main__":
    do_full_sweep = len(sys.argv) > 1 and sys.argv[1].lower() == "full"

    try:
        curses.wrapper(draw)
    except KeyboardInterrupt:
        pass
    
    print("\n" + "═"*75)
    print(f"🔎 ROS 2 JAZZY GRAPH AUDIT (V{VERSION})")
    print("═"*75)

    nodes_raw = run_cmd("ros2 node list")
    found_nodes = [n for n in nodes_raw.split('\n') if n]
    
    for node in found_nodes:
        print(f"\n● {node.upper()}")
        node_info = run_cmd(f"ros2 node info {node}")
        
        # Parse Publishers
        pubs_match = re.search(r'Publishers:(.*?)Service Servers:', node_info, re.S)
        if pubs_match:
            clean_pubs = [p.strip() for p in pubs_match.group(1).split('\n') if '/' in p]
            print(f"  └─ Publishers  : {', '.join(clean_pubs[:5])}")

    if not do_full_sweep:
        print("\n💡 Tip: Run './agbot-diagnostic.py full' for deep topic inspection.")

    print("\n" + "="*45)
    print("📊 DIAGNOSTIC COMPLETE")
    print("="*45)
