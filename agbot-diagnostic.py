#!/usr/bin/env python3
import os
import subprocess
import re
import sys
import curses

VERSION = "6.2-SOWBOT"

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

EXPECTED_NODES = [
    "/controller", 
    "/usb_cam", 
    "/UI_NODE", 
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
        
        stdscr.addstr(0, 0, f"🚀 MISSION CONTROL - {VERSION}", curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, f"Host: {'Jetson' if cfg['IS_JETSON'] == 'true' else 'PC'}", curses.color_pair(4))
        stdscr.addstr(2, 0, "="*65, curses.color_pair(3))

        stdscr.addstr(4, 0, "HARDWARE MAPPING", curses.A_UNDERLINE)
        hw_labels = [("GPS", cfg["GPS"]), ("MCU", cfg["MCU"])]
        for i, (label, port) in enumerate(hw_labels):
            status = "[VIRTUAL]" if "virtual" in port.lower() else "[PHYSICAL]"
            col = curses.color_pair(4) if "virtual" in port.lower() else curses.color_pair(1)
            stdscr.addstr(5+i, 2, f"{label:<4} {port:<18}: {status}", col)

        stdscr.addstr(8, 0, "ACTIVE NODES", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in nodes_list
            status = "[ACTIVE]" if exists else "[OFFLINE]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(9+i, 2, f"{node:<28}: {status}", col)

        stdscr.addstr(15, 0, "DATA STREAMS", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            exists = topic in topics_list
            status = "[FLOWING]" if exists else "[NO DATA]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(16+i, 2, f"{label:<14}: {status}", col)

        stdscr.addstr(22, 0, "Press 'q' for POST-FLIGHT AUDIT", curses.color_pair(4))
        stdscr.refresh()
        if stdscr.getch() == ord('q'):
            break

if __name__ == "__main__":
    do_full_sweep = len(sys.argv) > 1 and sys.argv[1].lower() == "full"

    try:
        curses.wrapper(draw)
    except KeyboardInterrupt:
        pass
    
    # --- THIS SECTION WAS MISSING IN YOUR PREVIOUS RUN ---
    print("\n" + "═"*75)
    print(f"🔎 VERBOSE ROS 2 GRAPH AUDIT ({VERSION})")
    print("═"*75)

    nodes_raw = run_cmd("ros2 node list")
    found_nodes = [n for n in nodes_raw.split('\n') if n]
    
    for node in found_nodes:
        print(f"\n● {node.upper()}")
        node_info = run_cmd(f"ros2 node info {node}")
        
        pubs_match = re.search(r'Publishers:(.*?)Service Servers:', node_info, re.S)
        if pubs_match:
            clean_pubs = [p.strip() for p in pubs_match.group(1).split('\n') if '/' in p]
            print(f"  └─ Publishers  : {', '.join(clean_pubs[:5])}")

    if do_full_sweep:
        print("\n" + "═"*75)
        print("📡 DATA FLOW CHECK (1.5s Samples)")
        print("═"*75)
        
        all_topics = run_cmd("ros2 topic list").split('\n')
        for t in all_topics:
            if not t or "parameter" in t: continue
            data = run_cmd(f"timeout 1.5s ros2 topic echo {t} --once --no-arr")
            status = "✅ [ACTIVE]" if data else "❌ [SILENT]"
            print(f"{status} {t:<35}")
    else:
        print(f"\n💡 Tip: Run './agbot-diagnostic.py full' to see data flow.")
