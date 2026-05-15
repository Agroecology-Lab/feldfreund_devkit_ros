#!/usr/bin/env python3
import os
import subprocess
import re
import sys
import curses

VERSION = "7.0-SOWBOT"

def get_env_config():
    """Reads hardware ports and mode dynamically from .env on the host."""
    config = {"GPS": "NOT_SET", "GPS1": "NOT_SET", "MCU": "NOT_SET", "IS_JETSON": "false"}
    env_path = ".env"
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            content = f.read()
            for key, pat in [("GPS",  r"GPS_PORT_ROVER=(.*)"),
                              ("GPS1", r"GPS_PORT_ROVER1=(.*)"),
                              ("MCU",  r"MCU_PORT=(.*)"),
                              ("IS_JETSON", r"IS_JETSON=(.*)")]:
                m = re.search(pat, content)
                if m: config[key] = m.group(1).strip()
    return config

# Nodes actually present in devkit.launch.py
EXPECTED_NODES = [
    ("/rover/ublox_dgnss",        "F9P rover driver"),
    ("/rover/ublox_nav_sat_fix_hp","HP NavSatFix converter"),
    ("/relposned_heading_shim",    "dual-antenna heading shim"),
    ("/fusioncore",                "UKF odometry fusion"),
    ("/devkit_driver_node",        "Lizard ESP32 bridge"),
    ("/ui_node",                   "NiceGUI cockpit"),
    ("/navsatfix_relay",           "fix relay → /gnss/fix"),
    ("/odom_wheels_relay",         "odom relay → /odom/wheels"),
]

# Topics that must be flowing for the GPS pipeline to be healthy
TARGET_TOPICS = {
    "F9P PVT":        "/rover/ubx_nav_pvt",
    "HP NavSatFix":   "/rover/fix",
    "RELPOSNED":      "/rover/ubx_nav_rel_pos_ned",
    "GNSS fix":       "/gnss/fix",
    "GNSS heading":   "/gnss/heading",
    "Wheel odom":     "/odom",
    "Fusion odom":    "/fusion/odom",
    "Battery":        "/battery_state",
}

# Fix status codes from sensor_msgs/NavSatStatus
FIX_STATUS = {"-1": "NO FIX", "0": "AUTONOMOUS", "1": "SBAS",
              "2": "DGNSS",   "4": "RTK FLOAT",  "5": "RTK FIXED"}

def _sample_fix_quality(run_cmd_fn):
    """Return (status_str, colour_pair) by echoing /gnss/fix once."""
    raw = run_cmd_fn("timeout 3s ros2 topic echo /gnss/fix --once --no-arr 2>/dev/null")
    if not raw:
        return "NO MSG", 2
    m = re.search(r'status:\s*(-?\d+)', raw)
    code = m.group(1) if m else "?"
    label = FIX_STATUS.get(code, f"STATUS={code}")
    colour = {"-1": 2, "0": 4, "1": 4, "2": 4, "4": 4, "5": 1}.get(code, 4)
    return label, colour

def _sample_shim_stats(run_cmd_fn):
    """Return latest published/rejected counts from relposned_heading_shim."""
    raw = run_cmd_fn(
        "timeout 3s ros2 topic echo /rosout --field msg --once 2>/dev/null | grep 'heading shim'")
    if not raw:
        return None
    m = re.search(r'published=(\d+)\s+rejected=(\d+)', raw)
    if m:
        return int(m.group(1)), int(m.group(2))
    return None

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
    stdscr.timeout(2000)
    cfg = get_env_config()

    while True:
        stdscr.clear()
        nodes_list  = run_cmd("ros2 node list")
        topics_list = run_cmd("ros2 topic list")

        row = 0
        stdscr.addstr(row, 0, f"  SOWBOT MISSION CONTROL  {VERSION}", curses.color_pair(3) | curses.A_BOLD)
        row += 1
        host = "Jetson" if cfg["IS_JETSON"] == "true" else "PC"
        stdscr.addstr(row, 0, f"  Host: {host}", curses.color_pair(4))
        row += 1
        stdscr.addstr(row, 0, "─" * 68, curses.color_pair(3))
        row += 1

        # Hardware ports
        stdscr.addstr(row, 0, " HARDWARE", curses.A_UNDERLINE); row += 1
        for label, key in [("Rover GPS", "GPS"), ("Base GPS", "GPS1"), ("MCU", "MCU")]:
            port = cfg[key]
            virtual = "virtual" in port.lower()
            status = "[VIRTUAL]" if virtual else "[PHYSICAL]"
            col = curses.color_pair(4) if virtual else curses.color_pair(1)
            stdscr.addstr(row, 2, f"{label:<10} {port:<22} {status}", col)
            row += 1
        row += 1

        # Node health
        stdscr.addstr(row, 0, " NODES", curses.A_UNDERLINE); row += 1
        for node, desc in EXPECTED_NODES:
            exists = node in nodes_list
            status = "ACTIVE " if exists else "OFFLINE"
            col    = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(row, 2, f"{node:<38} [{status}]  {desc}", col)
            row += 1
        row += 1

        # Topic health
        stdscr.addstr(row, 0, " GPS PIPELINE", curses.A_UNDERLINE); row += 1
        for label, topic in TARGET_TOPICS.items():
            exists = topic in topics_list
            status = "FLOWING" if exists else "SILENT "
            col    = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(row, 2, f"{label:<14} {topic:<38} [{status}]", col)
            row += 1
        row += 1

        # GPS fix quality (sampled inline — takes ~3s on first render, then cached)
        stdscr.addstr(row, 0, " GPS FIX QUALITY", curses.A_UNDERLINE); row += 1
        fix_label, fix_col = _sample_fix_quality(run_cmd)
        base_present = "virtual" not in cfg.get("GPS1", "virtual").lower()
        heading_note = "dual-antenna READY" if base_present else "single F9P — heading needs base"
        stdscr.addstr(row, 2, f"Fix: {fix_label:<16}  Heading: {heading_note}",
                      curses.color_pair(fix_col))
        row += 2

        stdscr.addstr(row, 0, " Press 'q' to exit to audit  |  Ctrl-C to quit",
                      curses.color_pair(4))
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
    print(f"  VERBOSE ROS 2 GRAPH AUDIT ({VERSION})")
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

    # GPS pipeline summary
    print("\n" + "═"*75)
    print("  GPS PIPELINE AUDIT")
    print("═"*75)

    def run_cmd_plain(cmd):
        try:
            src = "source /opt/ros/jazzy/setup.bash && [ -f install/setup.bash ] && source install/setup.bash"
            return subprocess.check_output(f"{src}; {cmd}", shell=True,
                                           executable="/bin/bash", stderr=subprocess.DEVNULL).decode().strip()
        except KeyboardInterrupt:
            raise
        except Exception:
            return ""

    try:
        fix_raw = run_cmd_plain("timeout 4s ros2 topic echo /gnss/fix --once --no-arr 2>/dev/null")
        if fix_raw:
            status_m = re.search(r'status:\s*(-?\d+)', fix_raw)
            code = status_m.group(1) if status_m else "?"
            print(f"  /gnss/fix      status={code} ({FIX_STATUS.get(code, '?')})")
        else:
            print("  /gnss/fix      NO MESSAGE")

        relpos_raw = run_cmd_plain("timeout 4s ros2 topic echo /rover/ubx_nav_rel_pos_ned --once --no-arr 2>/dev/null")
        if relpos_raw:
            rv  = re.search(r'rel_pos_valid:\s*(\S+)', relpos_raw)
            hv  = re.search(r'rel_pos_heading_valid:\s*(\S+)', relpos_raw)
            bl  = re.search(r'rel_pos_length:\s*(\S+)', relpos_raw)
            baseline_m = f"{float(bl.group(1))/100:.2f}m" if bl else "?"
            print(f"  /rover/ubx_nav_rel_pos_ned  rel_pos_valid={rv and rv.group(1)}  "
                  f"heading_valid={hv and hv.group(1)}  baseline={baseline_m}")
            if rv and rv.group(1).lower() == "false":
                print("    NOTE: rel_pos_valid=False is normal with a single F9P (no base station)")
        else:
            print("  /rover/ubx_nav_rel_pos_ned  NO MESSAGE")

        heading_raw = run_cmd_plain("timeout 4s ros2 topic echo /gnss/heading --once --no-arr 2>/dev/null")
        print(f"  /gnss/heading  {'PUBLISHING' if heading_raw else 'SILENT (expected without base F9P)'}")
    except KeyboardInterrupt:
        print("\n  (interrupted)")
        sys.exit(0)

    if do_full_sweep:
        print("\n" + "═"*75)
        print("  DATA FLOW CHECK (1.5s samples)")
        print("═"*75)
        all_topics = run_cmd_plain("ros2 topic list").split('\n')
        try:
            for t in all_topics:
                if not t or "parameter" in t:
                    continue
                data = run_cmd_plain(f"timeout 1.5s ros2 topic echo {t} --once --no-arr")
                status = "[ACTIVE]" if data else "[SILENT]"
                print(f"  {status} {t}")
        except KeyboardInterrupt:
            print("\n  (interrupted)")
    else:
        print(f"\n  Tip: run './agbot-diagnostic.py full' to sample every topic.")
