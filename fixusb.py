#!/usr/bin/env python3
import os
import platform
import re
import subprocess
from pathlib import Path

import serial.tools.list_ports

# Official Hardware IDs
UBLOX_VID = 0x1546  # u-blox AG (F9P)
ESP32_VID = 0x303a  # Espressif Systems (ESP32-S3 MCU)
SEP_VID   = 0x1513  # Septentrio (Alternative GPS)
AXIS_IP   = '192.168.42.3'

def check_host_tools():
    """Verify required host utilities are installed."""
    tools = {"setserial": "setserial", "fuser": "psmisc"}
    missing = [p for t, p in tools.items() if subprocess.call(['which', t], stdout=subprocess.DEVNULL) != 0]
    if missing:
        print(f'Warning: Missing host tools. Run: sudo apt update && sudo apt install -y {" ".join(missing)}')
    return len(missing) == 0

def get_existing_safety_ack():
    """Check if the safety acknowledgement already exists in the environment."""
    env_path = Path('.env')
    if not env_path.exists():
        return 'false'
    with open(env_path, 'r') as f:
        for line in f:
            if line.startswith('SOWBOT_SAFETY_ACK='):
                return line.strip().split('=')[1].lower()
    return 'false'

def handle_safety_disclaimer():
    """Displays the legally binding warning and requests first-time acknowledgement."""
    if get_existing_safety_ack() == 'true':
        return 'true'

    print("\033[91m" + "!"*70)
    print("CRITICAL SAFETY WARNING & LIABILITY DISCLAIMER")
    print("!"*70 + "\033[0m")
    print("THIS SOFTWARE COULD CONTROL PHYSICAL HARDWARE CAPABLE OF PRODUCING")
    print("SIGNIFICANT KINETIC FORCE.")
    print("\n1. EXPERIMENTAL STATUS: This branch ('sowbot') contains experimental code")
    print("   generated and refined with AI assistance. It has NOT undergone")
    print("   full-scale field validation.")
    print("2. STATUTORY NOTICE (UK): Usage is at the user's sole risk. Users are")
    print("   reminded that operating agricultural robotics requires a professional")
    print("   duty of care.")
    print("3. MANDATORY HARDWARE SAFETY: This software MUST NOT be used to control")
    print("   a robot without an independent, hard-wired, physical Emergency Stop.")
    print("4. NO LIABILITY: To the extent permitted by the laws of England and Wales,")
    print("   the contributors exclude all liability for property damage, crop loss,")
    print("   or indirect consequential damages.")
    print("-" * 70)

    ack = input("\033[1mType 'I ACCEPT' to acknowledge these terms and proceed: \033[0m")

    if ack == "I ACCEPT":
        print("\033[92mAcknowledgement recorded.\033[0m\n")
        return 'true'
    else:
        print("\033[91mAcknowledgement failed. Motors will be disabled.\033[0m\n")
        return 'false'

def sanitize_hardware(port, baud):
    """The 'Sane' Reset: Kills zombies and fixes ASIO latency."""
    if not port or port == 'virtual' or not Path(port).exists():
        return
    print(f'Sanitizing {port} at {baud}...')
    try:
        subprocess.run(['sudo', 'fuser', '-k', port], stderr=subprocess.DEVNULL)
        subprocess.run(['sudo', 'setserial', port, 'low_latency'], stderr=subprocess.DEVNULL)
        subprocess.run(['sudo', 'stty', '-F', port, baud, 'raw', '-echo'], stderr=subprocess.DEVNULL)
        subprocess.run(['sudo', 'chmod', '666', port], stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f'Warning during sanitization of {port}: {e}')

def find_usb_bus_path(vid: int) -> str:
    """
    Find the /dev/bus/usb/BUS/DEVICE path for a device by vendor ID.
    ublox_dgnss uses libusb directly and requires this path, not /dev/ttyACMx.
    Device numbers change on reconnect so this must be detected dynamically.
    Returns 'virtual' if not found.
    """
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        vid_hex = f'{vid:04x}'
        for line in result.stdout.splitlines():
            if vid_hex in line.lower():
                m = re.match(r'Bus (\d+) Device (\d+)', line)
                if m:
                    path = f'/dev/bus/usb/{m.group(1)}/{m.group(2)}'
                    subprocess.run(['sudo', 'chmod', '666', path], stderr=subprocess.DEVNULL)
                    return path
    except Exception as e:
        print(f'Warning: USB bus path detection failed: {e}')
    return 'virtual'


def detect_camera() -> tuple[str, str]:
    """Detect the preferred camera device node using v4l2-ctl.

    Strategy:
      1. Run ``v4l2-ctl --list-devices`` and parse the output.
      2. Prefer any device whose human-readable name contains "HiCamera"
         (case-insensitive). Use the *first* /dev/videoN listed under that
         entry — that is always the capture node; the second entry is the
         metadata node.
      3. Fall back to the first /dev/videoN listed under any "Integrated
         Camera" entry.
      4. Final fallback: /dev/video0 if v4l2-ctl is unavailable or returns
         nothing useful.

    Returns (device_path, label) where label is a human-readable string for
    the .env comment and startup log, e.g. "HiCamera /dev/video2" or
    "Integrated /dev/video0".

    v4l2-ctl output format (relevant excerpt):
        HiCamera: UVC Camera (usb-0000:00:14.0-2):
            /dev/video2
            /dev/video3
            /dev/media1
        Integrated Camera: Integrated C (usb-0000:00:14.0-8):
            /dev/video0
            /dev/video1
            /dev/media0

    The parser collects /dev/videoN nodes under each named heading and picks
    the first one, which is always the capture interface.
    """
    hicamera_dev = None
    integrated_dev = None

    try:
        result = subprocess.run(
            ['v4l2-ctl', '--list-devices'],
            capture_output=True, text=True, timeout=5)
        output = result.stdout

        current_is_hi = False
        current_is_integrated = False

        for raw_line in output.splitlines():
            line = raw_line.strip()

            if not line:
                current_is_hi = False
                current_is_integrated = False
                continue

            if line.startswith('/dev/video'):
                dev = line.split()[0]
                if current_is_hi and hicamera_dev is None:
                    hicamera_dev = dev
                elif current_is_integrated and integrated_dev is None:
                    integrated_dev = dev
                # Skip /dev/media* lines — they don't start with /dev/video
                continue

            # Heading line — determine camera type for the block that follows.
            lower = line.lower()
            current_is_hi = 'hicamera' in lower
            current_is_integrated = (not current_is_hi) and 'integrated' in lower

    except FileNotFoundError:
        print('Warning: v4l2-ctl not found; camera detection unavailable.')
    except subprocess.TimeoutExpired:
        print('Warning: v4l2-ctl timed out during camera detection.')
    except Exception as e:
        print(f'Warning: camera detection failed: {e}')

    if hicamera_dev:
        print(f'Found HiCamera: {hicamera_dev} (preferred)')
        return hicamera_dev, f'HiCamera {hicamera_dev}'
    if integrated_dev:
        print(f'HiCamera not found; using Integrated Camera: {integrated_dev}')
        return integrated_dev, f'Integrated {integrated_dev}'

    print('Warning: no camera detected by v4l2-ctl; defaulting to /dev/video0')
    return '/dev/video0', 'fallback /dev/video0'


def scan_and_export():
    print('Scanning for Open Agbot Hardware...')
    check_host_tools()

    # Check/Request Safety Acknowledgement
    safety_ack = handle_safety_disclaimer()

    arch = platform.machine()
    is_jetson = (arch == 'aarch64')
    ports = serial.tools.list_ports.comports()

    gnss_found = []
    mcu_device = None

    for p in ports:
        if p.vid in [UBLOX_VID, SEP_VID]:
            g_type = 'ublox' if p.vid == UBLOX_VID else 'septentrio'
            g_serial = p.serial_number or ''
            gnss_found.append((p.device, g_type, g_serial, p.vid))
            print(f'Found {g_type.upper()} GPS: {p.device} (serial={g_serial or "unknown"})')
        elif p.vid == ESP32_VID:
            mcu_device = p.device
            print(f'Found ESP32 MCU: {mcu_device}')

    if not mcu_device and is_jetson:
        mcu_device = '/dev/ttyTHS0'
        print(f'Using Jetson Header MCU: {mcu_device}')

    # ── Deterministic receiver assignment ────────────────────────────────────
    # USB enumeration order is NOT stable across reboots or reconnects.
    # Assign rover/base by serial number when known; fall back to order only
    # when serials are absent (iSerial=0 devices).
    #
    # Priority:
    #   1. If GPS_SERIAL_ROVER / GPS_SERIAL_ROVER1 are already set in the
    #      environment (e.g. from a previous run or manual override), respect them.
    #   2. Match enumerated devices to those serials.
    #   3. If no serials are set and both devices have iSerial=0, assign by order
    #      but WARN loudly so the operator knows assignment may be wrong.

    env_serial_rover  = os.environ.get('GPS_SERIAL_ROVER',  '')
    env_serial_rover1 = os.environ.get('GPS_SERIAL_ROVER1', '')

    # Separate ublox and septentrio devices
    ublox_devs = [(dev, ser, vid) for dev, typ, ser, vid in gnss_found if typ == 'ublox']
    sep_devs   = [(dev, ser, vid) for dev, typ, ser, vid in gnss_found if typ == 'septentrio']

    def pick_ublox(prefer_serial: str, remaining: list):
        """Return (device, serial, vid), rest. prefer_serial='' means take first."""
        if prefer_serial:
            for i, (dev, ser, vid) in enumerate(remaining):
                if ser == prefer_serial:
                    return (dev, ser, vid), remaining[:i] + remaining[i+1:]
            print(f'WARNING: GPS_SERIAL={prefer_serial!r} not found in enumerated devices!')
        if remaining:
            return remaining[0], remaining[1:]
        return None, remaining

    rover_dev, ublox_devs  = pick_ublox(env_serial_rover,  ublox_devs)
    rover1_dev, ublox_devs = pick_ublox(env_serial_rover1, ublox_devs)

    # Check for zero-serial ambiguity
    zero_serial_count = sum(1 for _, s, _ in
                            ([rover_dev] if rover_dev else []) +
                            ([rover1_dev] if rover1_dev else [])
                            if not s)
    if zero_serial_count > 1:
        print('WARNING: Multiple u-blox receivers with iSerial=0 detected.')
        print('         Receiver assignment is by enumeration order and may be wrong')
        print('         on reconnect. Set GPS_SERIAL_ROVER / GPS_SERIAL_ROVER1 env')
        print('         vars or flash unique serial numbers via u-center.')

    # Unpack results
    if rover_dev:
        r_port, r_serial, r_vid = rover_dev
        r_type = 'ublox'
    elif sep_devs:
        r_port, r_serial, r_vid = sep_devs.pop(0)
        r_type = 'septentrio'
    else:
        r_port, r_type, r_serial, r_vid = 'virtual', 'none', '', 0

    if rover1_dev:
        r1_port, r1_serial, r1_vid = rover1_dev
        r1_type = 'ublox'
    elif sep_devs:
        r1_port, r1_serial, r1_vid = sep_devs.pop(0)
        r1_type = 'septentrio'
    else:
        r1_port, r1_type, r1_serial, r1_vid = 'virtual', 'none', '', 0
    mcu_p = mcu_device if mcu_device else 'virtual'

    # Resolve USB bus paths for ublox receivers (libusb requires bus path, not tty)
    r_usb_path  = find_usb_bus_path(r_vid)  if r_type  == 'ublox' else 'virtual'
    r1_usb_path = find_usb_bus_path(r1_vid) if r1_type == 'ublox' else 'virtual'

    if r_type == 'ublox' and r_usb_path != 'virtual':
        print(f'Resolved USB bus path for Rover:  {r_usb_path}')
    if r1_type == 'ublox' and r1_usb_path != 'virtual':
        print(f'Resolved USB bus path for Rover1: {r1_usb_path}')

    # Sanitize serial ports (still needed for baud/latency even if ublox uses libusb)
    sanitize_hardware(r_port,  '460800')
    sanitize_hardware(r1_port, '460800')
    sanitize_hardware(mcu_p,   '115200')

    # ── Camera detection ─────────────────────────────────────────────────────
    # Prefer HiCamera (external crop-row camera); fall back to integrated.
    # USB_CAM_DEVICE carries the /dev/videoN path for the launch file.
    # USB_CAM_ENABLED stays true whenever any /dev/videoN exists (unchanged
    # semantics — it gates the usb_cam node in docker-compose).
    cam_device, cam_label = detect_camera()
    usb_cam = 'true' if Path(cam_device).exists() else 'false'

    # Axis camera — optional network device, short timeout
    try:
        axis_cam = 'true' if subprocess.run(
            ['ping', '-c', '1', '-W', '1', AXIS_IP],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2
        ).returncode == 0 else 'false'
    except (subprocess.TimeoutExpired, OSError):
        axis_cam = 'false'

    with open('.env', 'w') as f:
        f.write('# Auto-generated by fixusb.py\n')
        f.write(f'SOWBOT_SAFETY_ACK={safety_ack}\n')
        f.write(f'GPS_PORT_ROVER={r_port}\n')
        f.write(f'GPS_USB_PATH_ROVER={r_usb_path}\n')    # libusb path for ublox_dgnss node
        f.write(f'GPS_TYPE_ROVER={r_type}\n')
        f.write(f'GPS_SERIAL_ROVER={r_serial}\n')
        f.write(f'GPS_PORT_ROVER1={r1_port}\n')
        f.write(f'GPS_USB_PATH_ROVER1={r1_usb_path}\n')  # libusb path for ublox_dgnss node
        f.write(f'GPS_TYPE_ROVER1={r1_type}\n')
        f.write(f'GPS_SERIAL_ROVER1={r1_serial}\n')
        f.write(f'MCU_PORT={mcu_p}\n')
        f.write(f'USB_CAM_ENABLED={usb_cam}\n')
        f.write(f'USB_CAM_DEVICE={cam_device}\n')        # /dev/videoN for crop-row camera
        f.write(f'AXIS_CAM_ENABLED={axis_cam}\n')
        f.write(f'USER_ID={os.getuid()}\n')
        f.write(f'GROUP_ID={os.getgid()}\n')
        f.write(f'IS_JETSON={"true" if is_jetson else "false"}\n')
        f.write('TMAP2_FILE=/workspace/maps/mixed_test_map\n')
        # NTRIP: set to true once config/ntrip.yaml is filled in with real creds.
        # Keeps corrections disabled by default so a misconfigured ntrip.yaml
        # doesn't spam connection errors on every boot.
        existing_ntrip = os.environ.get('NTRIP_ENABLED', 'false')
        f.write(f'NTRIP_ENABLED={existing_ntrip}\n')

    print(f'\nConfiguration Exported to .env')
    print(f'Safety Ack: {safety_ack} | MCU: {mcu_p}')
    print(f'Rover:  {r_port}  usb={r_usb_path}  serial={r_serial or "none"}  type={r_type}')
    print(f'Rover1: {r1_port}  usb={r1_usb_path}  serial={r1_serial or "none"}  type={r1_type}')
    print(f'Camera: {cam_label}  enabled={usb_cam}')

if __name__ == '__main__':
    scan_and_export()
