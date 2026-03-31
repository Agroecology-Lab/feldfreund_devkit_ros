#!/usr/bin/env python3
import os
import re
import subprocess
import platform
import serial.tools.list_ports
from pathlib import Path

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
                    # Ensure the container/host can access it
                    subprocess.run(['sudo', 'chmod', '666', path], stderr=subprocess.DEVNULL)
                    return path
    except Exception as e:
        print(f'Warning: USB bus path detection failed: {e}')
    return 'virtual'

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

    r_port,   r_type,  r_serial,  r_vid  = gnss_found[0] if len(gnss_found) > 0 else ('virtual', 'none', '', 0)
    r1_port,  r1_type, r1_serial, r1_vid = gnss_found[1] if len(gnss_found) > 1 else ('virtual', 'none', '', 0)
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

    usb_cam = 'true' if Path('/dev/video0').exists() else 'false'
    axis_cam = 'true' if subprocess.run(['ping', '-c', '1', '-W', '1', AXIS_IP],
                                     stdout=subprocess.DEVNULL).returncode == 0 else 'false'

    with open('.env', 'w') as f:
        f.write('# Auto-generated by fixusb.py\n')
        f.write(f'SOWBOT_SAFETY_ACK={safety_ack}\n')
        f.write(f'GPS_PORT_ROVER={r_port}\n')
        f.write(f'GPS_USB_PATH_ROVER={r_usb_path}\n')   # libusb path for ublox_dgnss node
        f.write(f'GPS_TYPE_ROVER={r_type}\n')
        f.write(f'GPS_SERIAL_ROVER={r_serial}\n')
        f.write(f'GPS_PORT_ROVER1={r1_port}\n')
        f.write(f'GPS_USB_PATH_ROVER1={r1_usb_path}\n') # libusb path for ublox_dgnss node
        f.write(f'GPS_TYPE_ROVER1={r1_type}\n')
        f.write(f'GPS_SERIAL_ROVER1={r1_serial}\n')
        f.write(f'MCU_PORT={mcu_p}\n')
        f.write(f'USB_CAM_ENABLED={usb_cam}\n')
        f.write(f'AXIS_CAM_ENABLED={axis_cam}\n')
        f.write(f'USER_ID={os.getuid()}\n')
        f.write(f'GROUP_ID={os.getgid()}\n')
        f.write(f'IS_JETSON={"true" if is_jetson else "false"}\n')

    print(f'\nConfiguration Exported to .env')
    print(f'Safety Ack: {safety_ack} | MCU: {mcu_p}')
    print(f'Rover:  {r_port}  usb={r_usb_path}  serial={r_serial or "none"}  type={r_type}')
    print(f'Rover1: {r1_port}  usb={r1_usb_path}  serial={r1_serial or "none"}  type={r1_type}')

if __name__ == '__main__':
    scan_and_export()
