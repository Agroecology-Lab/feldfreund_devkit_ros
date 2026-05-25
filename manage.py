#!/usr/bin/env python3
import os
import subprocess
import sys
import signal
import time
import shutil
from pathlib import Path
from typing import List, Dict

# CycloneDDS: direct crossover Ethernet between Limbic (192.168.10.1) and
# Neo (192.168.10.2) via end0. Multicast disabled; explicit peer list so
# discovery works without a router or switch.
# TODO: verify interface name on first boot — run `ip -br link show | grep -v lo`
#       If it differs from end0, update NetworkInterface name= here and in
#       the nmcli commands in docs/network-setup.md.
CYCLONEDDS_URI = (
    "<CycloneDDS><Domain><General><Interfaces>"
    "<NetworkInterface name=\"end0\" priority=\"default\" multicast=\"false\"/>"
    "</Interfaces></General><Discovery>"
    "<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>"
    "<ParticipantIndex>auto</ParticipantIndex>"
    "<Peers>"
    "<Peer address=\"192.168.10.1\"/>"   # Limbic — TODO: confirm IP
    "<Peer address=\"192.168.10.2\"/>"   # Neo    — TODO: confirm IP
    "</Peers>"
    "</Discovery></Domain></CycloneDDS>"
)


class DevkitManager:
    def __init__(self):
        self.image_name = 'sowbot:jazzy'
        self.container_name = 'sowbot_runtime'
        self.root_dir = Path(__file__).parent.resolve()
        signal.signal(signal.SIGINT, self._handle_exit)

    def _log(self, msg: str, level: str = "INFO"):
        print(f"[{time.strftime('%H:%M:%S')}] [{level}] {msg}")

    def _handle_exit(self, signum, frame):
        self._log("Shutdown signal received. Stopping container...", "WARN")
        subprocess.run(['docker', 'stop', self.container_name], capture_output=True)
        sys.exit(0)

    def build(self, full_clean: bool = False):
        """Builds the Docker image."""
        build_cmd = ['docker', 'build', '-t', self.image_name, '-f', 'docker/Dockerfile', '.']
        if full_clean:
            self._log("Full rebuild requested: Purging Docker cache...", "WARN")
            build_cmd.insert(2, '--no-cache')
        if subprocess.run(build_cmd).returncode != 0:
            self._log("Build failed.", "ERROR")
            sys.exit(1)

    def _get_env_config(self) -> Dict[str, str]:
        env_file = self.root_dir / '.env'
        if not env_file.exists():
            return {}
        return {k.strip(): v.strip() for line in env_file.read_text().splitlines()
                if '=' in line and not line.startswith('#') for k, v in [line.split('=', 1)]}

    def _find_ublox_interfaces(self) -> List[str]:
        """Returns sysfs interface names for bound ublox cdc_acm devices."""
        ifaces = []
        cdc_path = Path('/sys/bus/usb/drivers/cdc_acm')
        if not cdc_path.exists():
            return ifaces
        for entry in cdc_path.iterdir():
            if not entry.is_symlink():
                continue
            vendor_file = entry / 'device' / 'idVendor'
            if not vendor_file.exists():
                parts = entry.name.split(':')
                if len(parts) == 2:
                    vendor_file = Path(f'/sys/bus/usb/devices/{parts[0]}/idVendor')
            try:
                if vendor_file.exists() and vendor_file.read_text().strip() == '1546':
                    ifaces.append(entry.name)
            except Exception:
                pass
        return ifaces

    def _cdc_acm_bind(self, ifaces: List[str]):
        for iface in ifaces:
            subprocess.run(
                ['sudo', 'sh', '-c', f'echo "{iface}" > /sys/bus/usb/drivers/cdc_acm/bind'],
                stderr=subprocess.DEVNULL)

    def _cdc_acm_unbind(self, ifaces: List[str]):
        for iface in ifaces:
            subprocess.run(
                ['sudo', 'sh', '-c', f'echo "{iface}" > /sys/bus/usb/drivers/cdc_acm/unbind'],
                stderr=subprocess.DEVNULL)

    def _usb_reset_f9p(self, usb_path: str):
        """Hard-resets F9P to clear stale libusb state."""
        if not usb_path or usb_path == 'virtual':
            return
        if subprocess.run(['which', 'usbreset'], capture_output=True).returncode != 0:
            self._log("usbreset not found. Install with: sudo apt install usbutils", "WARN")
            return
        self._log(f"USB reset of F9P at {usb_path}...")
        result = subprocess.run(['sudo', 'usbreset', usb_path], capture_output=True, text=True)
        if result.returncode == 0:
            self._log("F9P USB reset complete.")
            time.sleep(1.0)
        else:
            self._log(f"USB reset failed: {result.stderr.strip()}", "WARN")

    def _base_docker_cmd(self, env_file: Path) -> List[str]:
        """Common docker run flags shared by all launch modes."""
        return [
            'docker', 'run', '-it', '--rm', '--name', self.container_name,
            '--net=host', '--privileged',
            '--env', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
            '--env', 'PYTHONPATH=/root/.lizard:/workspace/install/lib/python3.12/site-packages',
            '--env', f'DISPLAY={os.environ.get("DISPLAY", ":0")}',
            '--env', 'QT_X11_NO_MITSHM=1',
            '--env', 'GALLIUM_DRIVER=llvmpipe',
            '--env', 'MESA_LOADER_DRIVER_OVERRIDE=llvmpipe',
            '--env', f'CYCLONEDDS_URI={CYCLONEDDS_URI}',
            '--env', 'GZ_SIM_RESOURCE_PATH=/workspace/install/virtual_maize_field/share/virtual_maize_field/models',
            '-v', '/tmp/.X11-unix:/tmp/.X11-unix:rw',
            '--env-file', str(env_file) if env_file.exists() else '/dev/null',
            '-v', '/dev:/dev',
            '-v', f'{self.root_dir}/maps:/workspace/maps',
            self.image_name, 'bash', '-c',
        ]

    def _ros_source(self) -> str:
        """ROS sourcing preamble — used by all modes."""
        return (
            "source /opt/ros/jazzy/setup.bash && "
            "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi"
        )

    def run(self, extra_args: List[str]):
        """Runs the limbic ROS 2 stack within Docker."""
        env_file = self.root_dir / '.env'

        if (self.root_dir / 'fixusb.py').exists():
            bound_before = self._find_ublox_interfaces()
            if not bound_before:
                ublox_ifaces = []
                for dev in Path('/sys/bus/usb/devices').iterdir():
                    try:
                        if (dev / 'idVendor').read_text().strip() == '1546':
                            ublox_ifaces = [f'{dev.name}:1.0', f'{dev.name}:1.1']
                            break
                    except Exception:
                        pass
                if ublox_ifaces:
                    self._log("Binding cdc_acm for GPS detection...")
                    self._cdc_acm_bind(ublox_ifaces)

            subprocess.run(['sudo', 'python3', 'fixusb.py'], check=True)

            real_user = os.environ.get('SUDO_USER') or os.environ.get('USER') or os.getlogin()
            if env_file.exists():
                subprocess.run(['sudo', 'chown', f'{real_user}:', str(env_file)], capture_output=True)
                self._log(f"Restored {env_file.name} ownership to {real_user}")

            ifaces_to_unbind = self._find_ublox_interfaces()
            if ifaces_to_unbind:
                self._log("Unbinding cdc_acm for libusb access...")
                self._cdc_acm_unbind(ifaces_to_unbind)
                time.sleep(0.5)

        cfg = self._get_env_config()
        r_port = cfg.get('GPS_PORT_ROVER', 'virtual')
        mcu_port = cfg.get('MCU_PORT', 'virtual')
        is_sim = 'true' if (r_port == 'virtual' and mcu_port == 'virtual') else 'false'

        if is_sim == 'false':
            self._usb_reset_f9p(cfg.get('GPS_USB_PATH_ROVER', 'virtual'))
            if Path(mcu_port).exists():
                self._log(f"Waking MCU on {mcu_port}")
                os.system(f"stty -F {mcu_port} 115200 && (echo 's' > {mcu_port} &)")

        ros_command = (
            f"{self._ros_source()} && "
            "ros2 run virtual_maize_field generate_world fre22_task_navigation_mini 2>/dev/null && "
            "ln -sf /root/.ros/virtual_maize_field/generated.world "
            "/workspace/install/agro_robot_sim/share/agro_robot_sim/worlds/maize.world && "
            f"ros2 launch devkit_launch devkit.launch.py sim:={is_sim} rover_port:={r_port} mcu_port:={mcu_port} " +
            " ".join(extra_args)
        )

        subprocess.run(['xhost', '+local:docker'], capture_output=True)
        (self.root_dir / 'maps').mkdir(exist_ok=True)

        docker_cmd = self._base_docker_cmd(env_file) + [ros_command]

        self._log(f"Runtime active. Sim: {is_sim.upper()}")
        subprocess.run(docker_cmd)

    def run_neo(self, extra_args: List[str]):
        """Runs the Neo camera/row-follow stack within Docker.

        Intentionally minimal — no world generation, no GPS, no MCU wakeup.
        Neo only needs: source ROS, launch crop_row_nav.launch.py.
        """
        env_file = self.root_dir / '.env'

        ros_command = (
            f"{self._ros_source()} && "
            "ros2 launch devkit_launch neo.launch.py " +
            " ".join(extra_args)
        )

        subprocess.run(['xhost', '+local:docker'], capture_output=True)

        docker_cmd = self._base_docker_cmd(env_file) + [ros_command]

        self._log("Neo runtime active.")
        subprocess.run(docker_cmd)


if __name__ == '__main__':
    manager = DevkitManager()
    action = sys.argv[1] if len(sys.argv) > 1 else 'up'

    if action == 'build':
        manager.build(full_clean=False)
    elif action == 'full-build':
        manager.build(full_clean=True)
    elif action == 'neo':
        manager.run_neo(sys.argv[2:])
    else:
        # 'up' or bare invocation: pass remaining args (or all argv[1:] if not 'up')
        manager.run(sys.argv[2:] if action == 'up' else sys.argv[1:])
