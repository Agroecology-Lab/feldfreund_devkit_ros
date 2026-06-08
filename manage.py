#!/usr/bin/env python3
import os
import subprocess
import sys
import signal
import time
import shutil
from pathlib import Path
from typing import List, Dict

# The Limbic <-> Neo direct crossover-Ethernet link. A host is treated as
# "on the robot" only if it owns one of these exact addresses; otherwise DDS
# falls back to loopback. Order matters: index 0 = Limbic, index 1 = Neo.
CROSSOVER_PEERS = ('192.168.10.1', '192.168.10.2')

# CycloneDDS config is assembled at runtime (see _cyclonedds_uri):
#   - sim, or any dev box not on the crossover link -> loopback (local-only).
#   - real Limbic/Neo hardware -> peer-to-peer config bound to the crossover
#     interface, multicast off (no switch/router on that link).
# Set DDS_INTERFACE in .env to force the peer-to-peer config onto a named
# interface (see docs/network-setup.md).


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

    def build(self, full_clean: bool = False, sim: bool = False):
        """Builds the Docker image."""
        build_cmd = ['docker', 'build', '-t', self.image_name, '-f', 'docker/Dockerfile', '.']

        if full_clean:
            self._log("Full rebuild requested: Purging Docker cache...", "WARN")
            build_cmd.insert(2, '--no-cache')

        if sim:
            build_cmd += ['--build-arg', 'INSTALL_SIM=true']

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

    def _find_crossover_interface(self) -> str:
        """Returns the interface that owns a CROSSOVER_PEERS address, else None.

        This is how we tell a real robot host from a dev box: the peer-to-peer
        DDS config only functions for a host whose own IP is in the peer list.
        """
        try:
            result = subprocess.run(['ip', '-4', '-o', 'addr', 'show'],
                                    capture_output=True, text=True)
            if result.returncode != 0:
                return None
            for line in result.stdout.splitlines():
                # e.g. "3: end0    inet 192.168.10.1/24 brd ... scope global end0"
                parts = line.split()
                if len(parts) >= 4 and parts[2] == 'inet':
                    if parts[3].split('/')[0] in CROSSOVER_PEERS:
                        return parts[1]
        except Exception:
            pass
        return None

    def _loopback_dds_uri(self) -> str:
        """Single-host DDS config: discovery over loopback multicast.

        Always valid — every node runs in one container, so loopback reaches
        all of them and needs no real network interface.
        """
        return (
            "<CycloneDDS><Domain><General><Interfaces>"
            "<NetworkInterface name=\"lo\" priority=\"default\" multicast=\"true\"/>"
            "</Interfaces></General><Discovery>"
            "<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>"
            "<ParticipantIndex>auto</ParticipantIndex>"
            "</Discovery></Domain></CycloneDDS>"
        )

    def _crossover_dds_uri(self, iface: str) -> str:
        """Peer-to-peer DDS config bound to the Limbic<->Neo crossover link.

        Multicast disabled; explicit peer list so discovery works without a
        router or switch on the direct Ethernet link.
        """
        peers = "".join(f"<Peer address=\"{ip}\"/>" for ip in CROSSOVER_PEERS)
        return (
            "<CycloneDDS><Domain><General><Interfaces>"
            f"<NetworkInterface name=\"{iface}\" priority=\"default\" multicast=\"false\"/>"
            "</Interfaces></General><Discovery>"
            "<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>"
            "<ParticipantIndex>auto</ParticipantIndex>"
            f"<Peers>{peers}</Peers>"
            "</Discovery></Domain></CycloneDDS>"
        )

    def _cyclonedds_uri(self, cfg: Dict[str, str], is_sim: bool) -> str:
        """Picks the right CycloneDDS config for where we're actually running."""
        if is_sim:
            self._log("Sim mode: CycloneDDS on loopback (local-only DDS).")
            return self._loopback_dds_uri()

        override = (cfg.get('DDS_INTERFACE') or '').strip()
        if override:
            self._log(f"DDS_INTERFACE override: peer-to-peer DDS on '{override}'.")
            return self._crossover_dds_uri(override)

        iface = self._find_crossover_interface()
        if iface:
            self._log(f"Crossover link found on '{iface}': peer-to-peer DDS.")
            return self._crossover_dds_uri(iface)

        self._log("Not on the 192.168.10.0/24 crossover link; "
                  "CycloneDDS falling back to loopback (local-only DDS).", "WARN")
        return self._loopback_dds_uri()

    def _base_docker_cmd(self, env_file: Path, cyclonedds_uri: str,
                         extra_flags: List[str] = None) -> List[str]:
        """Common docker run flags shared by all launch modes.

        extra_flags lets a mode add its own volumes/env vars (e.g. limbic-only
        sim mounts) before the image name.
        """
        cmd = [
            'docker', 'run', '-it', '--rm', '--name', self.container_name,
            '--net=host', '--privileged',
            '--env', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
            '--env', 'PYTHONPATH=/root/.lizard:/workspace/install/lib/python3.12/site-packages',
            '--env', f'DISPLAY={os.environ.get("DISPLAY", ":0")}',
            '--env', 'QT_X11_NO_MITSHM=1',
            '--env', 'GALLIUM_DRIVER=llvmpipe',
            '--env', 'MESA_LOADER_DRIVER_OVERRIDE=llvmpipe',
            '--env', f'CYCLONEDDS_URI={cyclonedds_uri}',
            '--env', 'GZ_SIM_RESOURCE_PATH=/workspace/install/virtual_maize_field/share/virtual_maize_field/models',
            '-v', '/tmp/.X11-unix:/tmp/.X11-unix:rw',
            '--env-file', str(env_file) if env_file.exists() else '/dev/null',
            '-v', '/dev:/dev',
            '-v', f'{self.root_dir}/maps:/workspace/maps',
        ]
        if extra_flags:
            cmd += extra_flags
        cmd += [self.image_name, 'bash', '-c']
        return cmd

    def _ros_source(self) -> str:
        """ROS sourcing preamble — used by all modes."""
        return (
            "source /opt/ros/jazzy/setup.bash && "
            "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi"
        )

    def run(self, extra_args: List[str]):
        """Runs the limbic ROS 2 stack within Docker."""
        env_file = self.root_dir / '.env'
        usb_devices_path = Path('/sys/bus/usb/devices')
        has_usb_hardware = usb_devices_path.exists()

        if not has_usb_hardware:
            self._log("No USB hardware detected. Skipping hardware detection.", "WARN")

        if has_usb_hardware and (self.root_dir / 'fixusb.py').exists():
            # Prep for fixusb.py: bind so it can detect serial ports
            bound_before = self._find_ublox_interfaces()
            if not bound_before:
                ublox_ifaces = []
                for dev in usb_devices_path.iterdir():
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

            # Restore ownership of .env to the actual user
            real_user = os.environ.get('SUDO_USER') or os.environ.get('USER') or os.getlogin()
            if env_file.exists():
                subprocess.run(['sudo', 'chown', f'{real_user}:', str(env_file)], capture_output=True)
                self._log(f"Restored {env_file.name} ownership to {real_user}")

            # Unbind so ublox_dgnss can claim the device via libusb
            ifaces_to_unbind = self._find_ublox_interfaces()
            if ifaces_to_unbind:
                self._log("Unbinding cdc_acm for libusb access...")
                self._cdc_acm_unbind(ifaces_to_unbind)
                time.sleep(0.5)

        cfg = self._get_env_config()
        r_port = cfg.get('GPS_PORT_ROVER', 'virtual')
        mcu_port = cfg.get('MCU_PORT', 'virtual')
        # A detected GPS used to silently flip us out of sim, leaving a sim
        # field with a real-GPS-anchored base_link 27 km off the costmap and
        # no fake_nav2_server (so no virtual robot rendered). FORCE_SIM keeps
        # the sim backend regardless of attached hardware.
        force_sim = '--sim' in extra_args or cfg.get('FORCE_SIM') == '1' \
            or os.environ.get('FORCE_SIM') == '1'
        is_sim = 'true' if (force_sim or (r_port == 'virtual' and mcu_port == 'virtual')) else 'false'

        # Survey origin for the topo map datum. In real-GPS runs this MUST match
        # fusioncore's GNSS reference origin, or the field frame and base_link
        # will disagree by the lat/lon offset. Defaults to 0/0 (sim-safe).
        datum_lat = cfg.get('FIELD_DATUM_LAT', '0.0')
        datum_lon = cfg.get('FIELD_DATUM_LON', '0.0')
        datum_alt = cfg.get('FIELD_DATUM_ALT', '0.0')

        # --sim is our own flag; don't forward it to ros2 launch.
        extra_args = [a for a in extra_args if a != '--sim']

        if is_sim == 'false':
            self._usb_reset_f9p(cfg.get('GPS_USB_PATH_ROVER', 'virtual'))
            if Path(mcu_port).exists():
                self._log(f"Waking MCU on {mcu_port}")
                os.system(f"stty -F {mcu_port} 115200 && (echo 's' > {mcu_port} &)")

        # World + topo generation is sim-only. On real hardware you want your
        # surveyed field map, not a freshly generated maize sim that would
        # overwrite it.
        # World + topo generation is sim-only. Direction: topo map is the
        # source of truth, the Gazebo world is derived from it (plants studded
        # in the inter-row gaps of the saved R*_IN/OUT nodes). On first boot
        # there's no authored map yet, so we bootstrap the node positions from
        # a vmf gt_map.csv via get_maize_topo.py, then immediately rebuild
        # maize.world FROM that map with get_topo_maize_world.py. The vmf
        # generated.world is no longer launched, so we no longer symlink it.
        topo_world = ("/workspace/install/agro_robot_sim/share/"
                      "agro_robot_sim/worlds/maize.world")
        world_gen = (
            # Bootstrap node positions only if no map has been authored yet.
            "([ -f /workspace/maps/maize_map ] || ("
            "ros2 run virtual_maize_field generate_world fre22_task_navigation_mini 2>/dev/null && "
            "python3 /workspace/get_maize_topo.py "
            "--csv /root/.ros/virtual_maize_field/gt_map.csv "
            f"--out /workspace/maps/maize_map --name maize_map --rows 6 "
            f"--lat {datum_lat} --lon {datum_lon} --alt {datum_alt})) && "
            # Derive the Gazebo world from whatever map now exists. rm -f first
            # in case a stale symlink from an older build still occupies the path.
            f"rm -f {topo_world} && "
            "python3 /workspace/get_topo_maize_world.py "
            "--topo /workspace/maps/maize_map "
            f"--out {topo_world} --name maize_field && "
            "cp /workspace/src/devkit_launch/resource/fake_nav2_server.py "
            "/workspace/src/topological_navigation/topological_nav_simulator/topological_nav_simulator/fake_nav2_server.py && "
        ) if is_sim == 'true' else ""

        ros_command = (
            f"{self._ros_source()} && "
            + world_gen +
            f"ros2 launch devkit_launch devkit.launch.py sim:={is_sim} rover_port:={r_port} mcu_port:={mcu_port} " +
            " ".join(extra_args)
        )

        if shutil.which('xhost'):
            subprocess.run(['xhost', '+local:docker'], capture_output=True)
        (self.root_dir / 'maps').mkdir(exist_ok=True)

        # Limbic-only flags: topological map env + get_maize_topo.py mount.
        limbic_flags = [
            '--env', 'TMAP2_FILE=/workspace/maps/maize_map',
            '-v', f'{self.root_dir}/get_maize_topo.py:/workspace/get_maize_topo.py:ro',
            '-v', f'{self.root_dir}/get_topo_maize_world.py:/workspace/get_topo_maize_world.py:ro',
        ]
        cyclonedds_uri = self._cyclonedds_uri(cfg, is_sim == 'true')
        docker_cmd = self._base_docker_cmd(env_file, cyclonedds_uri, limbic_flags) + [ros_command]

        self._log(f"Runtime active. Sim: {is_sim.upper()}")
        subprocess.run(docker_cmd)

    def run_neo(self, extra_args: List[str]):
        """Runs the Neo camera/row-follow stack within Docker.

        Intentionally minimal — no world generation, no GPS, no MCU wakeup.
        Neo only needs: source ROS, launch crop_row_nav.launch.py.
        """
        env_file = self.root_dir / '.env'
        cfg = self._get_env_config()

        ros_command = (
            f"{self._ros_source()} && "
            "ros2 launch devkit_launch neo.launch.py " +
            " ".join(extra_args)
        )

        if shutil.which('xhost'):
            subprocess.run(['xhost', '+local:docker'], capture_output=True)

        # Neo is a hardware stack: peer-to-peer DDS when on the crossover link,
        # loopback fallback when run standalone on a dev box.
        cyclonedds_uri = self._cyclonedds_uri(cfg, is_sim=False)
        docker_cmd = self._base_docker_cmd(env_file, cyclonedds_uri) + [ros_command]

        self._log("Neo runtime active.")
        subprocess.run(docker_cmd)


if __name__ == '__main__':
    manager = DevkitManager()
    action = sys.argv[1] if len(sys.argv) > 1 else 'up'
    sim = '+sim' in sys.argv

    if action == 'build':
        manager.build(full_clean=False, sim=sim)
    elif action == 'full-build':
        manager.build(full_clean=True, sim=sim)
    elif action == 'neo':
        manager.run_neo(sys.argv[2:])
    elif action == 'neo-tsm':
        # Same Neo stack, Triangle Scan Method detector. detector:=tsm is
        # prepended so an explicit detector:= on the command line still wins.
        manager.run_neo(['detector:=tsm'] + sys.argv[2:])
    else:
        # 'up' or bare invocation: pass remaining args (or all argv[1:] if not 'up')
        manager.run(sys.argv[2:] if action == 'up' else sys.argv[1:])
