#!/usr/bin/env python3
import os
import subprocess
import sys
import signal
import time
import shutil
from pathlib import Path
from typing import List, Dict

class DevkitManager:
    def __init__(self):
        self.image_name = 'sowbot:jazzy'
        self.container_name = 'sowbot_runtime'
        self.root_dir = Path(__file__).parent.resolve()
        self.src_dir = self.root_dir / 'src'
        signal.signal(signal.SIGINT, self._handle_exit)

    def _log(self, msg: str, level: str = "INFO"):
        print(f"[{time.strftime('%H:%M:%S')}] [{level}] {msg}")

    def _handle_exit(self, signum, frame):
        self._log("Shutdown signal received. Stopping container...", "WARN")
        subprocess.run(['docker', 'stop', self.container_name], capture_output=True)
        sys.exit(0)

    def sync_workspace(self):
        """Standardizes the workspace by mirroring EVERYTHING from src/ to root."""
        if not self.src_dir.exists():
            self._log("src/ directory not found!", "ERROR")
            return

        # 1. Identify everything currently in src/
        current_packages = [d.name for d in self.src_dir.iterdir() if d.is_dir() and not d.name.startswith('.')]
        
        # 2. Sync each one
        for pkg in current_packages:
            pkg_src = self.src_dir / pkg
            pkg_root = self.root_dir / pkg
            
            # Remove stale symlinks or folders at the root level (the Docker context area)
            if pkg_root.exists():
                if pkg_root.is_symlink():
                    pkg_root.unlink()
                else:
                    shutil.rmtree(pkg_root)
            
            self._log(f"Syncing {pkg} to build context...")
            shutil.copytree(pkg_src, pkg_root)

        # 3. Cleanup: Remove folders at root that no longer exist in src/
        for item in self.root_dir.iterdir():
            if item.is_dir() and item.name not in current_packages:
                # Protection: Don't delete standard project folders
                if item.name not in ['src', 'docker', 'build', 'install', 'log', '.git', '__pycache__']:
                    self._log(f"Cleaning up ghost package: {item.name}", "DEBUG")
                    shutil.rmtree(item)

    def build(self, full_clean: bool = False):
        """Builds the Docker image."""
        self.sync_workspace()
        build_cmd = ['docker', 'build', '-t', self.image_name, '-f', 'docker/Dockerfile', '.']
        
        if full_clean:
            self._log("Full rebuild requested: Purging host artifacts and cache...", "WARN")
            build_cmd.insert(2, '--no-cache')
            for d in ['build', 'install', 'log']: 
                shutil.rmtree(self.root_dir / d, ignore_errors=True)

        if subprocess.run(build_cmd).returncode != 0:
            self._log("Build failed.", "ERROR"); sys.exit(1)

    def _get_env_config(self) -> Dict[str, str]:
        env_file = self.root_dir / '.env'
        if not env_file.exists(): return {}
        return {k.strip(): v.strip() for line in env_file.read_text().splitlines() 
                if '=' in line and not line.startswith('#') for k, v in [line.split('=', 1)]}

    def run(self, extra_args: List[str]):
        """Runs the stack."""
        if (self.root_dir / 'fixusb.py').exists():
            subprocess.run(['python3', 'fixusb.py'], check=True)

        cfg = self._get_env_config()
        r_port = cfg.get('GPS_PORT_ROVER', 'virtual')
        mcu_port = cfg.get('MCU_PORT', 'virtual')
        is_sim = 'true' if (r_port == 'virtual' and mcu_port == 'virtual') else 'false'

        # Wake MCU if hardware is present
        if is_sim == 'false' and Path(mcu_port).exists():
            self._log(f"Waking MCU on {mcu_port}")
            os.system(f"stty -F {mcu_port} 115200 && (echo 's' > {mcu_port} &)")

        ros_command = (
            "source /opt/ros/jazzy/setup.bash && "
            "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && "
            f"ros2 launch devkit_launch devkit.launch.py sim:={is_sim} rover_port:={r_port} mcu_port:={mcu_port} " +
            " ".join(extra_args)
        )

        docker_cmd = [
            'docker', 'run', '-it', '--rm', '--name', self.container_name, '--net=host', '--privileged',
            '--env', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
            '--env', 'PYTHONPATH=/root/.lizard:/workspace/install/lib/python3.12/site-packages',
            '--env-file', str(self.root_dir / '.env') if (self.root_dir / '.env').exists() else '/dev/null',
            '-v', '/dev:/dev', '-v', f'{self.root_dir}:/workspace/host_root:ro',
            self.image_name, 'bash', '-c', ros_command
        ]

        self._log(f"Runtime active. Sim: {is_sim.upper()}")
        subprocess.run(docker_cmd)

if __name__ == '__main__':
    manager = DevkitManager()
    action = sys.argv[1] if len(sys.argv) > 1 else 'up'
    if action == 'build': manager.build(full_clean=False)
    elif action == 'full-build': manager.build(full_clean=True)
    else: manager.run(sys.argv[2:] if action == 'up' else sys.argv[1:])
