#!/usr/bin/env python3
import os
import subprocess
import sys
import signal
import time
from pathlib import Path
from typing import List, Dict, Optional

class DevkitManager:
    def __init__(self):
        self.image_name = 'feldfreund:jazzy'
        self.container_name = 'feldfreund_runtime'
        self.root_dir = Path(__file__).parent.resolve()
        self.src_dir = self.root_dir / 'src'
        self.packages = ['devkit_launch', 'devkit_ui', 'devkit_driver']
        
        # Register signal handlers for clean exit
        signal.signal(signal.SIGINT, self._handle_exit)

    def _log(self, msg: str, level: str = "INFO"):
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] [{level}] {msg}")

    def _handle_exit(self, signum, frame):
        self._log("Shutdown signal received. Cleaning up...", "WARN")
        subprocess.run(['docker', 'stop', self.container_name], capture_output=True)
        sys.exit(0)

    def _generate_setup_boilerplate(self, path: Path, name: str):
        """Standardises package identity for colcon indexing and node execution."""
        setup_py = path / 'setup.py'
        
        # Define entry points: critical for resolving 'libexec' directory issues
        entry_points_dict = {'console_scripts': []}
        
        if name == 'devkit_driver':
            entry_points_dict['console_scripts'].append('devkit_driver_node = devkit_driver.devkit_driver_node:main')
        elif name == 'devkit_ui':
            # Updated to match grep result: devkit_ui/ui_node.py
            entry_points_dict['console_scripts'].append('ui_node = devkit_ui.ui_node:main')

        content = f"""from setuptools import find_packages, setup
import os
from glob import glob
package_name = '{name}'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    description='Agroecology Lab Devkit Package',
    license='Apache-2.0',
    entry_points={entry_points_dict},
)"""
        setup_py.write_text(content)
        
        setup_cfg = path / 'setup.cfg'
        cfg_content = f"[develop]\nscript_dir=$base/lib/{name}\n[install]\ninstall_scripts=$base/lib/{name}\n"
        setup_cfg.write_text(cfg_content)
        
        resource_dir = path / 'resource'
        resource_dir.mkdir(exist_ok=True)
        (resource_dir / name).touch()

    def sync_workspace(self):
        """Ensures the host filesystem matches ROS 2 expectations."""
        for pkg in self.packages:
            pkg_root = self.root_dir / pkg
            pkg_src = self.src_dir / pkg
            
            if pkg_src.is_dir() and not pkg_root.exists():
                self._log(f"Syncing {pkg} from src/ to root...")
                subprocess.run(['cp', '-r', str(pkg_src), str(pkg_root)])
            
            if pkg_root.exists():
                self._log(f"Refreshing build glue for {pkg}")
                self._generate_setup_boilerplate(pkg_root, pkg)

    def build(self, full_clean: bool = False):
        """Executes the container build pipeline."""
        try:
            subprocess.run(['docker', 'info'], capture_output=True, timeout=5, check=True)
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            self._log("Docker daemon unresponsive.", "ERROR")
            sys.exit(1)

        self.sync_workspace()
        
        if full_clean:
            self._log("Cleaning Docker artifacts...", "WARN")
            subprocess.run(['docker', 'rm', '-f', self.container_name], capture_output=True, timeout=10)
            subprocess.run(['docker', 'rmi', '-f', self.image_name], capture_output=True, timeout=20)

        self._log(f"Building {self.image_name}...")
        result = subprocess.run(['docker', 'build', '-t', self.image_name, '-f', 'docker/Dockerfile', '.'])
        if result.returncode != 0:
            self._log("Build failed.", "ERROR")
            sys.exit(1)

    def _get_env_config(self) -> Dict[str, str]:
        env_file = self.root_dir / '.env'
        config = {}
        if env_file.exists():
            for line in env_file.read_text().splitlines():
                if '=' in line and not line.startswith('#'):
                    k, v = line.split('=', 1)
                    config[k.strip()] = v.strip()
        return config

    def run(self, extra_args: List[str]):
        """Manages hardware orchestration and container runtime."""
        if (self.root_dir / 'fixusb.py').exists():
            self._log("Optimising USB permissions...")
            subprocess.run(['python3', 'fixusb.py'], check=True)

        cfg = self._get_env_config()
        r_port = cfg.get('GPS_PORT_ROVER', 'virtual')
        mcu_port = cfg.get('MCU_PORT', 'virtual')
        is_sim = 'true' if (r_port == 'virtual' and mcu_port == 'virtual') else 'false'

        if is_sim == 'false' and Path(mcu_port).exists():
            self._log(f"Waking MCU on {mcu_port}")
            os.system(f"stty -F {mcu_port} 115200 && (echo 's' > {mcu_port} &)")

        ros_command = (
            "export PATH=$PATH:/workspace/host_root && "
            "source /opt/ros/jazzy/setup.bash && "
            "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && "
            f"ros2 launch devkit_launch devkit.launch.py sim:={is_sim} rover_port:={r_port} mcu_port:={mcu_port} " +
            " ".join(extra_args)
        )

        docker_cmd = [
            'docker', 'run', '-it', '--rm', '--name', self.container_name,
            '--net=host', '--privileged',
            '--env', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
            '--env', 'PYTHONPATH=/root/.lizard:/workspace/install/lib/python3.12/site-packages',
            '--env-file', str(self.root_dir / '.env') if (self.root_dir / '.env').exists() else '/dev/null',
            '-v', '/dev:/dev',
            '-v', f'{self.root_dir}:/workspace/host_root:ro',
            self.image_name, 'bash', '-c', ros_command
        ]

        self._log(f"Runtime active. Sim: {is_sim.upper()}")
        subprocess.run(docker_cmd)

if __name__ == '__main__':
    manager = DevkitManager()
    if len(sys.argv) > 1:
        action = sys.argv[1]
        if action == 'build':
            manager.build(full_clean=False)
        elif action == 'full-build':
            manager.build(full_clean=True)
        elif action == 'up':
            manager.run(sys.argv[2:])
        else:
            manager.run(sys.argv[1:])
    else:
        manager.run([])
