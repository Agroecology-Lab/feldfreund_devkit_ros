#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

# Project configuration
IMAGE_NAME = 'feldfreund:jazzy'
CONTAINER_NAME = 'feldfreund_runtime'

def log(msg: str):
    print(f'[MNG] {msg}')

def prepare_workspace():
    """Simple symlink management for host-side IDE support (Intellisense)."""
    root = Path.cwd()
    src_dir = root / 'src'
    packages = ['devkit_driver', 'devkit_launch', 'devkit_ui']
    
    src_dir.mkdir(exist_ok=True)
    for pkg in packages:
        pkg_path = root / pkg
        link_path = src_dir / pkg
        if not link_path.exists() and pkg_path.exists():
            log(f"Linking {pkg} for IDE visibility...")
            os.symlink(Path('..') / pkg, link_path)

def run_build(full_clean=False):
    """Builds the Docker image and extracts the bundled install folder."""
    prepare_workspace()
    
    if full_clean:
        log('Purging local build artifacts...')
        subprocess.run(['sudo', 'rm', '-rf', 'build/', 'install/', 'log/'], check=False)
    
    build_cmd = ['docker', 'build', '-t', IMAGE_NAME, '-f', 'docker/Dockerfile', '.']
    if full_clean:
        build_cmd.insert(2, '--no-cache')
    
    try:
        subprocess.run(build_cmd, check=True)
        log('Extracting build artifacts for host Intellisense...')
        subprocess.run(['docker', 'rm', '-f', 'temp_extract'], capture_output=True)
        subprocess.run(['docker', 'create', '--name', 'temp_extract', IMAGE_NAME], check=True)
        subprocess.run(['docker', 'cp', 'temp_extract:/workspace/install', '.'], check=True)
        subprocess.run(['docker', 'rm', 'temp_extract'], check=True)
        
        # Ensure your host user owns the extracted files so you can edit/read them
        subprocess.run(['sudo', 'chown', '-R', f'{os.getuid()}:{os.getgid()}', 'install/'], check=True)
        log('Build complete.')
    except subprocess.CalledProcessError as e:
        log(f'Build failed: {e}')
        sys.exit(1)

def run_runtime(extra_args, sowbot_enabled=False):
    """Executes the runtime with environment isolation and hardware resilience."""
    
    # 1. Environment Isolation: Block host ROS paths from leaking into Docker
    # This prevents the 'librmw_cyclonedds_cpp.so' not found error
    clean_env = {k: v for k, v in os.environ.items() if not k.startswith(('ROS_', 'RMW_', 'AMENT_'))}

    # 2. Hardware Mapping from .env
    env_file = Path('.env')
    ports = {}
    if env_file.exists():
        for line in env_file.read_text().splitlines():
            if '=' in line and not line.startswith('#'):
                k, v = line.strip().split('=', 1)
                ports[k.strip()] = v.strip()

    mcu_p = ports.get('MCU_PORT', 'virtual')
    r_port = ports.get('GPS_PORT_ROVER', 'virtual')
    r1_port = ports.get('GPS_PORT_ROVER1', 'virtual')
    r_type = ports.get('GPS_TYPE_ROVER', 'none')
    r1_type = ports.get('GPS_TYPE_ROVER1', 'none')

    is_virtual = (mcu_p == 'virtual')
    sim_flag = 'sim:=true' if is_virtual else 'sim:=false'

    # 3. Resilient Hardware Prep (The "Anti-Ghost" loop)
    if not is_virtual:
        for p in [r_port, r1_port, mcu_p]:
            if p and p.startswith('/dev/'):
                for i in range(1, 6):
                    if Path(p).exists():
                        subprocess.run(['sudo', 'chmod', '666', p], check=False)
                        break
                    log(f'Waiting for hardware {p} ({i}/5)...')
                    time.sleep(1)
        
        if Path(mcu_p).exists():
            # Wake up Lizard firmware (ESP32)
            subprocess.run(f"stty -F {mcu_p} 115200 && (echo 's' > {mcu_p} &)", shell=True)

    # 4. Construct Command (Leveraging Docker ENTRYPOINT)
    # The Dockerfile ENTRYPOINT handles the ROS plumbing; we just provide the launch command.
    ros_launch_cmd = (
        f"ros2 launch devkit_launch devkit.launch.py "
        f"{sim_flag} sowbot:={'true' if sowbot_enabled else 'false'} "
        f"mcu_port:={mcu_p} rover_port:={r_port} rover_type:={r_type} "
        f"rover1_port:={r1_port} rover1_type:={r1_type} "
        f"{' '.join(extra_args)}"
    )

    docker_cmd = [
        'docker', 'run', '-it', '--rm', '--name', CONTAINER_NAME,
        '--net=host', '--privileged',
        '--env-file', str(env_file) if env_file.exists() else '/dev/null',
        '-v', '/dev:/dev', '-v', f'{os.getcwd()}:/workspace', '-w', '/workspace',
        IMAGE_NAME, '/bin/bash', '-c', ros_launch_cmd
    ]

    log(f"Launching (Sim: {is_virtual}, Sowbot: {sowbot_enabled})")
    try:
        # We pass clean_env to ensure the docker process itself is isolated from host ROS vars
        subprocess.run(docker_cmd, env=clean_env)
    except KeyboardInterrupt:
        log('User-initiated shutdown.')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Feldfreund DevKit Manager')
    parser.add_argument('command', choices=['build', 'up', 'clean'], nargs='?', default='up')
    parser.add_argument('--sowbot', action='store_true', help='Enable sowbot hardware logic')
    parser.add_argument('--full', action='store_true', help='Full clean build with no cache')
    parser.add_argument('extra', nargs=argparse.REMAINDER, help='Extra ROS 2 launch arguments')

    args = parser.parse_args()

    if args.command == 'clean' or args.full:
        run_build(full_clean=True)
    elif args.command == 'build':
        run_build(full_clean=False)
    
    if args.command == 'up' or not sys.argv[1:]:
        if not Path('install/setup.bash').exists():
            log('No install folder found. Initiating build...')
            run_build(full_clean=args.full)
        run_runtime(args.extra, sowbot_enabled=args.sowbot)
