#!/usr/bin/env python3
import os
import subprocess
import sys
from pathlib import Path

# Project configuration
IMAGE_NAME = 'feldfreund:jazzy'
CONTAINER_NAME = 'feldfreund_runtime'

def log(msg: str):
    print(f'[MNG] {msg}')

def run_build(full=False):
    '''Builds the image. full-build cleans only project-specific cruft.'''
    if full:
        log("PERFORMING DEEP CLEAN: Purging project images and build cache...")
        # 1. Kill the specific project container if it exists
        subprocess.run(['docker', 'rm', '-f', CONTAINER_NAME], capture_output=True)
        
        # 2. Remove the project image (breaks the cache link)
        subprocess.run(['docker', 'rmi', '-f', IMAGE_NAME], capture_output=True)
        
        # 3. Targeted Cache Purge (Safe for Portainer)
        # Reclaims the GBs of BuildKit layers from previous failed attempts
        subprocess.run(['docker', 'builder', 'prune', '-a', '-f', '--filter', f'label=shared.image={IMAGE_NAME}'], capture_output=False)
        
        # 4. Clean dangling images
        subprocess.run(['docker', 'image', 'prune', '-f'], capture_output=True)
        
    log(f"Building {IMAGE_NAME}...")
    build_cmd = ['docker', 'build', '-t', IMAGE_NAME, '-f', 'docker/Dockerfile', '.']
    
    if full:
        # Force a totally fresh pull and ignore all cache
        build_cmd.extend(['--no-cache', '--pull'])
    
    try:
        subprocess.run(build_cmd, check=True)
        log("Build successful.")
    except subprocess.CalledProcessError:
        log("Build failed.")
        sys.exit(1)

def run_runtime(extra_args):
    '''Hardware prep then launch with explicit environment sourcing.'''
    
    # 1. Hardware Prep
    if Path('fixusb.py').exists():
        log("Running fixusb.py...")
        subprocess.run(['python3', 'fixusb.py'], check=True)

    # 2. Environment/Port Loading
    env_file = Path('.env')
    ports = {}
    if env_file.exists():
        for line in env_file.read_text().splitlines():
            if '=' in line and not line.startswith('#'):
                k, v = line.strip().split('=', 1)
                ports[k.strip()] = v.strip()

    r_port  = ports.get('GPS_PORT_ROVER', 'virtual')
    mcu_p   = ports.get('MCU_PORT', 'virtual')
    is_virtual = (r_port == 'virtual' and mcu_p == 'virtual')
    sim_flag = 'sim:=true' if is_virtual else 'sim:=false'

    # 3. Wake up hardware
    if not is_virtual and Path(mcu_p).exists():
        log(f"Waking up Lizard on {mcu_p}...")
        os.system(f"stty -F {mcu_p} 115200 && (echo 's' > {mcu_p} &)")

    # 4. The Execution Wrapper
    # Sourcing ensures ROS knows where the packages and libraries live
    ros_command = (
        "source /opt/ros/jazzy/setup.bash && "
        "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && "
        f"ros2 launch devkit_launch devkit.launch.py {sim_flag} rover_port:={r_port} mcu_port:={mcu_p} " + 
        " ".join(extra_args)
    
    )

    docker_cmd = [
        'docker', 'run', '-it', '--rm', '--name', CONTAINER_NAME,
        '--net=host', '--privileged',
        '--env', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
        # Ensure Python can see both Lizard and your built ROS packages
        '--env', 'PYTHONPATH=/root/.lizard:/workspace/install/lib/python3.12/site-packages',
        '--env-file', '.env' if env_file.exists() else '/dev/null',
        '-v', '/dev:/dev',
        # MOUNT POINT FIX: 
        # We mount to /workspace/src so it doesn't overlap the /workspace/install folder 
        # created inside the image during the build.
        '-v', f'{os.path.abspath("src")}:/workspace/src:ro',
        IMAGE_NAME, 
        'bash', '-c', ros_command
    ]
    
    log("Launching container...")
    # We do NOT scrub the environment here; we let Docker handle it via --env flags
    subprocess.run(docker_cmd)

if __name__ == '__main__':
    if len(sys.argv) > 1:
        cmd = sys.argv[1]
        if cmd == 'build':
            run_build(full=False)
        elif cmd == 'full-build':
            run_build(full=True)
        elif cmd == 'up':
            run_runtime(sys.argv[2:])
        else:
            run_runtime(sys.argv[1:])
    else:
        run_runtime([])
