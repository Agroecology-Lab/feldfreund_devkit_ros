#!/usr/bin/env python3
import subprocess
import os
import sys
import time

# --- CONFIGURATION ---
IMAGE_NAME = "feldfreund:jazzy"
CONTAINER_NAME = "feldfreund_runtime"
ROS_DISTRO = "jazzy"

def prepare_workspace():
    """
    Ensures the 'src' directory exists and contains symlinks to the packages.
    """
    workspace_root = os.getcwd()
    src_dir = os.path.join(workspace_root, "src")
    packages = ["devkit_driver", "devkit_launch", "devkit_ui"]

    if not os.path.exists(src_dir):
        print(f"Creating missing workspace directory: {src_dir}")
        os.makedirs(src_dir, exist_ok=True)

    for pkg in packages:
        pkg_path = os.path.join(workspace_root, pkg)
        link_path = os.path.join(src_dir, pkg)
        
        if os.path.exists(pkg_path):
            # Check if link is missing OR broken
            if not os.path.islink(link_path) or not os.path.exists(link_path):
                if os.path.islink(link_path):
                    os.remove(link_path)  # Remove broken link
                print(f"Linking {pkg} -> src/{pkg}")
                os.symlink(os.path.join("..", pkg), link_path)
        else:
            print(f"Warning: Source package {pkg} not found at {pkg_path}")

def run_build(full=False):
    """Builds the Docker image and syncs artifacts."""
    prepare_workspace()
    if full:
        print("FULL BUILD: Purging all host artifacts (build/install/log)...")
        subprocess.run(['sudo', 'rm', '-rf', 'build/', 'install/', 'log/'], check=False)
    
    print(f"Building Docker Image (Cache={'OFF' if full else 'ON'})...")
    build_cmd = ["docker", "build", "-t", IMAGE_NAME, "-f", "docker/Dockerfile", "."]
    if full: 
        build_cmd.insert(2, "--no-cache")
    
    try:
        subprocess.run(build_cmd, check=True)
    except subprocess.CalledProcessError:
        print("Docker build failed.")
        sys.exit(1)

    print(f"Syncing build artifacts to host ({ROS_DISTRO})...")
    user_info = f"{os.getuid()}:{os.getgid()}"
    setup_cmd = f"source /opt/ros/{ROS_DISTRO}/setup.bash"
    if os.path.exists("install/setup.bash"):
        setup_cmd += " && source install/setup.bash"

    cmd_sync = [
        "docker", "run", "--rm", "--user", user_info,
        "--entrypoint", "/bin/bash",
        "-v", f"{os.getcwd()}:/workspace", "-w", "/workspace",
        IMAGE_NAME, "-c",            
        f"{setup_cmd} && colcon build --symlink-install --executor sequential --continue-on-error"
    ]
    subprocess.run(cmd_sync, check=True)

def run_runtime(extra_args):
    """Hardware prep and launch."""
    if os.path.exists('fixusb.py'):
        subprocess.run(['python3', 'fixusb.py'], check=True)

    ports = {}
    if os.path.exists('.env'):
        with open('.env', 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    key, value = line.strip().split('=', 1)
                    ports[key.strip()] = value.strip()
    
    gps_p = ports.get('GPS_PORT', 'virtual')
    mcu_p = ports.get('MCU_PORT', 'virtual')
    is_virtual = (gps_p == 'virtual' and mcu_p == 'virtual')

    if not is_virtual:
        for p in [gps_p, mcu_p]:
            if p.startswith('/dev/'):
                subprocess.run(['sudo', 'chmod', '666', p], check=False)
        if mcu_p.startswith('/dev/'):
            os.system(f"stty -F {mcu_p} 115200 && (echo 's' > {mcu_p} &)")

    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)
    sim_flag = "sim:=true" if is_virtual else "sim:=false"
    setup_cmd = f"source /opt/ros/{ROS_DISTRO}/setup.bash"
    if os.path.exists("install/setup.bash"):
        setup_cmd += " && source install/setup.bash"

    print(f"Launching Runtime | GPS: {gps_p} | MCU: {mcu_p}")
    cmd = [
        "docker", "run", "-it", "--rm", "--name", CONTAINER_NAME,
        "--entrypoint", "/bin/bash", "--net=host", "--privileged",
        "--env-file", ".env" if os.path.exists('.env') else "/dev/null",
        "-v", "/dev:/dev", "-v", f"{os.getcwd()}:/workspace", "-w", "/workspace",
        IMAGE_NAME, "-c",
	f"{setup_cmd} && ros2 launch devkit_launch devkit.launch.py {sim_flag} "
        f"gps_port:={gps_p} mcu_port:={mcu_p} {' '.join(extra_args)}"
    ]
    subprocess.run(cmd)

if __name__ == "__main__":
    build_triggers = ["build", "full-build"]
    needs_build = not os.path.exists('install/setup.bash') or any(arg in sys.argv for arg in build_triggers)
    
    if needs_build:
        run_build("full-build" in sys.argv)
    
    if not any(arg in build_triggers for arg in sys.argv):
        launch_args = [arg for arg in sys.argv[1:] if arg != "up"]
        run_runtime(launch_args)
