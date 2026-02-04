#!/usr/bin/env python3
import subprocess
import os
import sys

# Project configuration
IMAGE_NAME = 'feldfreund:jazzy'
CONTAINER_NAME = 'feldfreund_runtime'
ROS_DISTRO = 'jazzy'

def prepare_workspace():
    '''Ensures src exists and heals package structure for ROS 2 Jazzy.'''
    workspace_root = os.getcwd()
    src_dir = os.path.join(workspace_root, 'src')
    packages = ['devkit_driver', 'devkit_launch', 'devkit_ui']

    # Alignment of misplaced launch files
    ui_launch_dir = os.path.join(workspace_root, 'devkit_ui', 'launch')
    misplaced_ui = os.path.join(workspace_root, 'devkit_launch', 'launch', 'ui.launch.py')
    correct_ui = os.path.join(ui_launch_dir, 'ui.launch.py')
    ui_setup_py = os.path.join(workspace_root, 'devkit_ui', 'setup.py')

    if not os.path.exists(ui_launch_dir):
        print(f'Creating directory: {ui_launch_dir}')
        os.makedirs(ui_launch_dir, exist_ok=True)

    if os.path.exists(misplaced_ui) and not os.path.exists(correct_ui):
        print('Moving ui.launch.py to devkit_ui/launch...')
        os.rename(misplaced_ui, correct_ui)

    # Patching setup.py for proper ROS 2 indexing
    if os.path.exists(ui_setup_py):
        with open(ui_setup_py, 'r') as f:
            content = f.read()
        
        old_path = "os.path.join('share', package_name)"
        new_path = "os.path.join('share', package_name, 'launch')"
        
        if old_path in content and new_path not in content:
            print(f'Patching {ui_setup_py}...')
            new_content = content.replace(old_path, new_path)
            with open(ui_setup_py, 'w') as f:
                f.write(new_content)

    # Symlink management for Docker mount
    os.makedirs(src_dir, exist_ok=True)
    for pkg in packages:
        pkg_path = os.path.join(workspace_root, pkg)
        link_path = os.path.join(src_dir, pkg)
        
        if os.path.islink(link_path):
            os.remove(link_path)
        elif os.path.exists(link_path):
            subprocess.run(['sudo', 'rm', '-rf', link_path], check=True)

        if os.path.exists(pkg_path):
            print(f'Linking {pkg} -> src/{pkg}')
            os.symlink(os.path.join('..', pkg), link_path)
        else:
            print(f'Warning: {pkg} not found on host!')

def run_build(full=False):
    '''Builds Docker image and extracts artifacts to the host.'''
    prepare_workspace()
    
    if full:
        print('Purging build artifacts (build/install/log)...')
        subprocess.run(['sudo', 'rm', '-rf', 'build/', 'install/', 'log/'], check=False)
    
    build_cmd = ['docker', 'build', '-t', IMAGE_NAME, '-f', 'docker/Dockerfile', '.']
    if full:
        build_cmd.insert(2, '--no-cache')
    
    try:
        subprocess.run(build_cmd, check=True)
        print('Extracting internal build to host...')
        subprocess.run(['docker', 'rm', '-f', 'temp_extract'], capture_output=True)
        subprocess.run(['docker', 'create', '--name', 'temp_extract', IMAGE_NAME], check=True)
        subprocess.run(['docker', 'cp', 'temp_extract:/workspace/install', '.'], check=True)
        subprocess.run(['docker', 'rm', 'temp_extract'], check=True)
        
        user_info = f'{os.getuid()}:{os.getgid()}'
        subprocess.run(['sudo', 'chown', '-R', user_info, 'install/'], check=True)
        print('Build and Extraction Complete.')
        
    except subprocess.CalledProcessError:
        print('Build failed.')
        sys.exit(1)

def run_runtime(extra_args):
    '''Prepares hardware and executes the ROS 2 runtime environment.'''
    if os.path.exists('fixusb.py'):
        subprocess.run(['python3', 'fixusb.py'], check=True)

    ports = {}
    if os.path.exists('.env'):
        with open('.env', 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    k, v = line.strip().split('=', 1)
                    ports[k.strip()] = v.strip()
    
    # Port retrieval from environment
    r_port  = ports.get('GPS_PORT_ROVER', 'virtual')
    r1_port = ports.get('GPS_PORT_ROVER1', 'virtual')
    mcu_p   = ports.get('MCU_PORT', 'virtual')
    r_type  = ports.get('GPS_TYPE_ROVER', 'none')
    r1_type = ports.get('GPS_TYPE_ROVER1', 'none')

    is_virtual = (r_port == 'virtual' and mcu_p == 'virtual')
    sowbot_enabled = 'true' if any('sowbot:=true' in arg for arg in extra_args) else 'false'

    # Hardware sanitisation
    if not is_virtual:
        for p in [r_port, r1_port, mcu_p]:
            if p and p.startswith('/dev/'):
                subprocess.run(['sudo', 'chmod', '666', p], check=False)
        
        if mcu_p.startswith('/dev/'):
            stty_cmd = f"stty -F {mcu_p} 115200 && (echo 's' > {mcu_p} &)"
            os.system(stty_cmd)

    if not os.path.exists('install/setup.bash'):
        print('Error: install/setup.bash not found. Run ./manage.py build first.')
        return

    sim_flag = 'sim:=true' if is_virtual else 'sim:=false'
    setup_cmd = f'source /opt/ros/{ROS_DISTRO}/setup.bash && source install/setup.bash'
    
    # Docker execution with hardware passthrough
    cmd = [
        'docker', 'run', '-it', '--rm', '--name', CONTAINER_NAME,
        '--net=host', '--privileged',
        '--env-file', '.env' if os.path.exists('.env') else '/dev/null',
        '-v', '/dev:/dev', '-v', f'{os.getcwd()}:/workspace', '-w', '/workspace',
        IMAGE_NAME, '/bin/bash', '-c',
        f'{setup_cmd} && ros2 launch devkit_launch devkit.launch.py '
        f'{sim_flag} '
        f'rover_port:={r_port} rover_type:={r_type} '
        f'rover1_port:={r1_port} rover1_type:={r1_type} '
        f'mcu_port:={mcu_p} sowbot:={sowbot_enabled} {" ".join(extra_args)}'
    ]
    subprocess.run(cmd)

if __name__ == '__main__':
    build_triggers = ['build', 'full-build']
    needs_build = not os.path.exists('install/setup.bash') or any(arg in sys.argv for arg in build_triggers)
    
    if needs_build:
        run_build('full-build' in sys.argv)
    
    if not any(arg in build_triggers for arg in sys.argv) or 'up' in sys.argv:
        launch_args = [arg for arg in sys.argv[1:] if arg not in build_triggers and arg != 'up']
        run_runtime(launch_args)
