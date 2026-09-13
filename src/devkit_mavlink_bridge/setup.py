import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'devkit_mavlink_bridge'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This ensures any launch files actually in this package are installed
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # This ensures any config files actually in this package are installed
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='Agroecology Lab',
    description='MAVLink bridge from Sowbot cmd_vel/localisation to an ArduPilot Rover RTU in GUIDED mode.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # FORMAT: 'executable_name = package_name.file_name:main_function'
            'mavlink_bridge_node = devkit_mavlink_bridge.mavlink_bridge_node:main',
        ],
    },
)
