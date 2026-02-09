from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'devkit_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This ensures any launch files actually in this package are installed
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # This ensures any config files actually in this package are installed
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    description='Agroecology Lab Devkit Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # FORMAT: 'executable_name = package_name.file_name:main_function'
            'devkit_driver_node = devkit_driver.devkit_driver_node:main',
        ],
    },
)
