import os
from glob import glob

from setuptools import setup

package_name = 'devkit_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zauberzeug GmbH',
    maintainer_email='ros@zauberzeug.com',
    description='Example UI using NiceGUI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_node = devkit_ui.ui_node:main',
        ],
    },
)
