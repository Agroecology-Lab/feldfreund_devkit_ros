from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sowbot_row_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agroecology Lab Ltd',
    maintainer_email='sam@agroecologylab.com',
    description='Monocular RGB crop-row following for Sowbot',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crop_row_node = sowbot_row_follow.crop_row_node:main',
            'limbic_row_follow = sowbot_row_follow.limbic_row_follow_node:main',
        ],
    },
)
