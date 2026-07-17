
from setuptools import find_packages, setup

package_name = 'devkit_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'geometry_msgs', 'sensor_msgs'],
    zip_safe=True,
    maintainer='Zauberzeug GmbH',
    maintainer_email='ros@zauberzeug.com',
    description='ROS2 setup for the Feldfreund DevKit',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'devkit_driver_node = devkit_driver.devkit_driver_node:main'
        ],
    },
)
