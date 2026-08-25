from setuptools import find_packages, setup

package_name = 'devkit_f2c_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zauberzeug GmbH',
    maintainer_email='ros@zauberzeug.com',
    description='Field2Cover planning core (projection + swath generation), no UI dependency',
    license='MIT License',
    # No console_scripts — this is a library package, imported by devkit_ui
    # and the terrain-aware pipeline, not run as a node.
)
