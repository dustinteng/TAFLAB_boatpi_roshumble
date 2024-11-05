from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'boat_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ros2 package to be discoverable
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include package.xml file
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('boat_launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boat',
    maintainer_email='boat@todo.todo',
    description='Package to launch sensors and actuators',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # No console scripts needed unless you have executable nodes here
        ],
    },
)
