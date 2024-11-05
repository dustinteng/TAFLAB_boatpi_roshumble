from setuptools import setup

package_name = 'control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Control package for boat actuators',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'boat_control = control_py.boat_control_node:main',
        ],
    },
)
