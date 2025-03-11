from setuptools import setup

package_name = 'boat_logic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Required for ROS 2 packages
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'pyserial', 'digi-xbee'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Main logic node for the boat',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'main_logic_node = boat_logic.main_logic_node:main',
        ],
    },
)


