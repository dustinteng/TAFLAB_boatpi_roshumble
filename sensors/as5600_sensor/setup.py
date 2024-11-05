from setuptools import find_packages, setup

package_name = 'as5600_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy', 'smbus2'],
    zip_safe=True,
    maintainer='boat',
    maintainer_email='boat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['as5600_node = as5600_sensor.as5600_node:main'
        ],
    },
)
