from setuptools import find_packages, setup

package_name = 'test_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # Registers the package in ament_index
        ('share/' + package_name, ['package.xml']),  # Installs package.xml
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='A test publisher node for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_publisher = test_publisher.publisher:main',
            'visualizer    = test_publisher.visualizer:main',
        ],
    },
)
