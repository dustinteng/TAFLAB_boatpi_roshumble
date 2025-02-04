from setuptools import find_packages, setup

package_name = 'witmotionmag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src') + ['lib', 'lib.data_processor', 'lib.data_processor.roles', 'lib.data_processor.interface', 'lib.protocol_resolver.roles', 'lib.protocol_resolver.interface'],
    package_dir={
        '': 'src',
        'lib': 'lib'  # This makes Python recognize it as a package
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boat',
    maintainer_email='dustinteng12@gmail.com',
    description='ROS2 package for WitMotionMag sensor',
    license='TODO: License declaration',  # Replace with your license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'witmotionmag_node = witmotionmag.witmotionmag_node:main'  # Check for the correct path to your main function
        ],
    },
)
