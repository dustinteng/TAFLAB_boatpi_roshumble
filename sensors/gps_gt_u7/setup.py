from setuptools import find_packages, setup

package_name = 'gps_gt_u7'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'pynmea2'],
    zip_safe=True,
    maintainer='boat',
    maintainer_email='dustinteng12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = gps_gt_u7.gps_node:main',
        ],
    },
)
