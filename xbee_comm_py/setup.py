from setuptools import find_packages, setup

package_name = 'xbee_comm_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['config.json'],  # Include config.json in the package
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',  'digi-xbee'],
    zip_safe=True,
    maintainer='dustinteng',
    maintainer_email='dustinteng@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbee_comm_node = xbee_comm_py.xbee_comm_node:main',
        ],
    },
)
